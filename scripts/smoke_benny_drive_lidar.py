#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Smoke-test Benny in Isaac Sim 6.2: package USD, drive forward, onboard LiDAR.

Loads the canonical USD from ``benny_joints_export_description`` (LiDAR + wheel
friction already authored). Does **not** URDF-import and does **not** write
``scripts/_usd_cache``.

Run from fish (GUI — needed for lidar point-cloud debug draw):

    conda deactivate
    set -e PYTHONPATH
    set -e LD_LIBRARY_PATH

    set -x ISAACSIM_PATH ~/Omniverse/isaacsim-6.2
    $ISAACSIM_PATH/python.sh \\
        /home/oleh-pc-ubuntu/PWR/7_sem/CPSwL/scripts/smoke_benny_drive_lidar.py \\
        --flip-right-wheel

Ground friction (PhysX material on the ground collider). Effective contact μ
is combined with wheel materials (drive wheels ≈ μs=1.0 / μd=0.9 in the USD).

Typical presets (static / dynamic):

    indoor   0.8 / 0.6   — default; tile / linoleum / painted floor
    rubber   1.0 / 0.9   — sticky; matches drive-wheel materials
    concrete 0.9 / 0.7   — warehouse / outdoor slab
    ice      0.15 / 0.08 — slip test

Useful flags:

    --ground-friction-preset indoor|rubber|concrete|ice
    --ground-mu-static 0.8 --ground-mu-dynamic 0.6   # overrides preset
    --duration 30
    --wheel-omega 5
    --flip-right-wheel
    --test
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_USD = (
    REPO_ROOT
    / "benny_joints_export_description"
    / "urdf"
    / "benny_joints_export"
    / "benny_joints_export.usda"
)

# static / dynamic restitution=0 for all presets
GROUND_FRICTION_PRESETS: dict[str, tuple[float, float]] = {
    "indoor": (0.8, 0.6),
    "rubber": (1.0, 0.9),
    "concrete": (0.9, 0.7),
    "ice": (0.15, 0.08),
}

parser = argparse.ArgumentParser(
    description="Benny smoke test: package USD + drive + onboard LiDAR + ground friction."
)
parser.add_argument(
    "--usd",
    type=str,
    default=str(DEFAULT_USD),
    help="Path to Benny package USD (default: benny_joints_export.usda).",
)
parser.add_argument("--duration", type=float, default=0.0, help="Quit after N seconds (0 = manual).")
parser.add_argument("--wheel-omega", type=float, default=5.0, help="Drive wheel angular speed [rad/s].")
parser.add_argument(
    "--flip-right-wheel",
    action="store_true",
    help="Negate right-wheel command (use if robot spins instead of going forward).",
)
parser.add_argument(
    "--ground-friction-preset",
    type=str,
    default="indoor",
    choices=sorted(GROUND_FRICTION_PRESETS.keys()),
    help="Ground PhysX friction preset (overridden by --ground-mu-* if set).",
)
parser.add_argument(
    "--ground-mu-static",
    type=float,
    default=None,
    help="Ground static friction μs (default: from preset).",
)
parser.add_argument(
    "--ground-mu-dynamic",
    type=float,
    default=None,
    help="Ground dynamic friction μd (default: from preset).",
)
parser.add_argument(
    "--ground-restitution",
    type=float,
    default=0.0,
    help="Ground restitution (default 0).",
)
parser.add_argument("--spawn-z", type=float, default=0.05, help="Initial robot Z above ground [m].")
parser.add_argument("--headless", action="store_true", help="No GUI (lidar draw disabled).")
parser.add_argument("--test", action="store_true", help="Short automated run then exit.")
args, _ = parser.parse_known_args()

if args.test:
    args.duration = args.duration or 3.0

mu_s_preset, mu_d_preset = GROUND_FRICTION_PRESETS[args.ground_friction_preset]
ground_mu_static = float(args.ground_mu_static if args.ground_mu_static is not None else mu_s_preset)
ground_mu_dynamic = float(args.ground_mu_dynamic if args.ground_mu_dynamic is not None else mu_d_preset)
ground_restitution = float(args.ground_restitution)

from isaacsim import SimulationApp

if SimulationApp is None:
    conda_env = os.environ.get("CONDA_DEFAULT_ENV") or os.environ.get("CONDA_PREFIX")
    raise SystemExit(
        "Isaac Sim failed to load SimulationApp (carb missing).\n"
        "Use a clean shell — deactivate conda and clear old Isaac PYTHONPATH:\n"
        "  conda deactivate\n"
        "  set -e PYTHONPATH\n"
        f"  # CONDA={conda_env!r}  PYTHONPATH={os.environ.get('PYTHONPATH')!r}\n"
        "  $ISAACSIM_PATH/python.sh scripts/smoke_benny_drive_lidar.py"
    )

simulation_app = SimulationApp(
    {
        "headless": bool(args.headless),
        "extra_args": ["--/rtx-transient/stableIds/enabled=true"],
    }
)

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni
from isaacsim.core.experimental.objects import Cube, DomeLight, GroundPlane
from isaacsim.core.experimental.prims import Articulation, GeomPrim, RigidPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

if not args.headless:
    app_utils.enable_extension("isaacsim.sensors.rtx.nodes")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def find_child_prim(root_path: str, name: str) -> str | None:
    """DFS for a prim whose name matches ``name`` under ``root_path``."""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return prim.GetPath().pathString
    return None


def find_omni_lidar(root_path: str) -> str | None:
    """Return the first OmniLidar prim path under ``root_path``, if any."""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() == "OmniLidar":
            return prim.GetPath().pathString
    return None


def apply_ground_friction(
    ground_path: str,
    *,
    mu_static: float,
    mu_dynamic: float,
    restitution: float,
) -> str:
    """Author a PhysicsMaterial on the ground plane collider and bind it.

    Returns:
        Path of the created material prim.
    """
    stage = omni.usd.get_context().get_stage()
    ground = stage.GetPrimAtPath(ground_path)
    if not ground or not ground.IsValid():
        raise RuntimeError(f"Ground prim missing: {ground_path}")

    # Collision lives on the Plane child of GroundPlane.
    plane_path = None
    for child in ground.GetChildren():
        if child.IsA(UsdGeom.Plane):
            plane_path = child.GetPath().pathString
            break
    if plane_path is None:
        # Fallback: bind on the ground root if structure differs.
        plane_path = ground_path
        carb.log_warn(f"[smoke] No Plane child under {ground_path}; binding friction on root.")

    mat_path = "/World/PhysicsMaterials/ground_friction"
    stage.DefinePrim("/World/PhysicsMaterials", "Scope")
    material = UsdShade.Material.Define(stage, mat_path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr().Set(float(mu_static))
    api.CreateDynamicFrictionAttr().Set(float(mu_dynamic))
    api.CreateRestitutionAttr().Set(float(restitution))

    collider = stage.GetPrimAtPath(plane_path)
    binding = UsdShade.MaterialBindingAPI.Apply(collider)
    binding.Bind(material, UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics")
    print(
        f"[smoke] Ground friction on {plane_path}: "
        f"μs={mu_static:.2f} μd={mu_dynamic:.2f} e={restitution:.2f}"
    )
    return mat_path


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------

usd_path = Path(args.usd).expanduser().resolve()
if not usd_path.is_file():
    raise FileNotFoundError(f"Benny USD not found: {usd_path}")

stage_utils.set_stage_up_axis("Z")
stage_utils.set_stage_units(meters_per_unit=1.0)
GroundPlane("/World/GroundPlane")
DomeLight("/World/DomeLight").set_intensities(800.0)
apply_ground_friction(
    "/World/GroundPlane",
    mu_static=ground_mu_static,
    mu_dynamic=ground_mu_dynamic,
    restitution=ground_restitution,
)

# Static boxes for the lidar to hit (ahead / left / right).
for name, pos, scale in (
    ("box_front", [1.5, 0.0, 0.25], [0.3, 0.8, 0.5]),
    ("box_left", [0.8, 1.0, 0.25], [0.4, 0.3, 0.5]),
    ("box_right", [0.8, -1.0, 0.25], [0.4, 0.3, 0.5]),
):
    path = f"/World/{name}"
    Cube(paths=path, positions=np.array([pos]), scales=np.array([scale]))
    RigidPrim(paths=path)
    GeomPrim(paths=path, apply_collision_apis=True)

robot_path = "/World/Benny"
# Package USD uses Physics variant set; ArticulationRootAPI is on the physx payload.
stage_utils.add_reference_to_stage(
    usd_path=str(usd_path),
    path=robot_path,
    variants=[("Physics", "physx")],
)
for _ in range(5):
    simulation_app.update()

art_roots = Articulation.fetch_articulation_root_api_prim_paths(robot_path)
art_path = next((p for p in art_roots if p), None)
if art_path is None:
    raise RuntimeError(
        f"No ArticulationRootAPI under {robot_path} after Physics=physx. "
        "Open the USD in Isaac and select Physics/physx, then save."
    )
print(f"[smoke] Robot USD: {usd_path}")
print(f"[smoke] Articulation root: {art_path}")
print(
    f"[smoke] Ground preset={args.ground_friction_preset} "
    f"(μs={ground_mu_static:.2f}, μd={ground_mu_dynamic:.2f})"
)

robot = WheeledRobot(
    art_path,
    wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
    positions=np.array([[0.0, 0.0, args.spawn_z]]),
)

# Prefer LiDAR already authored in the package USD.
lidar_path = find_omni_lidar(robot_path)
if lidar_path is None:
    lidar_parent = find_child_prim(robot_path, "base_link") or art_path
    lidar_path = f"{lidar_parent}/lidar_2d"
    print(f"[smoke] No OmniLidar in USD — creating Example_Rotary_2D at {lidar_path}")
    lidar = Lidar.create(
        lidar_path,
        config="Example_Rotary_2D",
        tick_rate=10.0,
        translations=np.array([[0.0, 0.08, 0.0]]),
    )
else:
    print(f"[smoke] Using package LiDAR: {lidar_path}")
    # Wrap without rewriting the asset transform.
    lidar = Lidar(lidar_path, accumulate_outputs=None, reset_xform_op_properties=False)

sensor = LidarSensor(lidar, annotators=[])
if not args.headless:
    sensor.attach_writer(
        "draw-point-cloud",
        size=0.03,
        color=[0.0, 1.0, 0.4, 1.0],
    )
    print("[smoke] LiDAR debug draw enabled (green points)")

try:
    import omni.replicator.core as rep
    from omni.replicator.core import Writer

    class HitCountWriter(Writer):
        def __init__(self) -> None:
            self.data_structure = "renderProduct"
            self.annotators = [rep.annotators.get("GenericModelOutput")]
            self.frame = 0
            self.last_count = 0

        def write(self, data: dict) -> None:
            if "renderProducts" not in data:
                return
            for _name, rp in data["renderProducts"].items():
                raw = rp.get("GenericModelOutput")
                if isinstance(raw, dict):
                    raw = raw.get("data")
                gmo = parse_generic_model_output_data(raw)
                self.last_count = int(gmo.numElements)
                if self.frame % 60 == 0:
                    print(f"[smoke] frame={self.frame} lidar_hits={self.last_count}")
            self.frame += 1

    rep.WriterRegistry.register(HitCountWriter)
    sensor.attach_writer("HitCountWriter")
except Exception as exc:  # noqa: BLE001 — smoke test should keep running
    carb.log_warn(f"[smoke] LiDAR hit-count writer skipped: {exc}")

SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
physics_scene = SimulationManager.get_physics_scenes()[0]
physics_scene.set_enabled_gpu_dynamics(False)

omega_l = float(args.wheel_omega)
omega_r = -float(args.wheel_omega) if args.flip_right_wheel else float(args.wheel_omega)
wheel_cmd = np.array([omega_l, omega_r], dtype=np.float32)
print(f"[smoke] Wheel command ω_L={omega_l:.2f} ω_R={omega_r:.2f} rad/s")
print("[smoke] Play: robot should drive forward; green LiDAR ring should hit the boxes.")
print("[smoke] If it spins in place, re-run with --flip-right-wheel")

app_utils.play()
app_utils.update_app(steps=10)

frame = 0
dt = 1.0 / 60.0
max_frames = int(args.duration / dt) if args.duration > 0 else None

while simulation_app.is_running():
    simulation_app.update()
    if not app_utils.is_playing():
        continue

    robot.apply_wheel_actions(wheel_cmd)
    frame += 1

    if frame % 120 == 0:
        try:
            pos = robot.get_world_poses()[0].numpy().reshape(-1)[:3]
            print(f"[smoke] t≈{frame * dt:.1f}s  pose_xy=({pos[0]:.2f}, {pos[1]:.2f}) z={pos[2]:.3f}")
        except Exception:
            pass

    if max_frames is not None and frame >= max_frames:
        print(f"[smoke] Duration {args.duration:.1f}s reached — exiting.")
        break
    if args.test and frame >= 90:
        break

app_utils.stop()
simulation_app.close()
print("[smoke] Done.")
