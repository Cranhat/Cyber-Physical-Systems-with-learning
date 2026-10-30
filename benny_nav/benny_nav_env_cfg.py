# SPDX-License-Identifier: MIT
"""Configuration of the Benny A->B navigation task (Isaac Lab 3.0, direct workflow).

Sensing: 360 deg 2D LiDAR only (Isaac Lab ``MultiMeshRayCaster`` during training).
Actions: discrete primitives, one at a time (DRIVE / TURN_L / TURN_R / STOP).
"""

from __future__ import annotations

from isaaclab_physx.physics import PhysxCfg

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import MultiMeshRayCasterCfg, patterns
from isaaclab.sim import SimulationCfg
from isaaclab.sim.schemas import UsdPhysicsRigidBodyCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass, replace

from .assets import BENNY_CFG

##
# Geometry of the arena (in the env frame, centre of the arena = env origin)
##

ARENA_SIZE_X = 2.0  # [m]
ARENA_SIZE_Y = 1.0  # [m]
WALL_THICKNESS = 0.02  # [m]
WALL_HEIGHT = 0.2  # [m]  (LiDAR beam is ~0.11 m above the ground -> walls/boxes must be taller)

# Fixed footprints (x, y) of the movable obstacle boxes. Pose (position, yaw) is randomised every episode.
OBSTACLE_SIZES: tuple[tuple[float, float], ...] = ((0.12, 0.12), (0.18, 0.10), (0.10, 0.22), (0.15, 0.15))
OBSTACLE_HEIGHT = 0.2  # [m]

NUM_BEAMS = 36  # 360 deg / 10 deg
PARK_Z = -5.0  # inactive obstacles / walls are teleported here (below the ground, invisible to the LiDAR)


def _kinematic_box(
    size: tuple[float, float, float], color: tuple[float, float, float], collision: bool = True
) -> RigidObjectCfg:
    return RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/placeholder",
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, PARK_Z)),
        spawn=sim_utils.CuboidCfg(
            size=size,
            rigid_props=UsdPhysicsRigidBodyCfg(rigid_body_enabled=True, kinematic_enabled=True),
            collision_props=sim_utils.UsdPhysicsCollisionCfg(collision_enabled=collision) if collision else None,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=0.7),
        ),
    )


def _named(cfg: RigidObjectCfg, name: str) -> RigidObjectCfg:
    cfg.prim_path = f"{{ENV_REGEX_NS}}/{name}"
    return cfg


@configclass
class BennyNavSceneCfg(InteractiveSceneCfg):
    """Ground + Benny + kinematic walls / obstacles / goal marker + LiDAR."""

    ground = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=0.8,
            dynamic_friction=0.6,
            restitution=0.0,
        ),
        debug_vis=False,
    )

    robot = replace(BENNY_CFG, prim_path="{ENV_REGEX_NS}/Robot")

    # Walls (long ones along x at +/-y, short ones along y at +/-x). Parked when the episode has no walls.
    wall_n = _named(
        _kinematic_box((ARENA_SIZE_X + 2 * WALL_THICKNESS, WALL_THICKNESS, WALL_HEIGHT), (0.55, 0.55, 0.6)), "wall_n"
    )
    wall_s = _named(
        _kinematic_box((ARENA_SIZE_X + 2 * WALL_THICKNESS, WALL_THICKNESS, WALL_HEIGHT), (0.55, 0.55, 0.6)), "wall_s"
    )
    wall_e = _named(_kinematic_box((WALL_THICKNESS, ARENA_SIZE_Y, WALL_HEIGHT), (0.55, 0.55, 0.6)), "wall_e")
    wall_w = _named(_kinematic_box((WALL_THICKNESS, ARENA_SIZE_Y, WALL_HEIGHT), (0.55, 0.55, 0.6)), "wall_w")

    # Obstacle boxes (fixed size per slot, random pose per episode).
    box_0 = _named(_kinematic_box((*OBSTACLE_SIZES[0], OBSTACLE_HEIGHT), (0.8, 0.3, 0.2)), "box_0")
    box_1 = _named(_kinematic_box((*OBSTACLE_SIZES[1], OBSTACLE_HEIGHT), (0.8, 0.5, 0.2)), "box_1")
    box_2 = _named(_kinematic_box((*OBSTACLE_SIZES[2], OBSTACLE_HEIGHT), (0.7, 0.3, 0.3)), "box_2")
    box_3 = _named(_kinematic_box((*OBSTACLE_SIZES[3], OBSTACLE_HEIGHT), (0.8, 0.4, 0.1)), "box_3")

    # Goal B marker (visual only, no collision).
    goal = _named(_kinematic_box((0.12, 0.12, 0.004), (0.1, 0.8, 0.2), collision=False), "goal")

    # 360 deg 2D LiDAR. Tracks the ``lidar_frame`` Xform authored on base_link in assets/benny_train.usda
    # (0.087 m above base_link, rotated so that the sensor x axis is the robot forward axis).
    lidar = MultiMeshRayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/Geometry/base_link/lidar_frame",
        spawn=None,
        ray_alignment="base",
        pattern_cfg=patterns.LidarPatternCfg(
            channels=1,
            vertical_fov_range=(0.0, 0.0),
            horizontal_fov_range=(-180.0, 180.0),
            horizontal_res=10.0,
        ),
        max_distance=2.0,
        update_period=0.0,
        debug_vis=False,
        mesh_prim_paths=[
            MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr=f"{{ENV_REGEX_NS}}/{name}", track_mesh_transforms=True)
            for name in (
                "wall_n",
                "wall_s",
                "wall_e",
                "wall_w",
                *(f"box_{i}" for i in range(len(OBSTACLE_SIZES))),
            )
        ],
    )

    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75)),
    )


@configclass
class BennyNavEnvCfg(DirectRLEnvCfg):
    """Benny A->B navigation with obstacle avoidance (training configuration)."""

    # --- env timing ---
    decimation = 12  # 12 * (1/120 s) = 0.1 s per RL step
    episode_length_s = 40.0
    action_space = 1  # one number = index of the discrete primitive (see NUM_ACTIONS)
    observation_space = NUM_BEAMS + 3 + 4 + 2
    state_space = 0

    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation, physics=PhysxCfg())

    scene: BennyNavSceneCfg = BennyNavSceneCfg(num_envs=512, env_spacing=4.0, replicate_physics=True)

    # --- discrete primitives (DRIVE / TURN_L / TURN_R / STOP) ---
    num_primitives: int = 4
    v_const: float = 0.20  # [m/s]
    omega_const: float = 1.5  # [rad/s]
    turn_gain: float = 1.3  # wheel-speed multiplier for TURN primitives so the realised yaw rate is ~omega_const
    wheel_accel_limit: float = 60.0  # [rad/s^2] wheel command ramp
    right_wheel_sign: float = -1.0  # right_wheel_joint axis is mirrored: positive joint velocity = backwards
    left_wheel_sign: float = 1.0

    # --- sensing ---
    lidar_d_max: float = 2.0  # [m]  (must match scene.lidar.max_distance)
    goal_dist_scale: float = 2.5  # [m]  goal distance normaliser

    # --- arena sampling ---
    spawn_margin: float = 0.15  # [m] min distance of A / B / obstacles from the arena border
    walls_probability: float = 0.7  # probability that an episode has walls (otherwise the arena is open)
    goal_min_dist: float = 0.25  # [m]
    goal_success_radius: float = 0.10  # [m]
    # [m] min LiDAR range below which we declare a hard collision. The chassis is 14x14 cm with the LiDAR at its
    # centre (edge 0.07 m, corner 0.10 m), so anything below ~0.1 m is physical contact.
    collision_dist: float = 0.11
    out_of_bounds_margin: float = 0.05  # [m] beyond the (virtual) arena border -> episode failure

    # --- reward ---
    w_progress: float = 10.0  # per metre of distance reduction
    w_reach: float = 10.0  # one-off success bonus
    w_collision: float = 5.0  # one-off collision / out-of-bounds penalty
    w_time: float = 0.1  # per second
    w_near: float = 5.0  # soft clearance penalty weight
    d_safe: float = 0.20  # [m] clearance below which w_near applies

    # --- curriculum (level -> (max obstacles, max goal distance)) ---
    curriculum_enabled: bool = True
    curriculum_start_level: int = 0
    curriculum_levels: tuple[tuple[int, float], ...] = ((0, 0.8), (2, 1.2), (3, 1.6), (4, 2.2))
    curriculum_promote_success: float = 0.8
    curriculum_window_episodes: int = 1024
    lidar_noise_levels: tuple[float, ...] = (0.0, 0.0, 0.0, 0.01)  # [m] LiDAR range noise std per level

    def play_mode(self):
        """Evaluation: hardest curriculum level, no curriculum progression, few environments."""
        super().play_mode()
        self.scene.num_envs = min(self.scene.num_envs, 16)
        self.curriculum_enabled = False
        self.curriculum_start_level = len(self.curriculum_levels) - 1


__all__ = ["BennyNavEnvCfg", "BennyNavSceneCfg", "NUM_BEAMS", "PARK_Z", "OBSTACLE_SIZES"]
