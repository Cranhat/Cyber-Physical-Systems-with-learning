# SPDX-License-Identifier: MIT
"""Benny articulation configuration for Isaac Lab."""

from __future__ import annotations

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

BENNY_USD_TRAIN = str(Path(__file__).resolve().parent / "benny_train.usda")
"""Benny USD without the RTX OmniLidar prim (used for headless training)."""

# --- Geometry measured from the meshes / USD (see benny_joints_export.usda notes) ---
WHEEL_RADIUS = 0.016  # [m]
WHEEL_TRACK = 0.1552  # [m] distance between the two wheel contact-patch centres
LIDAR_HEIGHT_BODY_Y = 0.087  # [m] lidar height above base_link origin (body +y == world up)

# Body (Fusion Y-up) -> world: Rx(+90) maps body +y to world +z. An extra Rz(+90) makes the
# front caster side (body +z) point along world +x, i.e. robot "forward" = +x at yaw 0.
# Quaternion (x, y, z, w) of Rz(90) * Rx(90).
BODY_TO_FORWARD_X_QUAT = (0.5, 0.5, 0.5, 0.5)
BASE_LINK_HEIGHT = 0.021  # [m] base_link origin height above the ground when resting on the wheels

BENNY_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=BENNY_USD_TRAIN,
        variants={"Physics": "physx"},
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, BASE_LINK_HEIGHT + 0.004),
        rot=BODY_TO_FORWARD_X_QUAT,
        joint_pos={".*": 0.0},
        joint_vel={".*": 0.0},
    ),
    actuators={
        # Velocity-controlled drive wheels: stiffness 0, damping gives the velocity loop.
        # effort limit keeps the acceleration realistic (URDF soft limit is 5 N*m, far too
        # much for a 1.9 kg robot with 3.2 cm wheels -> would pop wheelies).
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=["left_wheel_joint", "right_wheel_joint"],
            # Tuned in sim: realised ~0.18-0.19 m/s for the 0.20 m/s command and ~1.45 rad/s for 1.5 rad/s
            # (with turn_gain 1.3). The armature stabilises the very light wheels (no chatter).
            joint_effort_limit=0.3,
            armature=5e-5,
            joint_velocity_limit=20.0,
            stiffness=0.0,
            damping=0.1,
        ),
        # Passive caster joints. The small armature is essential: with zero armature the ~8 g caster rollers
        # spin up to >100 rad/s through contact jitter and pitch the robot over (~10% of random episodes).
        "casters": ImplicitActuatorCfg(
            joint_names_expr=[".*caster.*"],
            stiffness=0.0,
            damping=0.01,
            armature=1e-5,
            joint_velocity_limit=100.0,
        ),
    },
)
