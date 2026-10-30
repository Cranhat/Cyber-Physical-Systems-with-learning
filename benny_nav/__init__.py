# SPDX-License-Identifier: MIT
"""Benny differential-drive robot: A->B navigation with obstacle avoidance in Isaac Lab 3.0 (simulation only)."""

import gymnasium as gym

TASK_ID = "Isaac-Benny-Nav-v0"


def register() -> None:
    """Register the Gym task. Safe to call more than once."""
    if TASK_ID in gym.registry:
        return
    gym.register(
        id=TASK_ID,
        entry_point=f"{__name__}.benny_nav_env:BennyNavEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.benny_nav_env_cfg:BennyNavEnvCfg",
            "rsl_rl_cfg_entry_point": f"{__name__}.agents.rsl_rl_ppo_cfg:BennyNavPPORunnerCfg",
            "default_agent": "rsl_rl",
        },
    )


register()
