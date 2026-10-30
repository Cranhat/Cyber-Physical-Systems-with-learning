# SPDX-License-Identifier: MIT
"""rsl_rl PPO runner configuration for the Benny navigation task (discrete / categorical policy)."""

from __future__ import annotations

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg


@configclass
class CategoricalDistributionCfg(RslRlMLPModelCfg.DistributionCfg):
    """Categorical output distribution (see ``benny_nav/categorical.py``)."""

    class_name: str = "benny_nav.categorical:CategoricalDistribution"
    num_classes: int = 4


@configclass
class BennyNavPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 64  # 64 * 0.1 s = 6.4 s of experience per env per update
    max_iterations = 1000
    save_interval = 50
    experiment_name = "benny_nav"
    actor = RslRlMLPModelCfg(
        hidden_dims=[256, 128, 64],
        activation="elu",
        obs_normalization=False,
        distribution_cfg=CategoricalDistributionCfg(num_classes=4),
    )
    critic = RslRlMLPModelCfg(
        hidden_dims=[256, 128, 64],
        activation="elu",
        obs_normalization=False,
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.01,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=3.0e-4,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )
