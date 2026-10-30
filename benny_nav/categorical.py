# SPDX-License-Identifier: MIT
"""Categorical output distribution for ``rsl_rl`` (discrete actions with the stock PPO runner).

rsl_rl 5.x only ships Gaussian / Heteroscedastic / Beta distributions, but the actor's output distribution is
pluggable via ``distribution_cfg["class_name"]``. This class makes the actor output ``num_classes`` logits and
samples ONE integer action per environment, returned as a ``(N, 1)`` float tensor so that the unmodified
``RslRlVecEnvWrapper`` / rollout storage (which expect a ``(N, num_actions)`` tensor with ``num_actions = 1``) work.
"""

from __future__ import annotations

import torch
import torch.nn as nn
from torch.distributions import Categorical

from rsl_rl.modules.distribution import Distribution


class _ArgmaxOutput(nn.Module):
    """Export friendly deterministic output: ``argmax`` of the logits as a float column."""

    def forward(self, mlp_output: torch.Tensor) -> torch.Tensor:
        return torch.argmax(mlp_output, dim=-1, keepdim=True).to(mlp_output.dtype)


class CategoricalDistribution(Distribution):
    """Categorical policy head. ``output_dim`` is the env action dim (1); ``num_classes`` the number of primitives."""

    def __init__(self, output_dim: int = 1, num_classes: int = 4) -> None:
        super().__init__(output_dim)
        if output_dim != 1:
            raise ValueError(f"CategoricalDistribution supports a single discrete action per env, got {output_dim}.")
        self.num_classes = num_classes
        self._distribution: Categorical | None = None
        self._logits: torch.Tensor | None = None
        Categorical.set_default_validate_args(False)

    # ---- sampling -------------------------------------------------------------------------------------------

    def update(self, mlp_output: torch.Tensor) -> None:
        self._logits = torch.log_softmax(mlp_output, dim=-1)
        self._distribution = Categorical(logits=self._logits)

    def sample(self) -> torch.Tensor:
        return self._distribution.sample().unsqueeze(-1).to(self._logits.dtype)  # type: ignore[union-attr]

    def deterministic_output(self, mlp_output: torch.Tensor) -> torch.Tensor:
        return torch.argmax(mlp_output, dim=-1, keepdim=True).to(mlp_output.dtype)

    def as_deterministic_output_module(self) -> nn.Module:
        return _ArgmaxOutput()

    # ---- statistics -----------------------------------------------------------------------------------------

    @property
    def input_dim(self) -> int:
        return self.num_classes

    @property
    def mean(self) -> torch.Tensor:
        probs = self._distribution.probs  # type: ignore[union-attr]
        idx = torch.arange(self.num_classes, device=probs.device, dtype=probs.dtype)
        return (probs * idx).sum(dim=-1, keepdim=True)

    @property
    def std(self) -> torch.Tensor:
        probs = self._distribution.probs  # type: ignore[union-attr]
        idx = torch.arange(self.num_classes, device=probs.device, dtype=probs.dtype)
        mean = (probs * idx).sum(dim=-1, keepdim=True)
        var = (probs * (idx - mean) ** 2).sum(dim=-1, keepdim=True)
        return torch.sqrt(var)

    @property
    def entropy(self) -> torch.Tensor:
        return self._distribution.entropy()  # type: ignore[union-attr]

    @property
    def params(self) -> tuple[torch.Tensor, ...]:
        return (self._logits,)  # type: ignore[return-value]

    def log_prob(self, outputs: torch.Tensor) -> torch.Tensor:
        actions = outputs.reshape(outputs.shape[0], -1)[:, 0].round().long()
        return self._distribution.log_prob(actions)  # type: ignore[union-attr]

    def kl_divergence(self, old_params: tuple[torch.Tensor, ...], new_params: tuple[torch.Tensor, ...]) -> torch.Tensor:
        old_logp = torch.log_softmax(old_params[0], dim=-1)
        new_logp = torch.log_softmax(new_params[0], dim=-1)
        return (old_logp.exp() * (old_logp - new_logp)).sum(dim=-1)

    def init_mlp_weights(self, mlp: nn.Module) -> None:
        """Near-uniform initial policy: small gain on the logits layer."""
        last = mlp[-1]
        nn.init.orthogonal_(last.weight, gain=0.01)  # type: ignore[arg-type]
        nn.init.zeros_(last.bias)  # type: ignore[arg-type]
