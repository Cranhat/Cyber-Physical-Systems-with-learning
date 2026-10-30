#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Train Benny A->B navigation (rsl_rl PPO, categorical policy) in Isaac Lab 3.0.

Run from the repo root in fish (no conda, clean env), e.g.::

    conda deactivate
    set -e PYTHONPATH
    set -e LD_LIBRARY_PATH
    ~/Omniverse/isaacsim-6.2/python.sh scripts/benny_train.py --num_envs 512 --headless

Extra arguments are forwarded to the Isaac Lab ``rsl_rl`` training backend (``--max_iterations``, ``--seed``,
``--checkpoint``, ``--run_name`` ...).
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

import warp as wp  # noqa: E402

wp.config.enable_backward = False  # must be set before importing modules that define Warp kernels

import benny_nav  # noqa: E402,F401  (registers Isaac-Benny-Nav-v0)
from isaaclab_rl.entrypoints import run_train_cli  # noqa: E402


def main() -> int:
    # Isaac Lab 3.0 is headless by default (GUI only with ``--viz kit``); tolerate the old ``--headless`` flag.
    argv = [a for a in sys.argv[1:] if a != "--headless"]
    if "--task" not in argv:
        argv = ["--task", benny_nav.TASK_ID, *argv]
    if "--rl_library" not in argv:
        argv = ["--rl_library", "rsl_rl", *argv]
    return run_train_cli(argv)


if __name__ == "__main__":
    raise SystemExit(main())
