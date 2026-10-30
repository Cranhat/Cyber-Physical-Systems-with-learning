#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Play / evaluate a trained Benny navigation policy (GUI, few envs) in Isaac Lab 3.0.

Run from the repo root in fish (no conda, clean env), e.g.::

    ~/Omniverse/isaacsim-6.2/python.sh scripts/benny_play.py --num_envs 16 --checkpoint latest

``play_mode()`` of the task config selects the hardest curriculum level and at most 16 envs.
Extra arguments are forwarded to the Isaac Lab ``rsl_rl`` play backend.
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

import warp as wp  # noqa: E402

wp.config.enable_backward = False

import benny_nav  # noqa: E402,F401  (registers Isaac-Benny-Nav-v0)
from isaaclab_rl.entrypoints import run_play_cli  # noqa: E402


def main() -> int:
    argv = sys.argv[1:]
    if "--task" not in argv:
        argv = ["--task", benny_nav.TASK_ID, *argv]
    if "--rl_library" not in argv:
        argv = ["--rl_library", "rsl_rl", *argv]
    return run_play_cli(argv)


if __name__ == "__main__":
    raise SystemExit(main())
