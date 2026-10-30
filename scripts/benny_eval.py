#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Evaluate a trained Benny policy: success / collision / out-of-bounds / timeout statistics (headless by default).

Run from the repo root in a clean shell (no conda), e.g.::

    ~/Omniverse/isaacsim-6.2/python.sh scripts/benny_eval.py --checkpoint latest --episodes 500 --level 3

Add ``--viz kit`` to watch it in the Isaac Sim GUI (use fewer envs: ``--num_envs 4``).
The policy acts deterministically (arg-max over the 4 primitive logits).
"""

import argparse
import contextlib
import os
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

import warp as wp  # noqa: E402

wp.config.enable_backward = False

import torch  # noqa: E402

import benny_nav  # noqa: E402,F401  (registers Isaac-Benny-Nav-v0)
from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402
from isaaclab_rl.entrypoints.backends import cli_args_rsl_rl as cli_args  # noqa: E402
from isaaclab_rl.entrypoints.backends.play_rsl_rl import _resolve_checkpoint  # noqa: E402
from isaaclab_rl.entrypoints.common import (  # noqa: E402
    apply_env_overrides,
    close_env,
    create_isaaclab_env,
    set_hydra_args,
)
from isaaclab_rl.rsl_rl import (  # noqa: E402
    RslRlVecEnvWrapper,
    check_rsl_rl_version,
    create_rsl_rl_runner,
    handle_deprecated_rsl_rl_cfg,
)
from isaaclab_tasks.utils import resolve_task_config, setup_preset_cli  # noqa: E402


def _attach_rtx_lidar(u) -> None:
    """Attach the RTX LiDAR (Example_Rotary_2D, as in the package USD) to env_0's robot and draw its point cloud.

    Visualisation only: the policy observes the PhysX RayCaster scan (same 36-beam geometry). Needs ``--viz kit``.
    """
    import numpy as np  # noqa: F401
    import isaacsim.core.experimental.utils.app as app_utils
    import omni.usd
    from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

    app_utils.enable_extension("isaacsim.sensors.rtx.nodes")
    frame = "/World/envs/env_0/Robot/Geometry/base_link/lidar_frame"
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(frame).IsValid():
        print(f"[eval] WARNING: {frame} not found, RTX LiDAR not attached")
        return
    lidar = Lidar.create(f"{frame}/rtx_lidar", config="Example_Rotary_2D", tick_rate=10.0)
    sensor = LidarSensor(lidar, annotators=[])
    sensor.attach_writer("draw-point-cloud", size=0.02, color=[0.0, 1.0, 0.4, 1.0])
    u._rtx_lidar_sensor = sensor  # keep a reference alive
    print("[eval] RTX LiDAR attached (green points) at", frame)


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate a Benny navigation policy.")
    parser.add_argument("--task", default=benny_nav.TASK_ID)
    parser.add_argument("--agent", default="rsl_rl_cfg_entry_point")
    parser.add_argument("--num_envs", type=int, default=64)
    parser.add_argument("--episodes", type=int, default=500, help="Number of finished episodes to collect.")
    parser.add_argument("--level", type=int, default=None, help="Curriculum level to evaluate (default: hardest).")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--real-time", action="store_true", help="Pace the loop to real time (for the GUI).")
    parser.add_argument("--rtx_lidar", action="store_true", help="Draw an RTX LiDAR point cloud (GUI, --viz kit).")
    parser.add_argument("--disable_fabric", action="store_true", default=False)
    cli_args.add_rsl_rl_args(parser)
    add_launcher_args(parser)
    argv = [a for a in sys.argv[1:] if a != "--headless"]
    args_cli, remaining = setup_preset_cli(parser, argv)
    set_hydra_args(remaining)
    args_cli.checkpoint = args_cli.checkpoint or "latest"
    args_cli.frontend = "torch"

    installed_version = check_rsl_rl_version()
    env_cfg, agent_cfg = resolve_task_config(args_cli.task, args_cli.agent, play_mode=True)
    with launch_simulation(env_cfg, args_cli), contextlib.ExitStack() as cleanup:
        agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
        agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, installed_version)
        apply_env_overrides(args_cli, env_cfg)
        if args_cli.level is not None:
            env_cfg.curriculum_start_level = args_cli.level
        env_cfg.seed = args_cli.seed if args_cli.seed is not None else agent_cfg.seed

        log_root_path = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
        resume_path = _resolve_checkpoint(args_cli, agent_cfg, env_cfg, log_root_path)
        if resume_path is None:
            return 1
        print(f"[eval] checkpoint: {resume_path}")

        env = create_isaaclab_env(args_cli.task, env_cfg, args_cli, convert_marl_to_single_agent=False)
        cleanup.callback(lambda: close_env(env))
        u = env.unwrapped
        env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
        if args_cli.rtx_lidar:
            _attach_rtx_lidar(u)
        runner = create_rsl_rl_runner(env, agent_cfg)
        runner.load(resume_path)
        policy = runner.get_inference_policy(device=u.device)

        # Count outcomes right before the env resets the finished episodes.
        stats = {"success": 0, "collision": 0, "out_of_bounds": 0, "fell": 0, "time_out": 0, "episodes": 0}
        ep_steps_success: list[float] = []
        orig_reset = u._reset_idx

        def spy_reset(env_ids):
            ids = wp.to_torch(env_ids) if isinstance(env_ids, wp.array) else torch.as_tensor(env_ids, device=u.device)
            ids = ids.long()
            valid = u._episode_valid[ids]
            if bool(valid.any()):
                ended = ids[valid]
                stats["episodes"] += int(ended.numel())
                stats["success"] += int(u._success[ended].sum())
                stats["collision"] += int(u._collision[ended].sum())
                stats["out_of_bounds"] += int(u._out_of_bounds[ended].sum())
                stats["fell"] += int(u._fell[ended].sum())
                stats["time_out"] += int((u.reset_time_outs[ended] & ~u._success[ended]).sum())
                ok = ended[u._success[ended]]
                ep_steps_success.extend(u.episode_length_buf[ok].float().mul(u.step_dt).tolist())
            return orig_reset(env_ids)

        u._reset_idx = spy_reset

        print(f"[eval] curriculum level {u._level}, {u.num_envs} envs, collecting {args_cli.episodes} episodes ...")
        obs = env.get_observations()
        with torch.inference_mode():
            while stats["episodes"] < args_cli.episodes:
                t0 = time.time()
                obs, _, dones, _ = env.step(policy(obs))
                policy.reset(dones)
                if args_cli.real_time and u.step_dt - (time.time() - t0) > 0:
                    time.sleep(u.step_dt - (time.time() - t0))

        n = max(1, stats["episodes"])
        print("[eval] ---------------- results ----------------")
        print(f"[eval] level {u._level}, episodes {stats['episodes']}")
        for key in ("success", "collision", "out_of_bounds", "fell", "time_out"):
            print(f"[eval] {key:14s}: {stats[key]:5d}  ({100.0 * stats[key] / n:5.1f} %)")
        if ep_steps_success:
            print(f"[eval] mean time to goal (successful episodes): {sum(ep_steps_success) / len(ep_steps_success):.1f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
