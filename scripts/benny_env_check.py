#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Scripted checks of the Benny Isaac Lab environment (kinematics, LiDAR geometry, resets).

    ~/Omniverse/isaacsim-6.2/python.sh scripts/benny_env_check.py --headless --num_envs 4 --check kinematics
    ~/Omniverse/isaacsim-6.2/python.sh scripts/benny_env_check.py --headless --num_envs 8 --check lidar
    ~/Omniverse/isaacsim-6.2/python.sh scripts/benny_env_check.py --headless --num_envs 64 --check random
"""

import argparse
import contextlib
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

import warp as wp  # noqa: E402

wp.config.enable_backward = False

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import benny_nav  # noqa: E402
from benny_nav.benny_nav_env_cfg import ARENA_SIZE_X, ARENA_SIZE_Y  # noqa: E402

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

from isaaclab_tasks.utils import resolve_task_config, setup_preset_cli  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--check", choices=["kinematics", "lidar", "random", "tip"], default="kinematics")
parser.add_argument("--level", type=int, default=0, help="curriculum level (obstacles / goal distance)")
parser.add_argument("--steps", type=int, default=200)
add_launcher_args(parser)
args_cli, hydra_args = setup_preset_cli(parser, sys.argv[1:])
sys.argv = [sys.argv[0]] + hydra_args

env_cfg, _ = resolve_task_config(benny_nav.TASK_ID, "")
env_cfg.scene.num_envs = args_cli.num_envs
env_cfg.curriculum_enabled = False
env_cfg.curriculum_start_level = args_cli.level
args_cli.device = env_cfg.sim.device


def act(env, value: int) -> torch.Tensor:
    return torch.full((env.unwrapped.num_envs, 1), float(value), device=env.unwrapped.device)


def pose(env):
    robot = env.unwrapped.scene["robot"]
    s = env.unwrapped._robot_state()
    pos = s["pos"] - env.unwrapped.scene.env_origins
    return pos[:, :2].cpu(), s["heading"].cpu(), s["up_z"].cpu()


with launch_simulation(env_cfg, args_cli), contextlib.ExitStack() as cleanup:
    env = gym.make(benny_nav.TASK_ID, cfg=env_cfg)
    cleanup.callback(env.close)
    obs, _ = env.reset()
    print("[check] obs shape", obs["policy"].shape, "action space", env.action_space)
    u = env.unwrapped

    if args_cli.check == "kinematics":
        # Place all robots at the origin heading +x, free of obstacles and goals (override via direct writes).
        ids = torch.arange(u.num_envs, device=u.device)
        q0 = torch.tensor([[0.5, 0.5, 0.5, 0.5]], device=u.device).expand(u.num_envs, 4)
        pose0 = torch.cat([u.scene.env_origins[:, :3] + torch.tensor([0, 0, 0.025], device=u.device), q0], dim=-1)
        u._robot.write_root_pose_to_sim_index(root_pose=pose0, env_ids=ids)
        u._goal_xy_w[:] = u.scene.env_origins[:, :2] + 100.0  # far away, never reached
        for name, action, seconds in (("STOP", 3, 0.5), ("DRIVE", 0, 3.0), ("TURN_L", 1, 2.0), ("TURN_R", 2, 2.0)):
            xy0, yaw0, _ = pose(env)
            min_up = 1.0
            for _ in range(int(seconds / u.step_dt)):
                env.step(act(env, action))
                min_up = min(min_up, pose(env)[2].min().item())
            xy1, yaw1, up = pose(env)
            jv = u._robot.data.joint_vel.torch[:, u._wheel_ids].tolist()
            tq = u._robot.data.applied_torque.torch[:, u._wheel_ids].tolist()
            print(f"[check]   wheel joint vel {jv}\n[check]   torque {tq}\n[check]   target {u._wheel_targets[0].tolist()}")
            dxy = (xy1 - xy0).norm(dim=-1)
            dyaw = torch.remainder(yaw1 - yaw0 + math.pi, 2 * math.pi) - math.pi
            print(
                f"[check] {name:7s} {seconds:.1f}s: dpos={dxy.mean():.3f} m (v={dxy.mean() / seconds:.3f} m/s) "
                f"dyaw={dyaw.mean():+.3f} rad (w={dyaw.mean() / seconds:+.3f} rad/s) up_z={up.mean():.3f} (min {min_up:.3f}) "
                f"dx={(xy1 - xy0)[:, 0].mean():+.3f} dy={(xy1 - xy0)[:, 1].mean():+.3f}"
            )

    elif args_cli.check == "lidar":
        obs_reset = obs["policy"][:, :36].clone()
        obs, *_ = env.step(act(env, 3))
        stale_err = (obs["policy"][:, :36] - obs_reset).abs().max().item()
        print(f"[check] max |scan after reset - scan after 1 STOP step| = {stale_err:.4f} (normalised; ~0 if fresh)")
        for step in range(4):
            obs, *_ = env.step(act(env, 3))
        ranges = obs["policy"][:, :36] * env_cfg.lidar_d_max
        torch.set_printoptions(precision=2, linewidth=200)
        for i in range(min(4, u.num_envs)):
            xy, yaw, _ = pose(env)
            rel_goal = u._goal_xy_w[i].cpu() - (u.scene.env_origins[i, :2].cpu() + xy[i])
            print(f"[check] env {i}: pos={xy[i].tolist()} yaw={yaw[i]:.2f} goal_rel={rel_goal.tolist()}")
            print("[check]   ranges (beam 0 = forward, ccw, 10 deg):")
            print(ranges[i].cpu())
        print("[check] min/mean range over envs:", ranges.min().item(), ranges.mean().item())
        # Analytic comparison for envs with walls and no obstacles: find the beam convention that fits.
        xy, yaw, _ = pose(env)
        hx, hy = ARENA_SIZE_X / 2, ARENA_SIZE_Y / 2
        k = torch.arange(36, device=ranges.device).float() * math.radians(10.0)
        for off_deg in range(-180, 180, 10):
            for sgn in (1.0, -1.0):
                errs = []
                for i in range(u.num_envs):
                    if (ranges[i] > 1.99).all():
                        continue  # no walls in this env
                    ang = yaw[i] + math.radians(off_deg) + sgn * k
                    dx, dy = torch.cos(ang), torch.sin(ang)
                    tx = torch.where(dx > 0, (hx - xy[i, 0]) / dx, (-hx - xy[i, 0]) / dx)
                    ty = torch.where(dy > 0, (hy - xy[i, 1]) / dy, (-hy - xy[i, 1]) / dy)
                    exp = torch.minimum(tx.abs().clamp(max=2.0), ty.abs().clamp(max=2.0)).clamp(max=2.0)
                    errs.append((exp - ranges[i]).abs().mean().item())
                if errs:
                    mean_err = sum(errs) / len(errs)
                    if mean_err < 0.1:
                        print(f"[check] beam model fit: offset {off_deg} deg, sign {sgn:+.0f}: mean abs err {mean_err:.3f} m")

    elif args_cli.check == "tip":
        # Random actions in open space (goal far away, robots re-centred): which action transitions tip the robot?
        ids = torch.arange(u.num_envs, device=u.device)
        q0 = torch.tensor([[0.5, 0.5, 0.5, 0.5]], device=u.device).expand(u.num_envs, 4)
        pose0 = torch.cat([u.scene.env_origins[:, :3] + torch.tensor([0, 0, 0.025], device=u.device), q0], dim=-1)
        u._robot.write_root_pose_to_sim_index(root_pose=pose0, env_ids=ids)
        u._goal_xy_w[:] = u.scene.env_origins[:, :2] + 100.0
        prev = torch.full((u.num_envs,), 3, device=u.device)
        counts: dict = {}
        worst = 1.0
        for _ in range(args_cli.steps):
            a = torch.randint(0, 4, (u.num_envs,), device=u.device)
            env.step(a.float().unsqueeze(-1))
            up = pose(env)[2]
            worst = min(worst, up.min().item())
            for i in torch.nonzero(up < 0.9).flatten().tolist():
                key = f"{int(prev[i])}->{int(a[i])}"
                if len(counts) < 6:
                    from isaaclab.utils.math import quat_apply_inverse

                    q = u._robot.data.root_quat_w.torch[i : i + 1]
                    zb = quat_apply_inverse(q, torch.tensor([[0.0, 0.0, 1.0]], device=u.device))[0].tolist()
                    wv = u._robot.data.joint_vel.torch[i].tolist()
                    print(f"[check] tip detail {key}: world-up in body frame (x=side,y=up,z=front)={[round(x,2) for x in zb]} "
                          f"joint_vel={[round(x,1) for x in wv]} pos_z={u._robot.data.root_pos_w.torch[i,2].item():.3f}")
                counts[key] = counts.get(key, 0) + 1
            if (up < 0.9).any():
                bad = torch.nonzero(up < 0.9).flatten()
                bad_w = wp.from_torch(bad)
                u._robot.write_root_pose_to_sim_index(root_pose=pose0[bad], env_ids=bad_w)
                u._robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(len(bad), 6, device=u.device), env_ids=bad_w)
            prev = a
        print(f"[check] tip events (prev->cur action): {counts}; worst up_z {worst:.3f}")

    else:
        total_reward = 0.0
        term_counts: dict = {}
        fell_log: list = []
        orig_reset = u._reset_idx

        def spy_reset(env_ids):
            ids_t = wp.to_torch(env_ids) if isinstance(env_ids, wp.array) else torch.as_tensor(env_ids, device=u.device)
            for i in ids_t.tolist():
                if bool(u._fell[i]) and len(fell_log) < 12:
                    rel = (u._robot.data.root_pos_w.torch[i, :2] - u.scene.env_origins[i, :2]).tolist()
                    fell_log.append(
                        f"len={int(u.episode_length_buf[i])} pos=({rel[0]:.2f},{rel[1]:.2f}) "
                        f"min_range={u._min_range[i]:.3f} act={int(u._action[i])} prev={int(u._prev_action[i])}"
                    )
            return orig_reset(env_ids)

        u._reset_idx = spy_reset
        for step in range(args_cli.steps):
            actions = torch.randint(0, 4, (u.num_envs, 1), device=u.device).float()
            obs, rew, term, trunc, extras = env.step(actions)
            total_reward += rew.mean().item()
            for key, val in (extras.get("log") or {}).items():
                if key.startswith("Episode_Termination/"):
                    term_counts[key] = term_counts.get(key, 0) + int(val)
            assert torch.isfinite(obs["policy"]).all(), "non-finite observation"
            assert torch.isfinite(rew).all(), "non-finite reward"
        print(f"[check] random policy {args_cli.steps} steps ok, mean step reward {total_reward / args_cli.steps:.4f}")
        print("[check] termination counts over run:", term_counts)
        for line in fell_log:
            print("[check] fell:", line)
    print("[check] done")
