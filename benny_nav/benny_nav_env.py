# SPDX-License-Identifier: MIT
"""Benny A->B navigation with obstacle avoidance (Isaac Lab 3.0 direct RL environment).

* Observation (policy): 36 normalised LiDAR ranges (360 deg, fixed order, robot frame, counter-clockwise from the
  forward axis) + goal in the robot frame (dist, sin psi, cos psi) + previous action (one-hot) + body velocities.
* Action: ONE integer per env (stored as a float in a 1-D Box): 0 DRIVE, 1 TURN_L, 2 TURN_R, 3 STOP.
  The decoder zeroes the unused component, so v and omega are never non-zero together.
* Reward: progress + success bonus - collision - time - soft clearance penalty.
"""

from __future__ import annotations

from collections.abc import Sequence

import torch
import warp as wp

from isaaclab.envs import DirectRLEnv
from isaaclab.utils.math import quat_apply, quat_mul, wrap_to_pi

from .assets import BODY_TO_FORWARD_X_QUAT, WHEEL_RADIUS, WHEEL_TRACK
from .benny_nav_env_cfg import (
    ARENA_SIZE_X,
    ARENA_SIZE_Y,
    NUM_BEAMS,
    OBSTACLE_HEIGHT,
    OBSTACLE_SIZES,
    PARK_Z,
    WALL_HEIGHT,
    WALL_THICKNESS,
    BennyNavEnvCfg,
)

ACTION_DRIVE, ACTION_TURN_L, ACTION_TURN_R, ACTION_STOP = 0, 1, 2, 3


def _yaw_quat(yaw: torch.Tensor) -> torch.Tensor:
    """Quaternion (x, y, z, w) of a rotation about world z."""
    half = 0.5 * yaw
    zeros = torch.zeros_like(yaw)
    return torch.stack([zeros, zeros, torch.sin(half), torch.cos(half)], dim=-1)


class BennyNavEnv(DirectRLEnv):
    cfg: BennyNavEnvCfg

    def __init__(self, cfg: BennyNavEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        self._robot = self.scene["robot"]
        self._lidar = self.scene["lidar"]
        self._goal_obj = self.scene["goal"]
        self._walls = [self.scene[name] for name in ("wall_n", "wall_s", "wall_e", "wall_w")]
        self._boxes = [self.scene[f"box_{i}"] for i in range(len(OBSTACLE_SIZES))]

        wheel_ids, _ = self._robot.find_joints(["left_wheel_joint", "right_wheel_joint"], preserve_order=True)
        self._wheel_ids = torch.tensor(wheel_ids, device=self.device)

        dev = self.device
        # Primitive table: row = action id -> (v, omega). Exactly one component is non-zero.
        self._prim_v = torch.tensor([cfg.v_const, 0.0, 0.0, 0.0], device=dev)
        self._prim_w = torch.tensor([0.0, cfg.omega_const, -cfg.omega_const, 0.0], device=dev)
        self._wheel_sign = torch.tensor([cfg.left_wheel_sign, cfg.right_wheel_sign], device=dev)

        self._forward_body = torch.tensor([[0.0, 0.0, 1.0]], device=dev)  # body +z is the "front" (see assets)
        self._up_body = torch.tensor([[0.0, 1.0, 0.0]], device=dev)  # body +y is "up"
        self._q_nominal = torch.tensor([BODY_TO_FORWARD_X_QUAT], device=dev)
        self._obstacle_radius = torch.tensor([0.5 * (sx**2 + sy**2) ** 0.5 for sx, sy in OBSTACLE_SIZES], device=dev)

        n = self.num_envs
        self._action = torch.full((n,), ACTION_STOP, dtype=torch.long, device=dev)
        self._prev_action = torch.full((n,), ACTION_STOP, dtype=torch.long, device=dev)
        self._wheel_targets = torch.zeros(n, 2, device=dev)
        self._wheel_cmd = torch.zeros(n, 2, device=dev)  # rate-limited command actually sent to the drives
        self._goal_xy_w = torch.zeros(n, 2, device=dev)
        self._dist_prev = torch.zeros(n, device=dev)

        # per-step flags (filled in _get_dones, consumed in _get_rewards / _reset_idx)
        self._success = torch.zeros(n, dtype=torch.bool, device=dev)
        self._collision = torch.zeros(n, dtype=torch.bool, device=dev)
        self._out_of_bounds = torch.zeros(n, dtype=torch.bool, device=dev)
        self._fell = torch.zeros(n, dtype=torch.bool, device=dev)
        self._min_range = torch.full((n,), cfg.lidar_d_max, device=dev)
        self._dist = torch.zeros(n, device=dev)
        self._episode_valid = torch.zeros(n, dtype=torch.bool, device=dev)  # False until the first reset happened
        self._lidar_stale = False  # set by _reset_idx: the cached scan predates the teleport

        # curriculum
        self._level = int(max(0, min(cfg.curriculum_start_level, len(cfg.curriculum_levels) - 1)))
        self._cur_successes = 0
        self._cur_episodes = 0

    # ------------------------------------------------------------------ actions

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        act = actions.reshape(self.num_envs, -1)[:, 0].round().long().clamp(0, self.cfg.num_primitives - 1)
        self._prev_action = self._action
        self._action = act
        v = self._prim_v[act]
        w = self._prim_w[act]
        # Hard constraint: exactly one primitive at a time (never v and omega together).
        w = torch.where(v != 0.0, torch.zeros_like(w), w)
        # turn_gain compensates the in-place scrubbing of the casters (measured yaw rate < kinematic prediction).
        v_left = v - 0.5 * WHEEL_TRACK * self.cfg.turn_gain * w
        v_right = v + 0.5 * WHEEL_TRACK * self.cfg.turn_gain * w
        wheels = torch.stack([v_left, v_right], dim=-1) / WHEEL_RADIUS
        self._wheel_targets = wheels * self._wheel_sign

    def _apply_action(self) -> None:
        # Rate-limit the wheel velocity command (called every physics step) so abrupt primitive switches
        # (e.g. DRIVE -> TURN) do not pitch the top-heavy robot over.
        max_delta = self.cfg.wheel_accel_limit * self.physics_dt
        self._wheel_cmd = self._wheel_cmd + (self._wheel_targets - self._wheel_cmd).clamp(-max_delta, max_delta)
        self._robot.set_joint_velocity_target_index(target=self._wheel_cmd, joint_ids=self._wheel_ids)

    # ------------------------------------------------------------------ state helpers

    def _robot_state(self) -> dict[str, torch.Tensor]:
        data = self._robot.data
        quat = data.root_quat_w.torch
        pos = data.root_pos_w.torch
        fwd = quat_apply(quat, self._forward_body.expand(self.num_envs, 3))
        up = quat_apply(quat, self._up_body.expand(self.num_envs, 3))
        heading = torch.atan2(fwd[:, 1], fwd[:, 0])
        lin_vel = data.root_lin_vel_w.torch
        ang_vel = data.root_ang_vel_w.torch
        return {
            "pos": pos,
            "heading": heading,
            "up_z": up[:, 2],
            "v_fwd": (lin_vel[:, :2] * fwd[:, :2]).sum(dim=-1),
            "w_z": ang_vel[:, 2],
        }

    def _lidar_ranges(self) -> torch.Tensor:
        """Ranges [m] of the 360 deg scan, shape (N, NUM_BEAMS), clamped to d_max (misses -> d_max)."""
        hits = self._lidar.data.ray_hits_w.torch
        origin = self._lidar.data.pos_w.torch
        dist = torch.linalg.norm(hits - origin.unsqueeze(1), dim=-1)
        dist = torch.nan_to_num(dist, nan=self.cfg.lidar_d_max, posinf=self.cfg.lidar_d_max)
        dist = dist.clamp(max=self.cfg.lidar_d_max)
        # Raw pattern starts at -180 deg (backwards); roll so beam 0 = forward, then counter-clockwise (+10 deg each).
        return torch.roll(dist, shifts=-(NUM_BEAMS // 2), dims=1)

    # ------------------------------------------------------------------ observations

    def _get_observations(self) -> dict:
        s = self._robot_state()
        if self._lidar_stale:
            # Observations right after a reset must see the new poses, not the previous episode's scan.
            self._lidar.update(0.0, force_recompute=True)
            self._lidar_stale = False
        ranges = self._lidar_ranges()
        noise_std = self.cfg.lidar_noise_levels[min(self._level, len(self.cfg.lidar_noise_levels) - 1)]
        if noise_std > 0.0:
            ranges = (ranges + noise_std * torch.randn_like(ranges)).clamp(0.0, self.cfg.lidar_d_max)

        rel = self._goal_xy_w - s["pos"][:, :2]
        dist = torch.linalg.norm(rel, dim=-1)
        bearing = wrap_to_pi(torch.atan2(rel[:, 1], rel[:, 0]) - s["heading"])
        goal = torch.stack([dist / self.cfg.goal_dist_scale, torch.sin(bearing), torch.cos(bearing)], dim=-1)

        prev = torch.nn.functional.one_hot(self._action, self.cfg.num_primitives).float()
        twist = torch.stack([s["v_fwd"] / self.cfg.v_const, s["w_z"] / self.cfg.omega_const], dim=-1)

        obs = torch.cat([ranges / self.cfg.lidar_d_max, goal, prev, twist], dim=-1)
        return {"policy": obs}

    # ------------------------------------------------------------------ rewards / dones

    def _update_flags(self) -> None:
        cfg = self.cfg
        s = self._robot_state()
        rel_env = s["pos"][:, :2] - self.scene.env_origins[:, :2]
        self._dist = torch.linalg.norm(self._goal_xy_w - s["pos"][:, :2], dim=-1)
        self._min_range = self._lidar_ranges().min(dim=-1).values
        success = self._dist < cfg.goal_success_radius
        self._success = success
        self._collision = (self._min_range < cfg.collision_dist) & ~success
        self._out_of_bounds = (
            (rel_env[:, 0].abs() > ARENA_SIZE_X / 2 + cfg.out_of_bounds_margin)
            | (rel_env[:, 1].abs() > ARENA_SIZE_Y / 2 + cfg.out_of_bounds_margin)
        ) & ~success
        self._fell = (s["up_z"] < 0.5) & ~success

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        self._update_flags()
        terminated = self._success | self._collision | self._out_of_bounds | self._fell
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        return terminated, time_out

    def _get_rewards(self) -> torch.Tensor:
        cfg = self.cfg
        progress = self._dist_prev - self._dist
        self._dist_prev = self._dist.clone()
        failed = self._collision | self._out_of_bounds | self._fell
        near = torch.clamp(cfg.d_safe - self._min_range, min=0.0) ** 2
        return (
            cfg.w_progress * progress
            + cfg.w_reach * self._success.float()
            - cfg.w_collision * failed.float()
            - cfg.w_time * self.step_dt
            - cfg.w_near * near * self.step_dt
        )

    # ------------------------------------------------------------------ reset

    def _reset_idx(self, env_ids: Sequence[int] | torch.Tensor | None):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        elif isinstance(env_ids, wp.array):
            env_ids = wp.to_torch(env_ids)
        env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        n = env_ids.numel()
        if n == 0:
            return

        # ---- logging + curriculum bookkeeping for the episodes that just ended
        valid = self._episode_valid[env_ids]
        extras: dict[str, float] = {}
        if bool(valid.any()):
            ended = env_ids[valid]
            extras["Metrics/success_rate"] = self._success[ended].float().mean().item()
            extras["Episode_Termination/success"] = int(self._success[ended].sum().item())
            extras["Episode_Termination/collision"] = int(self._collision[ended].sum().item())
            extras["Episode_Termination/out_of_bounds"] = int(self._out_of_bounds[ended].sum().item())
            extras["Episode_Termination/fell"] = int(self._fell[ended].sum().item())
            extras["Episode_Termination/time_out"] = int(self.reset_time_outs[ended].sum().item())
            self._cur_successes += int(self._success[ended].sum().item())
            self._cur_episodes += int(ended.numel())
        self._maybe_promote_level()
        extras["Curriculum/level"] = float(self._level)
        self.extras["log"] = extras

        super()._reset_idx(env_ids)

        cfg = self.cfg
        dev = self.device
        max_obstacles, goal_max = cfg.curriculum_levels[self._level]
        origins = self.scene.env_origins[env_ids]  # (n, 3)

        # ---- start A, goal B
        half_x = ARENA_SIZE_X / 2 - cfg.spawn_margin
        half_y = ARENA_SIZE_Y / 2 - cfg.spawn_margin
        lo = torch.tensor([-half_x, -half_y], device=dev)
        hi = torch.tensor([half_x, half_y], device=dev)

        def sample_xy(count: int) -> torch.Tensor:
            return lo + (hi - lo) * torch.rand(count, 2, device=dev)

        start = sample_xy(n)
        yaw = (torch.rand(n, device=dev) * 2.0 - 1.0) * torch.pi
        goal = torch.zeros(n, 2, device=dev)
        found = torch.zeros(n, dtype=torch.bool, device=dev)
        for _ in range(40):
            cand = sample_xy(n)
            d = torch.linalg.norm(cand - start, dim=-1)
            ok = (d >= cfg.goal_min_dist) & (d <= goal_max) & ~found
            goal[ok] = cand[ok]
            found |= ok
        if not bool(found.all()):  # fallback: random direction at a random valid distance, clamped to the arena
            ang = torch.rand(n, device=dev) * 2 * torch.pi
            dist = cfg.goal_min_dist + (goal_max - cfg.goal_min_dist) * torch.rand(n, device=dev)
            cand = start + torch.stack([torch.cos(ang), torch.sin(ang)], dim=-1) * dist.unsqueeze(-1)
            cand = torch.max(torch.min(cand, hi), lo)
            goal[~found] = cand[~found]

        # ---- obstacles (rejection sampling, inactive slots are parked below the ground)
        n_obs = torch.randint(0, max_obstacles + 1, (n,), device=dev)
        placed_pos: list[torch.Tensor] = []
        placed_ok: list[torch.Tensor] = []
        box_pos = torch.zeros(len(self._boxes), n, 2, device=dev)
        box_active = torch.zeros(len(self._boxes), n, dtype=torch.bool, device=dev)
        box_yaw = (torch.rand(len(self._boxes), n, device=dev) * 2.0 - 1.0) * torch.pi
        obs_lo = torch.tensor([-ARENA_SIZE_X / 2 + 0.12, -ARENA_SIZE_Y / 2 + 0.12], device=dev)
        obs_hi = torch.tensor([ARENA_SIZE_X / 2 - 0.12, ARENA_SIZE_Y / 2 - 0.12], device=dev)
        clearance = 0.22  # robot radius (~0.1) + margin
        for k in range(len(self._boxes)):
            rad = self._obstacle_radius[k]
            ok_k = torch.zeros(n, dtype=torch.bool, device=dev)
            pos_k = torch.zeros(n, 2, device=dev)
            for _ in range(30):
                cand = obs_lo + (obs_hi - obs_lo) * torch.rand(n, 2, device=dev)
                valid = (torch.linalg.norm(cand - start, dim=-1) > rad + clearance) & (
                    torch.linalg.norm(cand - goal, dim=-1) > rad + clearance
                )
                for prev_pos, prev_ok, prev_k in zip(placed_pos, placed_ok, range(k)):
                    gap = torch.linalg.norm(cand - prev_pos, dim=-1) > rad + self._obstacle_radius[prev_k] + 0.18
                    valid &= gap | ~prev_ok
                take = valid & ~ok_k
                pos_k[take] = cand[take]
                ok_k |= take
            placed_pos.append(pos_k)
            placed_ok.append(ok_k)
            box_pos[k] = pos_k
            box_active[k] = ok_k & (n_obs > k)

        # ---- write poses
        zeros_vel = torch.zeros(n, 6, device=dev)
        robot_pos = torch.cat([origins[:, :2] + start, torch.full((n, 1), 0.025, device=dev)], dim=-1)
        robot_quat = quat_mul(_yaw_quat(yaw), self._q_nominal.expand(n, 4))
        self._robot.write_root_pose_to_sim_index(root_pose=torch.cat([robot_pos, robot_quat], dim=-1), env_ids=env_ids)
        self._robot.write_root_velocity_to_sim_index(root_velocity=zeros_vel, env_ids=env_ids)
        joint_pos = self._robot.data.default_joint_pos.torch[env_ids]
        joint_vel = self._robot.data.default_joint_vel.torch[env_ids]
        self._robot.write_joint_position_to_sim_index(position=joint_pos, env_ids=env_ids)
        self._robot.write_joint_velocity_to_sim_index(velocity=joint_vel, env_ids=env_ids)

        identity = torch.tensor([0.0, 0.0, 0.0, 1.0], device=dev).expand(n, 4)
        # goal marker
        goal_pose = torch.cat([origins[:, :2] + goal, torch.full((n, 1), 0.002, device=dev), identity], dim=-1)
        self._goal_obj.write_root_pose_to_sim_index(root_pose=goal_pose, env_ids=env_ids)
        # obstacles
        for k, box in enumerate(self._boxes):
            z = torch.where(box_active[k], torch.full((n,), OBSTACLE_HEIGHT / 2, device=dev), torch.full((n,), PARK_Z, device=dev))
            xy = origins[:, :2] + torch.where(box_active[k].unsqueeze(-1), box_pos[k], torch.zeros_like(box_pos[k]))
            pose = torch.cat([xy, z.unsqueeze(-1), _yaw_quat(box_yaw[k])], dim=-1)
            box.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        # walls
        walls_on = torch.rand(n, device=dev) < cfg.walls_probability
        wall_xy = [
            (0.0, ARENA_SIZE_Y / 2 + WALL_THICKNESS / 2),
            (0.0, -ARENA_SIZE_Y / 2 - WALL_THICKNESS / 2),
            (ARENA_SIZE_X / 2 + WALL_THICKNESS / 2, 0.0),
            (-ARENA_SIZE_X / 2 - WALL_THICKNESS / 2, 0.0),
        ]
        for wall, (wx, wy) in zip(self._walls, wall_xy):
            z = torch.where(walls_on, torch.full((n,), WALL_HEIGHT / 2, device=dev), torch.full((n,), PARK_Z, device=dev))
            xy = origins[:, :2] + torch.tensor([wx, wy], device=dev)
            pose = torch.cat([xy, z.unsqueeze(-1), identity], dim=-1)
            wall.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)

        # ---- reset bookkeeping
        self._goal_xy_w[env_ids] = origins[:, :2] + goal
        self._dist_prev[env_ids] = torch.linalg.norm(goal - start, dim=-1)
        self._dist[env_ids] = self._dist_prev[env_ids]
        self._action[env_ids] = ACTION_STOP
        self._prev_action[env_ids] = ACTION_STOP
        self._wheel_targets[env_ids] = 0.0
        self._wheel_cmd[env_ids] = 0.0
        self._success[env_ids] = False
        self._collision[env_ids] = False
        self._out_of_bounds[env_ids] = False
        self._fell[env_ids] = False
        self._min_range[env_ids] = cfg.lidar_d_max
        self._episode_valid[env_ids] = True
        self._lidar_stale = True

    def _maybe_promote_level(self) -> None:
        cfg = self.cfg
        if not cfg.curriculum_enabled or self._cur_episodes < cfg.curriculum_window_episodes:
            return
        rate = self._cur_successes / max(1, self._cur_episodes)
        if rate >= cfg.curriculum_promote_success and self._level < len(cfg.curriculum_levels) - 1:
            self._level += 1
            print(f"[benny_nav] curriculum: success {rate:.2f} -> level {self._level}", flush=True)
        self._cur_successes = 0
        self._cur_episodes = 0
