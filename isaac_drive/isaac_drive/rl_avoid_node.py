"""rl_avoid_node — 학습된 Residual RL 정책으로 장애물 회피.

두 가지 동작 모드
-----------------
goal mode (relay_mode=False, 기본)
  · /move_base_simple/goal 로 목표 지정 → 직접 cmd_vel 생성.
  · 독립 실행 시 사용.

relay mode (relay_mode=True)
  · upstream_cmd_vel_topic(기본 /mission/cmd_vel_raw) 의 cmd_vel 을 받아
    RL 장애물 회피 레이어를 거쳐 /cmd_vel 로 전달.
  · integrated_localization.launch.py 와 연동 시 사용.
  · mission_manager_node 의 cmd_vel_topic 을 /mission/cmd_vel_raw 로 바꾸고
    이 노드가 그 다음 단계를 처리.

흐름 (relay mode)
  coverage_node → /coverage/cmd_vel_raw
  mission_manager_node → /mission/cmd_vel_raw
  rl_avoid_node → /cmd_vel   ← 최종 로버 제어

훈련 포맷 (rough_env_cfg.py)
-----------------------------
  raycast : 4.0×2.4 m, res=0.2 → 21×13=273 rays
  obs(166): goal_body(3) + goal_yaw(2) + raycast_prom(153) + lin_vel(3) + ang_vel(3) + last_action(2)
  action  : [lin_residual, ang_residual]
"""
from __future__ import annotations

import math
import os
import threading
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

# ---------------------------------------------------------------------------
# RL actor (PyTorch)
# ---------------------------------------------------------------------------
try:
    import torch
    import torch.nn as nn
    _TORCH_OK = True
except ImportError:
    _TORCH_OK = False


def _build_and_load_actor(model_path: str):
    if not _TORCH_OK:
        raise ImportError("torch 미설치")
    ckpt = torch.load(model_path, map_location="cpu")
    sd   = ckpt["actor_state_dict"]
    obs_dim = sd["mlp.0.weight"].shape[1]
    act_dim = sd["mlp.4.weight"].shape[0]
    actor = nn.Sequential(
        nn.Linear(obs_dim, 128), nn.ELU(),
        nn.Linear(128, 128),     nn.ELU(),
        nn.Linear(128, act_dim),
    )
    actor.load_state_dict({
        "0.weight": sd["mlp.0.weight"], "0.bias": sd["mlp.0.bias"],
        "2.weight": sd["mlp.2.weight"], "2.bias": sd["mlp.2.bias"],
        "4.weight": sd["mlp.4.weight"], "4.bias": sd["mlp.4.bias"],
    })
    actor.eval()
    obs_mean = sd["obs_normalizer._mean"].squeeze(0)
    obs_std  = sd["obs_normalizer._std"].squeeze(0)
    return actor, obs_mean, obs_std, obs_dim, act_dim


# ---------------------------------------------------------------------------
# Base controller
# ---------------------------------------------------------------------------
@dataclass
class BaseCfg:
    cruise_speed:      float = 2.5
    goal_turn_k:       float = 2.5
    prom_radius:       int   = 2
    height_thresh:     float = 0.15
    front_range:       float = 2.0
    corridor:          float = 0.7
    avoid_base_ang:    float = 1.0
    avoid_max_ang:     float = 2.2
    avoid_lin_scale:   float = 0.65
    lin_residual_scale: float = 1.0
    ang_residual_scale: float = 1.5
    max_lin:           float = 3.0
    max_ang:           float = 2.5


def _compute_base(rover_x, rover_y, rover_yaw, goal_x, goal_y,
                   z_grid, cell_xy, rows, cols, cfg: BaseCfg):
    cy = math.cos(rover_yaw); sy = math.sin(rover_yaw)
    dx_g = goal_x - rover_x;  dy_g = goal_y - rover_y
    yaw_err = math.atan2(-dx_g*sy + dy_g*cy, dx_g*cy + dy_g*sy)
    base_lin = float(cfg.cruise_speed * max(0.0, math.cos(yaw_err)))
    base_ang = float(np.clip(cfg.goal_turn_k * yaw_err, -cfg.max_ang, cfg.max_ang))

    r = cfg.prom_radius
    if rows <= 2*r or cols <= 2*r:
        return base_lin, base_ang

    z_fill = np.where(np.isfinite(z_grid), z_grid.astype(np.float32), 0.0)
    core = z_fill[r:rows-r, r:cols-r]
    acc  = np.zeros_like(core); cnt = 0
    for dr in (-r, 0, r):
        for dc in (-r, 0, r):
            if dr == 0 and dc == 0: continue
            acc += z_fill[r+dr:rows-r+dr, r+dc:cols-r+dc]; cnt += 1
    prom = core - acc / cnt

    xy     = cell_xy.reshape(rows, cols, 2)
    cxy    = xy[r:rows-r, r:cols-r, :]
    fwd_c  = (cxy[...,0]-rover_x)*cy + (cxy[...,1]-rover_y)*sy
    lat_c  = -(cxy[...,0]-rover_x)*sy + (cxy[...,1]-rover_y)*cy

    is_obs   = np.isfinite(prom) & (prom > cfg.height_thresh)
    is_block = is_obs & (fwd_c>0) & (fwd_c<cfg.front_range) & (np.abs(lat_c)<cfg.corridor)
    if not is_block.any():
        return base_lin, base_ang

    weight   = is_block.astype(np.float32) * np.clip(prom, 0.0, None)
    w_sum    = max(weight.sum(), 1e-6)
    lat_mean = float((weight*lat_c).sum() / w_sum)
    avoid_dir = -1.0 if lat_mean >= 0.0 else 1.0
    center_w  = float(np.clip(1.0 - abs(lat_mean)/cfg.corridor, 0.0, 1.0))
    fwd_sum   = float((weight*fwd_c).sum() / w_sum)
    near_w    = float(np.clip(1.0 - fwd_sum/cfg.front_range, 0.0, 1.0))
    intensity = max(center_w, near_w)
    avoid_ang = cfg.avoid_base_ang + intensity*(cfg.avoid_max_ang - cfg.avoid_base_ang)
    return base_lin * cfg.avoid_lin_scale, avoid_dir * avoid_ang


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def _quat_to_yaw(qx, qy, qz, qw) -> float:
    return math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))


def _default_model_path() -> Optional[str]:
    here = os.path.dirname(os.path.abspath(__file__))
    base = os.path.normpath(os.path.join(here, "..", "avoid_test_new_", "logs"))
    if os.path.isdir(base):
        for root, _, files in os.walk(base):
            if "best_final.pt" in files:
                return os.path.join(root, "best_final.pt")
    return None


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------
class RLAvoidNode(Node):
    def __init__(self) -> None:
        super().__init__("rl_avoid_node")

        # ── 파라미터 ──────────────────────────────────────────────────────
        self.declare_parameter("model_path", "")
        self.declare_parameter("ipc_path",   "/tmp/a2_raycast.npz")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("cmd_vel_topic",  "/cmd_vel")
        self.declare_parameter("odom_topic",     "/ground_truth/odom")
        self.declare_parameter("pose_topic",     "/rover/estimated_pose")
        # goal mode
        self.declare_parameter("goal_topic",         "/move_base_simple/goal")
        self.declare_parameter("goal_reach_dist_m",  0.5)
        # relay mode
        self.declare_parameter("relay_mode",              False)
        self.declare_parameter("upstream_cmd_vel_topic",  "/mission/cmd_vel_raw")
        # relay mode: 전방 N m 앞을 가상 goal 로 삼아 obs 구성
        self.declare_parameter("relay_virtual_goal_dist", 8.0)

        model_path   = str(self.get_parameter("model_path").value).strip() or _default_model_path() or ""
        self.ipc_path = str(self.get_parameter("ipc_path").value)
        publish_hz   = float(self.get_parameter("publish_hz").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        odom_topic   = str(self.get_parameter("odom_topic").value)
        pose_topic   = str(self.get_parameter("pose_topic").value)
        goal_topic   = str(self.get_parameter("goal_topic").value)
        self.goal_reach_dist     = float(self.get_parameter("goal_reach_dist_m").value)
        self.relay_mode          = bool(self.get_parameter("relay_mode").value)
        upstream_topic           = str(self.get_parameter("upstream_cmd_vel_topic").value)
        self.relay_vgoal_dist    = float(self.get_parameter("relay_virtual_goal_dist").value)

        # ── actor 로드 ────────────────────────────────────────────────────
        self.actor = self.obs_mean = self.obs_std = None
        if model_path and os.path.isfile(model_path):
            try:
                self.actor, self.obs_mean, self.obs_std, od, ad = \
                    _build_and_load_actor(model_path)
                self.get_logger().info(f"[RL] actor 로드 완료: {model_path}  obs={od} act={ad}")
            except Exception as e:
                self.get_logger().error(f"[RL] actor 로드 실패: {e}")
        else:
            self.get_logger().warn(f"[RL] model_path 없음 — base controller 만 동작")

        self.base_cfg = BaseCfg()

        # ── 상태 ──────────────────────────────────────────────────────────
        self.lock        = threading.Lock()
        self.rover_x     = 0.0; self.rover_y   = 0.0; self.rover_yaw = 0.0
        self.lin_vel     = [0.0, 0.0, 0.0]
        self.ang_vel     = [0.0, 0.0, 0.0]
        self.goal_x      = 0.0; self.goal_y    = 0.0
        self.has_goal    = False
        self._goal_reached   = False
        self._last_ipc_mtime = 0.0
        self._ipc_data       = None
        self._last_action    = [0.0, 0.0]
        # relay mode: upstream cmd_vel 캐시
        self._upstream_lin   = 0.0
        self._upstream_ang   = 0.0
        self._upstream_stamp = 0.0  # 마지막 수신 시각 (epoch)

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self._on_ekf, 10)

        if self.relay_mode:
            self.create_subscription(Twist, upstream_topic, self._on_upstream, 10)
            self.get_logger().info(
                f"[RL] relay mode — upstream={upstream_topic} → {cmd_vel_topic}")
        else:
            self.create_subscription(PoseStamped, goal_topic, self._on_goal, 1)
            self.get_logger().info(
                f"[RL] goal mode — goal={goal_topic} → {cmd_vel_topic}")

        # ── Publisher ─────────────────────────────────────────────────────
        self.pub_cmd = self.create_publisher(Twist, cmd_vel_topic, 1)
        self.create_timer(1.0 / publish_hz, self._tick)

    # ── callbacks ─────────────────────────────────────────────────────────

    def _on_goal(self, msg: PoseStamped) -> None:
        with self.lock:
            self.goal_x = float(msg.pose.position.x)
            self.goal_y = float(msg.pose.position.y)
            self.has_goal = True
            self._goal_reached = False
        self.get_logger().info(f"[RL] 새 goal → ({self.goal_x:+.2f}, {self.goal_y:+.2f})")

    def _on_odom(self, msg: Odometry) -> None:
        with self.lock:
            t = msg.twist.twist
            self.lin_vel = [t.linear.x, t.linear.y, t.linear.z]
            self.ang_vel = [t.angular.x, t.angular.y, t.angular.z]
            p = msg.pose.pose
            self.rover_x   = float(p.position.x)
            self.rover_y   = float(p.position.y)
            self.rover_yaw = _quat_to_yaw(
                float(p.orientation.x), float(p.orientation.y),
                float(p.orientation.z), float(p.orientation.w))

    def _on_ekf(self, msg: PoseWithCovarianceStamped) -> None:
        with self.lock:
            p = msg.pose.pose
            self.rover_x   = float(p.position.x)
            self.rover_y   = float(p.position.y)
            self.rover_yaw = _quat_to_yaw(
                float(p.orientation.x), float(p.orientation.y),
                float(p.orientation.z), float(p.orientation.w))

    def _on_upstream(self, msg: Twist) -> None:
        import time as _time
        with self.lock:
            self._upstream_lin   = float(msg.linear.x)
            self._upstream_ang   = float(msg.angular.z)
            self._upstream_stamp = _time.time()

    # ── IPC ───────────────────────────────────────────────────────────────

    def _read_ipc(self) -> Optional[dict]:
        if not os.path.isfile(self.ipc_path):
            return self._ipc_data
        try:
            st = os.stat(self.ipc_path)
        except FileNotFoundError:
            return self._ipc_data
        if st.st_mtime <= self._last_ipc_mtime:
            return self._ipc_data
        self._last_ipc_mtime = st.st_mtime
        try:
            d = np.load(self.ipc_path, allow_pickle=False)
            self._ipc_data = {
                "cell_xyz": d["cell_world_xyz"],
                "is_miss":  d["cell_is_miss"],
                "rows":     int(d["grid_rows"]),
                "cols":     int(d["grid_cols"]),
            }
        except Exception as e:
            self.get_logger().warn(f"[RL] IPC 로드 실패: {e}")
        return self._ipc_data

    # ── obs 구성 ──────────────────────────────────────────────────────────

    def _build_obs(self, rover_x, rover_y, rover_yaw, goal_x, goal_y,
                   lin_vel, ang_vel, last_action, ipc) -> Optional[np.ndarray]:
        cy = math.cos(rover_yaw); sy = math.sin(rover_yaw)
        dx_g = goal_x - rover_x;  dy_g = goal_y - rover_y
        fwd_g = dx_g*cy + dy_g*sy
        lat_g = -dx_g*sy + dy_g*cy
        dist_g = math.hypot(dx_g, dy_g)
        goal_xyz = np.array([fwd_g, lat_g, dist_g], dtype=np.float32)

        desired_yaw = math.atan2(dy_g, dx_g)
        err = desired_yaw - rover_yaw
        goal_yaw = np.array([math.sin(err), math.cos(err)], dtype=np.float32)

        rows = ipc["rows"]; cols = ipc["cols"]
        cell_xyz = ipc["cell_xyz"]; is_miss = ipc["is_miss"]
        if cell_xyz.shape[0] != rows * cols:
            return None

        z_raw  = cell_xyz[:, 2].reshape(rows, cols).astype(np.float32)
        z_fill = np.where(is_miss.reshape(rows, cols), 0.0, z_raw)

        r = self.base_cfg.prom_radius
        if rows > 2*r and cols > 2*r:
            core = z_fill[r:rows-r, r:cols-r]
            acc  = np.zeros_like(core); cnt = 0
            for dr in (-r, 0, r):
                for dc in (-r, 0, r):
                    if dr == 0 and dc == 0: continue
                    acc += z_fill[r+dr:rows-r+dr, r+dc:cols-r+dc]; cnt += 1
            prom = np.where(np.isfinite(core - acc/cnt), core - acc/cnt, 0.0)
            prom_flat = np.clip(prom, -2.0, 2.0).flatten().astype(np.float32)
        else:
            prom_flat = np.zeros(153, dtype=np.float32)

        obs = np.concatenate([
            goal_xyz, goal_yaw, prom_flat,
            np.array(lin_vel, dtype=np.float32),
            np.array(ang_vel, dtype=np.float32),
            np.array(last_action, dtype=np.float32),
        ])
        return np.nan_to_num(obs, nan=0.0, posinf=2.0, neginf=-2.0)

    # ── 장애물 탐지 (relay mode 스위치용) ────────────────────────────────

    def _has_blocking_obstacle(self, ipc: dict,
                                rover_x: float, rover_y: float,
                                rover_yaw: float) -> bool:
        """전방 corridor 내에 blocking obstacle 이 있으면 True."""
        cfg  = self.base_cfg
        rows = ipc["rows"]; cols = ipc["cols"]
        r    = cfg.prom_radius
        if rows <= 2*r or cols <= 2*r:
            return False

        cell_xyz = ipc["cell_xyz"]
        if cell_xyz.shape[0] != rows * cols:
            return False

        z_fill = np.where(
            ipc["is_miss"].reshape(rows, cols),
            0.0,
            cell_xyz[:, 2].reshape(rows, cols).astype(np.float32))

        core = z_fill[r:rows-r, r:cols-r]
        acc  = np.zeros_like(core); cnt = 0
        for dr in (-r, 0, r):
            for dc in (-r, 0, r):
                if dr == 0 and dc == 0: continue
                acc += z_fill[r+dr:rows-r+dr, r+dc:cols-r+dc]; cnt += 1
        prom    = core - acc / cnt
        is_obs  = np.isfinite(prom) & (prom > cfg.height_thresh)

        cy = math.cos(rover_yaw); sy = math.sin(rover_yaw)
        cxy = cell_xyz[:, :2].reshape(rows, cols, 2)[r:rows-r, r:cols-r, :]
        fwd_c = (cxy[...,0]-rover_x)*cy + (cxy[...,1]-rover_y)*sy
        lat_c = -(cxy[...,0]-rover_x)*sy + (cxy[...,1]-rover_y)*cy

        is_block = (is_obs
                    & (fwd_c > 0.0) & (fwd_c < cfg.front_range)
                    & (np.abs(lat_c) < cfg.corridor))
        return bool(is_block.any())

    # ── 메인 루프 ─────────────────────────────────────────────────────────

    def _tick(self) -> None:
        import time as _time
        with self.lock:
            rover_x   = self.rover_x;   rover_y   = self.rover_y
            rover_yaw = self.rover_yaw
            lin_vel   = list(self.lin_vel)
            ang_vel   = list(self.ang_vel)
            last_act  = list(self._last_action)

            if self.relay_mode:
                up_age = _time.time() - self._upstream_stamp
                if up_age > 0.3:           # upstream 끊기면 정지
                    self.pub_cmd.publish(Twist())
                    return
                up_lin = self._upstream_lin
                up_ang = self._upstream_ang
            else:
                if not self.has_goal or self._goal_reached:
                    return
                goal_x = self.goal_x
                goal_y = self.goal_y

        # ── goal mode: 도착 판정 ─────────────────────────────────────────
        if not self.relay_mode:
            dist_to_goal = math.hypot(goal_x - rover_x, goal_y - rover_y)
            if self.goal_reach_dist > 0 and dist_to_goal < self.goal_reach_dist:
                self.pub_cmd.publish(Twist())
                with self.lock:
                    self._goal_reached = True
                self.get_logger().info(f"[RL] 목표 도착! dist={dist_to_goal:.2f}m")
                return

        # ── IPC 읽기 ─────────────────────────────────────────────────────
        ipc = self._read_ipc()

        # ── relay mode 분기 ──────────────────────────────────────────────
        if self.relay_mode:
            # upstream 정지 명령 → 그대로 전달
            if abs(up_lin) < 0.01 and abs(up_ang) < 0.01:
                self.pub_cmd.publish(Twist())
                return

            # IPC 없음 또는 장애물 없음 → upstream 그대로 통과
            if ipc is None or not self._has_blocking_obstacle(
                    ipc, rover_x, rover_y, rover_yaw):
                t = Twist(); t.linear.x = up_lin; t.angular.z = up_ang
                self.pub_cmd.publish(t)
                return

            # 장애물 감지 → RL 회피 모드
            # 가상 goal = 전방 relay_vgoal_dist m
            vd = self.relay_vgoal_dist
            goal_x = rover_x + math.cos(rover_yaw) * vd
            goal_y = rover_y + math.sin(rover_yaw) * vd
            print(f"[RL] ⚠ 장애물 감지 — RL 회피 모드 활성화")
        else:
            if ipc is None:
                return

        # ── Base controller + RL residual ────────────────────────────────
        cfg     = self.base_cfg
        rows    = ipc["rows"]; cols = ipc["cols"]
        z_grid  = np.where(ipc["is_miss"].reshape(rows, cols), 0.0,
                           ipc["cell_xyz"][:, 2].reshape(rows, cols).astype(np.float32))
        cell_xy = ipc["cell_xyz"][:, :2]

        base_lin, base_ang = _compute_base(
            rover_x, rover_y, rover_yaw, goal_x, goal_y,
            z_grid, cell_xy, rows, cols, cfg)

        # relay: upstream 속도 크기 존중
        if self.relay_mode:
            up_speed = abs(up_lin)
            if up_speed > 0.01:
                base_lin = up_speed * (1.0 if base_lin >= 0 else cfg.avoid_lin_scale)

        # RL residual
        lin_res = ang_res = 0.0
        if self.actor is not None:
            obs = self._build_obs(rover_x, rover_y, rover_yaw,
                                   goal_x, goal_y, lin_vel, ang_vel, last_act, ipc)
            if obs is not None and len(obs) == 166:
                try:
                    with torch.no_grad():
                        obs_t = torch.from_numpy(obs).float().unsqueeze(0)
                        obs_n = torch.clamp(
                            (obs_t - self.obs_mean) / (self.obs_std + 1e-8),
                            -5.0, 5.0)
                        act   = self.actor(obs_n).squeeze(0)
                        lin_res = float(act[0].item())
                        ang_res = float(act[1].item())
                except Exception as e:
                    self.get_logger().warn(f"[RL] inference 오류: {e}")

        final_lin = float(np.clip(
            base_lin + lin_res * cfg.lin_residual_scale, -cfg.max_lin, cfg.max_lin))
        final_ang = float(np.clip(
            base_ang + ang_res * cfg.ang_residual_scale, -cfg.max_ang, cfg.max_ang))

        with self.lock:
            self._last_action = [lin_res, ang_res]

        twist = Twist()
        twist.linear.x  = final_lin
        twist.angular.z = final_ang
        self.pub_cmd.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RLAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
