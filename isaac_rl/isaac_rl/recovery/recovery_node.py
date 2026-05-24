"""recovery_node.py — ROS2 로버 복구 노드.

기능:
  1. /imu/data 구독 → 넘어짐 감지 (|roll| > 45° 또는 |pitch| > 45°, 2초 지속)
  2. /recovery/start 서비스 → 복구 모드 시작 (웹 버튼에서 호출)
  3. recovery_policy.pt 로드 → M0609 관절 제어
  4. /joint_trajectory 발행 → M0609 실행
  5. /recovery/status 발행 → 웹 HUD 상태 표시

실행:
  source /opt/ros/humble/setup.bash
  source ~/dev_ws/rover_ws/install/setup.bash
  ros2 run isaac_rl recovery_node
"""
from __future__ import annotations

import math
import time
import threading
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    import torch
    _HAS_TORCH = True
except ImportError:
    _HAS_TORCH = False

# ── 정책 경로 ──────────────────────────────────────────────────────────────
_POLICY_PATH = Path(__file__).resolve().parents[2] / "policies" / "recovery_policy.pt"

# ── 파라미터 ──────────────────────────────────────────────────────────────
FALL_THRESHOLD_DEG   = 45.0   # 이 각도 이상이면 넘어진 것으로 판단
FALL_SUSTAIN_SEC     = 2.0    # 넘어짐 상태 지속 시간 (초)
RECOVERY_TIMEOUT_SEC = 15.0   # 복구 최대 시간 (초)
CONTROL_HZ           = 20     # 제어 주기 (Hz)

# M0609 관절 한계 (rad) — 스케일 변환용
_JOINT_LO = np.array([-3.14, -1.57, -1.57, -3.14, -1.57, -3.14])
_JOINT_HI = np.array([ 3.14,  1.57,  2.53,  3.14,  1.57,  3.14])
_JOINT_MID   = (_JOINT_LO + _JOINT_HI) / 2.0
_JOINT_SCALE = (_JOINT_HI - _JOINT_LO) / 2.0

M0609_JOINTS = ["joint_1", "joint_2", "joint_3",
                "joint_4", "joint_5", "joint_6"]


def _quat_to_euler(q: Quaternion):
    """geometry_msgs/Quaternion → (roll, pitch, yaw) rad."""
    x, y, z, w = q.x, q.y, q.z, q.w
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (z * z + y * y)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class RecoveryNode(Node):
    def __init__(self):
        super().__init__("rover_recovery_node")
        self._lock = threading.Lock()

        # 상태
        self._roll = 0.0
        self._pitch = 0.0
        self._joint_pos = np.zeros(6)
        self._joint_vel = np.zeros(6)
        self._fallen_since: float | None = None
        self._recovering = False
        self._recovery_start: float | None = None

        # 정책 로드
        self._policy = None
        self._load_policy()

        # QoS
        _sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 구독
        self.create_subscription(Imu, "/imu/data", self._on_imu, _sensor_qos)
        self.create_subscription(JointState, "/joint_states_raw",
                                 self._on_joint_states, 10)

        # 발행
        self._joint_pub  = self.create_publisher(JointState, "/m0609/joint_command", 10)
        self._status_pub = self.create_publisher(String, "/recovery/status", 10)

        # 서비스
        self.create_service(Trigger, "/recovery/start", self._on_start_srv)
        self.create_service(Trigger, "/recovery/stop",  self._on_stop_srv)

        # 제어 루프 타이머
        self.create_timer(1.0 / CONTROL_HZ, self._control_loop)

        self.get_logger().info(
            f"RecoveryNode 준비 — 정책: "
            f"{'로드됨' if self._policy else '없음 (학습 필요)'}"
        )

    # ── 정책 로드 ─────────────────────────────────────────────────────────
    def _load_policy(self):
        if not _HAS_TORCH:
            self.get_logger().warn("torch 없음 — 정책 로드 불가")
            return
        if not _POLICY_PATH.exists():
            self.get_logger().warn(
                f"정책 파일 없음: {_POLICY_PATH}\n"
                f"  → train_recovery.py 로 학습 후 policies/recovery_policy.pt 생성 필요"
            )
            return
        try:
            state_dict = torch.load(str(_POLICY_PATH), map_location="cpu")
            # 간단한 MLP (학습 시 ActorCritic actor 부분만 추출)
            from torch import nn
            self._policy = nn.Sequential(
                nn.Linear(22, 256), nn.ELU(),
                nn.Linear(256, 128), nn.ELU(),
                nn.Linear(128, 64),  nn.ELU(),
                nn.Linear(64, 6),   nn.Tanh(),
            )
            actor_keys = {k.replace("actor.", ""): v
                          for k, v in state_dict.items() if k.startswith("actor.")}
            if actor_keys:
                self._policy.load_state_dict(actor_keys)
            else:
                self._policy.load_state_dict(state_dict)
            self._policy.eval()
            self.get_logger().info(f"정책 로드 완료: {_POLICY_PATH}")
        except Exception as e:
            self.get_logger().error(f"정책 로드 실패: {e}")

    # ── 콜백 ─────────────────────────────────────────────────────────────
    def _on_imu(self, msg: Imu):
        roll, pitch, _ = _quat_to_euler(msg.orientation)
        with self._lock:
            self._roll  = roll
            self._pitch = pitch
            # 넘어짐 감지
            fallen = (abs(math.degrees(roll))  > FALL_THRESHOLD_DEG or
                      abs(math.degrees(pitch)) > FALL_THRESHOLD_DEG)
            if fallen:
                if self._fallen_since is None:
                    self._fallen_since = time.time()
            else:
                self._fallen_since = None

    def _on_joint_states(self, msg: JointState):
        with self._lock:
            for i, name in enumerate(M0609_JOINTS):
                if name in msg.name:
                    idx = msg.name.index(name)
                    if idx < len(msg.position):
                        self._joint_pos[i] = msg.position[idx]
                    if idx < len(msg.velocity):
                        self._joint_vel[i] = msg.velocity[idx]

    # ── 서비스 핸들러 ─────────────────────────────────────────────────────
    def _on_start_srv(self, req, res):
        with self._lock:
            if self._recovering:
                res.success = False
                res.message = "이미 복구 중"
                return res
            self._recovering = True
            self._recovery_start = time.time()
        self.get_logger().info("복구 모드 시작")
        res.success = True
        res.message = "복구 모드 시작됨"
        return res

    def _on_stop_srv(self, req, res):
        with self._lock:
            self._recovering = False
        self.get_logger().info("복구 모드 중지")
        res.success = True
        res.message = "복구 모드 중지됨"
        return res

    # ── 제어 루프 (20 Hz) ─────────────────────────────────────────────────
    def _control_loop(self):
        with self._lock:
            recovering      = self._recovering
            fallen_since    = self._fallen_since
            recovery_start  = self._recovery_start
            roll            = self._roll
            pitch           = self._pitch
            joint_pos       = self._joint_pos.copy()
            joint_vel       = self._joint_vel.copy()

        # 자동 복구 트리거: 2초 이상 넘어진 상태
        if (not recovering and fallen_since is not None and
                time.time() - fallen_since > FALL_SUSTAIN_SEC):
            with self._lock:
                self._recovering     = True
                self._recovery_start = time.time()
            recovering     = True
            recovery_start = time.time()
            self.get_logger().warn("넘어짐 감지 — 자동 복구 시작")

        # 복구 타임아웃
        if recovering and recovery_start and \
                time.time() - recovery_start > RECOVERY_TIMEOUT_SEC:
            with self._lock:
                self._recovering = False
            self._publish_status("TIMEOUT")
            self.get_logger().warn("복구 타임아웃")
            return

        # upright 성공 감지
        if recovering:
            if (abs(math.degrees(roll))  < 15.0 and
                    abs(math.degrees(pitch)) < 15.0):
                with self._lock:
                    self._recovering = False
                self._publish_status("SUCCESS")
                self.get_logger().info("복구 성공!")
                return

        if not recovering:
            self._publish_status("IDLE" if fallen_since is None else "FALLEN")
            return

        # ── 정책 inference ───────────────────────────────────────────────
        self._publish_status("RECOVERING")

        if self._policy is None:
            # 정책 없음 — 홈 자세로 복귀 (fallback)
            self._send_joint_command(np.zeros(6))
            return

        obs = np.array([
            roll, pitch, 0.0,   # yaw 무시
            0.3,                # rover_pos_z 추정
            0.0, 0.0, 0.0,     # lin_vel
            0.0, 0.0, 0.0,     # ang_vel
            *joint_pos,         # m0609 joint_pos (6)
            *joint_vel,         # m0609 joint_vel (6)
        ], dtype=np.float32)

        with torch.no_grad():
            action = self._policy(
                torch.tensor(obs).unsqueeze(0)
            ).squeeze(0).numpy()  # (6,) normalized

        # normalized → 관절 각도
        joint_target = _JOINT_MID + action * _JOINT_SCALE
        self._send_joint_command(joint_target)

    def _send_joint_command(self, positions: np.ndarray):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = M0609_JOINTS
        msg.position = positions.tolist()
        self._joint_pub.publish(msg)

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)


def main():
    rclpy.init()
    node = RecoveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
