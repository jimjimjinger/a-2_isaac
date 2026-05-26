"""Robot arm action server — M0609 manipulation.

pick_mineral 3-mode:
  enable_ik=True + use_world_target_ik=True (default, T2 정공법)
     → mineral world XYZ (action goal) + /ground_truth/odom 으로 arm base local
       변환 → DLS-IK 로 APPROACH/DESCEND/LIFT 자세 풀음 → /grasp/command snap.
       arm 이 mineral 까지 동적으로 접근.

  enable_ik=True + use_world_target_ik=False
     → 기존 wrist-cam servo IK (Phase 3b-3). YOLO detection 대기 + IK.
       wrist 마운트 calibration 필요. cheat 졸업 시 활성.

  enable_ik=False
     → simple-ver scripted PICK_DOWN dip + /grasp/command FixedJoint snap.
       mineral 좌표 무시. supervisor APPROACH align 의존. legacy fallback.

place_to_cargo / unload_to_base / deploy_solar_panel — scripted PLACE_BASKET.
"""
from __future__ import annotations

import math
import threading
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from isaac_interfaces.action import ExecuteArmTask
from isaac_interfaces.msg import Detection, DetectionArray
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .kinematics import M0609_DH, dls_ik, fk


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def _yaw_rot_z(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
HOME_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# Pick down — joint_1=0 (forward toward mineral), shoulder dip + wrist tilt.
# Mineral is in front of rover (supervisor APPROACH P-control aligns it).
PICK_DOWN_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# Cargo (back of rover) — HOME 의 joint_1 만 180° 회전. LIFT → HOME 경유 후
# 단순 yaw swing 으로 무게중심 안정성 ↑ (이전 [180,25,90,0,55,0] 큰 반경 dip 시
# rover 뒤로 기울어짐 우려).
CARGO_DEG = [180.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# Legacy fallback when IK is enabled and wrist detection missing.
PICK_TRAJ_DEG: List[List[float]] = [
    [  0.0,  0.0, 90.0, 0.0, 90.0, 0.0],
    [  0.0, 25.0, 90.0, 0.0, 65.0, 0.0],
    [  0.0,  0.0, 90.0, 0.0, 90.0, 0.0],
]

# rover_yolo_demo.py 포팅 — JS_PRE / JS_POST joint-space dump 시퀀스.
# ATTACH_LIFT 후 cargo (rover 뒤) 로 dump 자세 잡고 release → HOME 복귀.
# 단일 CARGO_DEG swing 보다 부드럽고 dump 자세 명확.
PLACE_TRAJ_PRE_DEG: List[List[float]] = [
    [  0.0,  0.0, 90.0, 0.0, 90.0, 0.0],   # HOME (lift 후 정렬)
    [180.0,  0.0, 90.0, 0.0, 90.0, 0.0],   # joint_1 0 → 180 (베이스 뒤)
    [180.0, 12.5, 90.0, 0.0, 55.0, 0.0],   # 어깨(완만) + 손목 dump
]
PLACE_TRAJ_POST_DEG: List[List[float]] = [
    [180.0,  0.0, 90.0, 0.0, 90.0, 0.0],   # 어깨/손목 복귀
    [  0.0,  0.0, 90.0, 0.0, 90.0, 0.0],   # HOME
]

# Place trajectory alias used by legacy place_to_cargo / unload_to_base /
# deploy_solar_panel scripted fallbacks (referenced by _execute_callback +
# _execute_pick_ik). Prior to this they were left undefined — wiring through
# PLACE_TRAJ_PRE+POST so those code paths don't NameError.
PLACE_BASKET_TRAJ_DEG: List[List[float]] = (
    [list(HOME_DEG)] + PLACE_TRAJ_PRE_DEG + PLACE_TRAJ_POST_DEG
)


def _wrist_optical_to_link6(
        xyz_optical: np.ndarray,
        mount_xyz_link6: Tuple[float, float, float],
        R_optical_to_link6: np.ndarray) -> np.ndarray:
    """Convert mineral XYZ from wrist cam optical frame to link_6 frame."""
    p_link6 = R_optical_to_link6 @ xyz_optical + np.asarray(mount_xyz_link6)
    return p_link6


def _link6_to_base(p_link6: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Point in link_6 frame → arm base frame using current joints."""
    T = fk(q)
    R = T[:3, :3]
    p = T[:3, 3]
    return R @ p_link6 + p


class ArmExecutorNode(Node):
    SUPPORTED_COMMANDS = {
        "pick_mineral",
        "place_to_cargo",
        "unload_to_base",
        "deploy_solar_panel",
    }

    def __init__(self) -> None:
        super().__init__("arm_executor_node")
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("step_duration_sec", 3.0)
        self.declare_parameter("joint_command_topic", "/arm/joint_command")
        self.declare_parameter("joint_state_topic", "/joint_states_raw")
        self.declare_parameter("wrist_detections_topic", "/perception/wrist_detections")
        self.declare_parameter("wrist_detection_wait_sec", 6.0)
        self.declare_parameter("hover_above_mineral_m", 0.05)
        self.declare_parameter("min_confidence", 0.5)
        # wrist-cam optical origin → link_6 origin (link_6 frame), m.
        # vehicle_v3.usd 의 rigid chain (link_6 → angle_bracket → realsense_d455 →
        # RSD455 → Camera_OmniVision_OV9782_Color) 의 fixed transform 을
        # ComputeLocalToWorldTransform 으로 추출 (모든 사이 joint 가 PhysicsFixedJoint).
        self.declare_parameter("wrist_mount_xyz_link6", [0.0115, 0.0450, 0.0500])
        # Rotation: OpenCV optical (x right, y down, z forward) → link_6 axes.
        # 동일 USD 추출에서 link_6 → USD camera body = diag(1,-1,-1), USD body →
        # OpenCV optical = diag(1,-1,-1) 합성 → identity. 즉 optical XYZ 와 link_6
        # XYZ 축이 우연히 정렬됨. 행 우선 9-flat.
        self.declare_parameter("wrist_R_optical_to_link6", [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ])
        self.declare_parameter("ik_use_orientation", False)
        # 3-mode pick_mineral:
        #   enable_ik=True (default) + world_target=True → T2 DLS-IK world target
        #     (mineral world XYZ → arm base local → IK → APPROACH/DESCEND/LIFT)
        #   enable_ik=True + world_target=False → wrist-cam servo IK (Phase 3b-3)
        #   enable_ik=False → simple-ver scripted PICK_DOWN (legacy fallback)
        self.declare_parameter("enable_ik", True)
        self.declare_parameter("use_world_target_ik", True)
        self.declare_parameter("odom_topic", "/ground_truth/odom")
        # IK phase 높이 (arm base local z offset, mineral world 좌표 기준 상대)
        self.declare_parameter("ik_approach_dz", 0.30)
        self.declare_parameter("ik_descend_dz", 0.05)
        self.declare_parameter("grasp_command_topic", "/grasp/command")
        # Step delay between grasp publish and arm continuing — gives
        # vehicle_v3 ScriptNode time to attach the FixedJoint.
        self.declare_parameter("grasp_publish_delay_sec", 0.3)

        # rover_yolo_demo.py 포팅 모드. "world_target" (default, 기존 7-state) /
        # "rover_yolo_demo" (9-state: HOME_PRE → WRIST_SERVO → APPROACH_DESCEND →
        # GRASP_CLOSE → ATTACH_LIFT → JS_PRE → RELEASE → JS_POST → DONE) /
        # "scripted" / "wrist_servo". 미지정 시 enable_ik + use_world_target_ik 로 fallback.
        self.declare_parameter("pick_style", "")
        # rover_yolo_demo 의 HOVER_ABOVE_MINERAL (TCP 가 mineral 위 4cm).
        self.declare_parameter("hover_above_mineral_z_m", 0.04)
        # rover_yolo_demo 의 LIFT_HEIGHT (mineral 위 45cm 까지 들어올림).
        self.declare_parameter("lift_height_m", 0.45)
        # 시연 시각화용 APPROACH phase 높이 (mineral 위 N m hover). 0 이면 phase
        # skip 하고 곧바로 DESCEND. default 0.20 → 위에서 접근 → 내려가기 단계 가시화.
        self.declare_parameter("approach_above_mineral_z_m", 0.20)
        # GRASP_CLOSE 후 ATTACH_LIFT 전 dwell (sec). vehicle_v3 의 FixedJoint snap
        # 이 instant 라서 dwell 없으면 grasp 순간이 안 보임. default 1.5s.
        self.declare_parameter("grasp_dwell_sec", 1.5)
        # WRIST_SERVO 단계 wrist detection 대기 시간 (sec). 짧게 (1초) — 못 잡으면 nav XYZ 사용.
        self.declare_parameter("wrist_servo_timeout_sec", 1.0)
        # WRIST_SERVO 의 XY 보정 활성화. 주의: yolo_perception_node 가 wrist det 을
        # optical frame 으로 publish ("nav: world, wrist: optical") → world frame 변환은
        # wrist_mount_xyz_link6 / wrist_R_optical_to_link6 calibration 정확도에 의존.
        # default False — nav XYZ (이미 world) 그대로 사용 (안전). calibration 검증
        # 후 true 로 ros2 param set 권장.
        self.declare_parameter("wrist_servo_apply_xy", False)

        self.joint_pub = self.create_publisher(
            JointState, str(self.get_parameter("joint_command_topic").value), 10)
        # /grasp/command — published as geometry_msgs/Twist (OmniGraph
        # constraint: no generic-string subscriber in isaacsim.ros2.bridge).
        # Encoding:
        #   pickup x y z target_id   → linear=(x,y,z) angular.x=+1.0
        #   release                  → linear=(0,0,0) angular.x=-1.0
        # vehicle_v3 ScriptNode decodes the sign of angular.x.
        self.grasp_pub = self.create_publisher(
            Twist, str(self.get_parameter("grasp_command_topic").value), 10)

        self.create_subscription(
            DetectionArray, str(self.get_parameter("wrist_detections_topic").value),
            self._on_wrist_detections, SENSOR_QOS)

        self.create_subscription(
            JointState, str(self.get_parameter("joint_state_topic").value),
            self._on_joint_states, SENSOR_QOS)

        # rover (= arm base via /ground_truth/odom chassisFrameId="base_link")
        # world pose — world target IK 에서 mineral world → arm base local 변환에 필요.
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self._on_odom, SENSOR_QOS)

        self._wrist_lock = threading.Lock()
        self._wrist_dets: List[Detection] = []
        self._wrist_stamp_ns: int = 0
        # Current arm joint positions (rad) from /joint_states_raw. None until
        # first message arrives — we fall back to HOME_DEG for IK base then.
        self._current_arm_q_rad: Optional[np.ndarray] = None
        self._joint_state_logged_once = False

        self._odom_lock = threading.Lock()
        self._rover_xyz: Optional[np.ndarray] = None
        self._rover_yaw: Optional[float] = None

        # Action name relative (no leading /) → ROS2 namespace 자동 prefix.
        # 단일 rover: /execute_arm_task. 다중 rover: /rover_1/execute_arm_task 등.
        self.action_server = ActionServer(
            self,
            ExecuteArmTask,
            "execute_arm_task",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self._initial_home_sent = False
        self.create_timer(1.0, self._send_initial_home_once)

        self.get_logger().info(
            "arm_executor_node ready: /execute_arm_task (wrist-cam visual servo + DLS-IK)")

    def _send_initial_home_once(self) -> None:
        if self._initial_home_sent:
            return
        self._publish_joint_state(HOME_DEG)
        self._initial_home_sent = True

    def _on_wrist_detections(self, msg: DetectionArray) -> None:
        with self._wrist_lock:
            self._wrist_dets = list(msg.detections)
            self._wrist_stamp_ns = self.get_clock().now().nanoseconds

    def _on_joint_states(self, msg: JointState) -> None:
        """Extract M0609 6 arm joints from rover's full joint state."""
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            q = np.array([name_to_pos[n] for n in JOINT_NAMES], dtype=np.float64)
        except KeyError as e:
            if not self._joint_state_logged_once:
                self.get_logger().warn(
                    f"joint_states_raw missing arm joint {e} — IK will use HOME as base. "
                    f"Available joints: {list(msg.name)}")
                self._joint_state_logged_once = True
            return
        self._current_arm_q_rad = q
        if not self._joint_state_logged_once:
            self.get_logger().info(
                f"joint_states_raw arm joints OK: "
                f"q_deg = [{','.join(f'{math.degrees(v):.1f}' for v in q)}]")
            self._joint_state_logged_once = True

    def _on_odom(self, msg: Odometry) -> None:
        """Rover (articulation root = arm base) world pose 추출.
        chassisFrameId='base_link' = vehicle_v3 의 m0609/base_link → arm base.
        """
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        with self._odom_lock:
            self._rover_xyz = np.array([p.x, p.y, p.z], dtype=np.float64)
            self._rover_yaw = _quat_to_yaw(o.x, o.y, o.z, o.w)

    def _best_wrist_detection(self,
                              wait_sec: float,
                              min_conf: float) -> Optional[Detection]:
        deadline_ns = self.get_clock().now().nanoseconds + int(wait_sec * 1e9)
        while self.get_clock().now().nanoseconds < deadline_ns:
            with self._wrist_lock:
                dets = [d for d in self._wrist_dets if d.confidence >= min_conf]
                stamp = self._wrist_stamp_ns
            if dets:
                # freshness check: must be within last 0.5s
                now_ns = self.get_clock().now().nanoseconds
                if now_ns - stamp < int(0.5e9):
                    return max(dets, key=lambda d: d.confidence)
            time.sleep(0.05)
        return None

    def _goal_callback(self, goal_request: ExecuteArmTask.Goal) -> GoalResponse:
        command = goal_request.command.strip()
        if command not in self.SUPPORTED_COMMANDS:
            self.get_logger().warning(f"Rejecting unsupported arm command: {command}")
            return GoalResponse.REJECT
        self.get_logger().info(
            f"Accepted arm goal command={command} target_id={goal_request.target_id} "
            f"xyz=({goal_request.target_x:.2f},{goal_request.target_y:.2f},"
            f"{goal_request.target_z:.2f})")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Arm goal cancel requested")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        command = goal_handle.request.command.strip()
        self.get_logger().info(f"Executing arm command: {command}")

        if command == "pick_mineral":
            style = str(self.get_parameter("pick_style").value).strip().lower()
            if style == "rover_yolo_demo":
                return self._execute_pick_rover_yolo_demo(goal_handle)
            if style == "scripted":
                return self._execute_pick_scripted(goal_handle)
            if style == "wrist_servo":
                return self._execute_pick_ik(goal_handle)
            if style == "world_target":
                return self._execute_pick_world_target(goal_handle)
            # pick_style 미지정 — 기존 enable_ik / use_world_target_ik 로 fallback.
            if bool(self.get_parameter("enable_ik").value):
                if bool(self.get_parameter("use_world_target_ik").value):
                    return self._execute_pick_world_target(goal_handle)
                return self._execute_pick_ik(goal_handle)
            return self._execute_pick_scripted(goal_handle)
        # Other commands (place_to_cargo, unload_to_base, deploy_solar_panel)
        # — fall back to scripted PLACE_BASKET sequence as visual demo.
        return self._execute_scripted(goal_handle, PLACE_BASKET_TRAJ_DEG, command)

    def _publish_grasp(self, cmd: str, x: float = 0.0, y: float = 0.0,
                       z: float = 0.0, target_id: str = "") -> None:
        """Publish grasp command as Twist (hijacked for OmniGraph compatibility)."""
        msg = Twist()
        if cmd == "pickup":
            msg.linear.x = float(x)
            msg.linear.y = float(y)
            msg.linear.z = float(z)
            msg.angular.x = 1.0    # pickup mode marker
        else:
            msg.angular.x = -1.0   # release mode marker
        # Publish a few times so OmniGraph subscriber is sure to capture it
        # (single-shot can be missed if next tick coincides with msg arrival).
        for _ in range(3):
            self.grasp_pub.publish(msg)
            time.sleep(0.05)
        delay = float(self.get_parameter("grasp_publish_delay_sec").value)
        if delay > 0.0:
            time.sleep(delay)
        self.get_logger().info(
            f"grasp/command -> {cmd} xyz=({x:.3f},{y:.3f},{z:.3f}) "
            f"target_id={target_id}")

    def _execute_pick_scripted(self, goal_handle):
        """Simple-ver pick — single forward dip + single cargo swing:
            HOME -> PICK_DOWN (forward, joint_1=0) -> GRASP publish -> HOME
                 -> CARGO (joint_1=180) -> RELEASE publish -> HOME
        """
        req = goal_handle.request
        tx, ty, tz = float(req.target_x), float(req.target_y), float(req.target_z)

        cur_q_rad = (self._current_arm_q_rad
                     if self._current_arm_q_rad is not None
                     else np.radians(HOME_DEG))
        cur_deg = list(np.degrees(cur_q_rad))

        # 1) HOME
        self._goto(cur_deg, HOME_DEG, goal_handle, "HOME_PRE", 0.0, 0.1)
        cur_deg = list(HOME_DEG)

        # 2) PICK_DOWN — forward dip (joint_1=0, mineral in front of rover)
        self._goto(cur_deg, PICK_DOWN_DEG, goal_handle, "PICK_DOWN", 0.1, 0.3)
        cur_deg = list(PICK_DOWN_DEG)

        # 3) GRASP — vehicle_v3 attaches nearest mineral to gripper
        self._publish_grasp("pickup", x=tx, y=ty, z=tz, target_id=req.target_id)
        fb = ExecuteArmTask.Feedback()
        fb.state = "GRASP_CLOSE"
        fb.progress = 0.35
        fb.message = "pickup published"
        goal_handle.publish_feedback(fb)

        # 4) LIFT — back to HOME (mineral comes along via FixedJoint)
        self._goto(cur_deg, HOME_DEG, goal_handle, "LIFT", 0.35, 0.55)
        cur_deg = list(HOME_DEG)

        # 5) CARGO — single swing to back of rover
        self._goto(cur_deg, CARGO_DEG, goal_handle, "CARGO_SWING", 0.55, 0.8)
        cur_deg = list(CARGO_DEG)

        # 6) RELEASE — vehicle_v3 detaches + hides the mineral (cargo stowed)
        self._publish_grasp("release")
        fb = ExecuteArmTask.Feedback()
        fb.state = "RELEASE"
        fb.progress = 0.85
        fb.message = "release published"
        goal_handle.publish_feedback(fb)

        # 7) HOME guarantee
        self._goto(cur_deg, HOME_DEG, goal_handle, "HOME_POST", 0.85, 1.0)
        self._publish_joint_state(HOME_DEG)
        goal_handle.succeed()
        self.get_logger().info("pick_mineral (scripted) done")
        return ExecuteArmTask.Result(success=True, message="pick_mineral completed")

    def _execute_pick_world_target(self, goal_handle):
        """T2 DLS-IK + /grasp/command FixedJoint snap (정공법 + grip cheat).

        Action goal 의 target_xyz 를 mineral world XYZ 로 받아:
          1. /ground_truth/odom 도착 대기 (arm base world pose)
          2. mineral world → arm base local 변환 (yaw 보정)
          3. APPROACH / DESCEND / LIFT 의 target 을 mineral 기준 z offset 으로 IK
          4. State machine: HOME → APPROACH → DESCEND → grasp pickup → LIFT
             → CARGO → grasp release → HOME
        """
        req = goal_handle.request
        mineral_world = np.array(
            [req.target_x, req.target_y, req.target_z], dtype=np.float64)

        # 1) /ground_truth/odom 도착 대기 (최대 2초)
        deadline_ns = self.get_clock().now().nanoseconds + int(2e9)
        while self.get_clock().now().nanoseconds < deadline_ns:
            with self._odom_lock:
                if self._rover_xyz is not None:
                    break
            time.sleep(0.05)
        with self._odom_lock:
            arm_base_xyz = (None if self._rover_xyz is None
                            else self._rover_xyz.copy())
            yaw = self._rover_yaw
        if arm_base_xyz is None or yaw is None:
            self.get_logger().error(
                "world_target_ik: no /ground_truth/odom — abort")
            goal_handle.abort()
            return ExecuteArmTask.Result(success=False, message="no odom")

        # 2) mineral world → arm base local (yaw 보정)
        R = _yaw_rot_z(yaw)
        target_local = R.T @ (mineral_world - arm_base_xyz)
        self.get_logger().info(
            f"mineral world=({mineral_world[0]:.2f},{mineral_world[1]:.2f},"
            f"{mineral_world[2]:.2f}) arm_base world=({arm_base_xyz[0]:.2f},"
            f"{arm_base_xyz[1]:.2f},{arm_base_xyz[2]:.2f}) yaw={math.degrees(yaw):.1f}° "
            f"→ target_local=({target_local[0]:.3f},{target_local[1]:.3f},"
            f"{target_local[2]:.3f})")

        # 3) IK target — APPROACH (위), DESCEND (mineral 옆)
        dz_a = float(self.get_parameter("ik_approach_dz").value)
        dz_d = float(self.get_parameter("ik_descend_dz").value)
        t_approach = target_local + np.array([0.0, 0.0, dz_a])
        t_descend = target_local + np.array([0.0, 0.0, dz_d])

        q_home = np.array([math.radians(v) for v in HOME_DEG])
        q_approach, ok_a, err_a = dls_ik(t_approach, None, q_home,
                                          position_only=True)
        q_descend, ok_d, err_d = dls_ik(t_descend, None, q_approach,
                                         position_only=True)
        if not (ok_a and ok_d):
            self.get_logger().error(
                f"IK 미수렴: APPROACH={ok_a}(err={err_a:.4f}) "
                f"DESCEND={ok_d}(err={err_d:.4f})")
            goal_handle.abort()
            return ExecuteArmTask.Result(
                success=False,
                message=f"IK fail: a={ok_a} d={ok_d}")

        deg_approach = list(np.degrees(q_approach))
        deg_descend = list(np.degrees(q_descend))
        self.get_logger().info(
            f"IK solved: APPROACH q_deg={[round(v,1) for v in deg_approach]} "
            f"DESCEND q_deg={[round(v,1) for v in deg_descend]}")

        # 4) State machine — LIFT 제거 (DESCEND → HOME_MID 직접 보간으로 통합):
        #   HOME → APPROACH → DESCEND → pickup → HOME_MID → CARGO → release → HOME
        cur_deg = list(HOME_DEG)
        self._goto(cur_deg, HOME_DEG, goal_handle, "HOME_PRE", 0.0, 0.05)
        cur_deg = list(HOME_DEG)

        self._goto(cur_deg, deg_approach, goal_handle, "APPROACH", 0.05, 0.25)
        cur_deg = list(deg_approach)

        self._goto(cur_deg, deg_descend, goal_handle, "DESCEND", 0.25, 0.45)
        cur_deg = list(deg_descend)

        # 5) Grasp pickup — FixedJoint snap (mineral → gripper link origin)
        self._publish_grasp("pickup",
                            x=float(mineral_world[0]),
                            y=float(mineral_world[1]),
                            z=float(mineral_world[2]),
                            target_id=req.target_id)
        fb = ExecuteArmTask.Feedback()
        fb.state = "GRASP_PICKUP"
        fb.progress = 0.5
        fb.message = "pickup published"
        goal_handle.publish_feedback(fb)

        # 6) DESCEND → HOME 직접 보간 (LIFT 통합). mineral 은 collision off 라
        # rover body 통과해도 충돌 영향 X. arm 자세 dip → 직립 자연 펴짐.
        self._goto(cur_deg, HOME_DEG, goal_handle, "HOME_MID", 0.5, 0.7)
        cur_deg = list(HOME_DEG)

        # 7) CARGO yaw swing (HOME 의 joint_1 만 180°)
        self._goto(cur_deg, CARGO_DEG, goal_handle, "CARGO_SWING", 0.7, 0.85)
        cur_deg = list(CARGO_DEG)

        # 8) Release — detach + MakeInvisible
        self._publish_grasp("release")
        fb = ExecuteArmTask.Feedback()
        fb.state = "RELEASE"
        fb.progress = 0.9
        fb.message = "release published"
        goal_handle.publish_feedback(fb)

        # 9) HOME
        self._goto(cur_deg, HOME_DEG, goal_handle, "HOME_POST", 0.9, 1.0)
        self._publish_joint_state(HOME_DEG)
        goal_handle.succeed()
        self.get_logger().info("pick_mineral (T2 DLS-IK world target) done")
        return ExecuteArmTask.Result(
            success=True, message="pick_mineral completed (IK)")

    def _execute_pick_rover_yolo_demo(self, goal_handle):
        """rover_yolo_demo.py 의 PickPlaceStateMachine 행동 포팅 — 9 phase.

        HOME_PRE → WRIST_SERVO (XY 보정) → APPROACH_DESCEND (TCP 가 mineral 위 hover)
          → GRASP_CLOSE (/grasp/command pickup) → ATTACH_LIFT (mineral 위 lift_height)
          → JS_PRE (PLACE_TRAJ_PRE_DEG 3-step dump) → RELEASE (/grasp/command release)
          → JS_POST (PLACE_TRAJ_POST_DEG 2-step return) → DONE

        호환성 차이 (rover_yolo_demo standalone 대비):
          - IK: Isaac Sim Jacobian 대신 자체 kinematics.dls_ik (arm base local).
          - Grip: USD FixedJoint 대신 /grasp/command Twist (vehicle_v3 ScriptNode 수신).
          - Gripper joint drive: 생략 (vehicle_v3 가 grip cmd 받아서 FixedJoint snap).
          - Wrist servo: 직접 cam 읽기 대신 /perception/wrist_detections 토픽 사용.
          - TCP offset 보정: 단순화 — link_6 기준 IK 에 hover Z offset 만 더함.
        """
        req = goal_handle.request
        mineral_world = np.array(
            [req.target_x, req.target_y, req.target_z], dtype=np.float64)

        # 1) /ground_truth/odom 도착 대기 (최대 2초) — arm base world pose
        deadline_ns = self.get_clock().now().nanoseconds + int(2e9)
        while self.get_clock().now().nanoseconds < deadline_ns:
            with self._odom_lock:
                if self._rover_xyz is not None:
                    break
            time.sleep(0.05)
        with self._odom_lock:
            arm_base_xyz = (None if self._rover_xyz is None
                            else self._rover_xyz.copy())
            yaw = self._rover_yaw
        if arm_base_xyz is None or yaw is None:
            self.get_logger().error(
                "rover_yolo_demo: no /ground_truth/odom — abort")
            goal_handle.abort()
            return ExecuteArmTask.Result(success=False, message="no odom")

        hover = float(self.get_parameter("hover_above_mineral_z_m").value)
        lift_h = float(self.get_parameter("lift_height_m").value)
        approach_z = float(
            self.get_parameter("approach_above_mineral_z_m").value)
        grasp_dwell = float(self.get_parameter("grasp_dwell_sec").value)
        wrist_wait = float(
            self.get_parameter("wrist_servo_timeout_sec").value)
        min_conf = float(self.get_parameter("min_confidence").value)

        cur_q_rad = (self._current_arm_q_rad
                     if self._current_arm_q_rad is not None
                     else np.radians(HOME_DEG))
        cur_deg = list(np.degrees(cur_q_rad))

        # ── Phase 1: HOME_PRE ─────────────────────────────────────────
        self._goto(cur_deg, HOME_DEG, goal_handle, "HOME_PRE", 0.0, 0.05)
        cur_deg = list(HOME_DEG)

        # ── Phase 2: WRIST_SERVO — wrist det 짧게 대기 (선택적 XY 보정) ─
        # yolo_perception_node 는 wrist det 을 optical frame 으로 publish 함.
        # default 로는 nav XYZ (이미 world) 신뢰 → wrist det 도착 시점만 잠깐 대기.
        # wrist_servo_apply_xy=True 시 optical → link_6 → arm base → world 변환 적용.
        det = self._best_wrist_detection(wrist_wait, min_conf)
        apply_wrist = bool(self.get_parameter("wrist_servo_apply_xy").value)
        if det is not None and apply_wrist:
            xyz_optical = np.array(
                [det.world_position.x, det.world_position.y,
                 det.world_position.z], dtype=np.float64)
            mount_xyz = tuple(float(v) for v in
                              self.get_parameter("wrist_mount_xyz_link6").value)
            R_flat = list(
                self.get_parameter("wrist_R_optical_to_link6").value)
            R_oc = np.array(R_flat, dtype=np.float64).reshape(3, 3)
            p_link6 = _wrist_optical_to_link6(xyz_optical, mount_xyz, R_oc)
            q_cur = (self._current_arm_q_rad
                     if self._current_arm_q_rad is not None
                     else np.radians(HOME_DEG))
            p_arm_base = _link6_to_base(p_link6, q_cur)
            p_world_refined = arm_base_xyz + _yaw_rot_z(yaw) @ p_arm_base
            old_xy = mineral_world[:2].copy()
            mineral_world[0] = float(p_world_refined[0])
            mineral_world[1] = float(p_world_refined[1])
            self.get_logger().info(
                f"WRIST_SERVO refined XY (world) {old_xy.round(3).tolist()} → "
                f"{mineral_world[:2].round(3).tolist()} "
                f"(class={det.class_name} conf={det.confidence:.2f})")
        elif det is not None:
            self.get_logger().info(
                f"WRIST_SERVO det observed (class={det.class_name} "
                f"conf={det.confidence:.2f}) — XY refinement 비활성 "
                f"(wrist_servo_apply_xy=False), nav XYZ 사용")
        else:
            self.get_logger().info(
                f"WRIST_SERVO no det in {wrist_wait:.1f}s — nav XYZ 사용")
        fb = ExecuteArmTask.Feedback()
        fb.state = "WRIST_SERVO"
        fb.progress = 0.1
        fb.message = "wrist refine done"
        goal_handle.publish_feedback(fb)

        # mineral world → arm base local (yaw 보정)
        R = _yaw_rot_z(yaw)
        target_local = R.T @ (mineral_world - arm_base_xyz)

        # ── Phase 3a: APPROACH — TCP 가 mineral 위 approach_z (높은 hover) ──
        # 시각화용 — 위에서 접근 → 내려가기 두 단계 명확히 보이게.
        # approach_z<=0 이면 skip 하고 곧바로 DESCEND.
        q_home = np.array([math.radians(v) for v in HOME_DEG])
        q_seed = q_home
        if approach_z > 1e-3:
            t_approach = target_local + np.array([0.0, 0.0, approach_z])
            q_approach, ok_a, err_a = dls_ik(
                t_approach, None, q_seed, position_only=True)
            if not ok_a:
                self.get_logger().warn(
                    f"APPROACH IK 마진 err={err_a:.4f} → DESCEND 로 점프")
            else:
                deg_approach = list(np.degrees(q_approach))
                self.get_logger().info(
                    f"APPROACH target_local=({t_approach[0]:.3f},"
                    f"{t_approach[1]:.3f},{t_approach[2]:.3f}) "
                    f"q_deg={[round(v,1) for v in deg_approach]}")
                self._goto(cur_deg, deg_approach, goal_handle, "APPROACH",
                           0.1, 0.25)
                cur_deg = list(deg_approach)
                q_seed = q_approach

        # ── Phase 3b: DESCEND — TCP 가 mineral 위 hover (4cm) ────────
        t_descend = target_local + np.array([0.0, 0.0, hover])
        q_descend, ok_d, err_d = dls_ik(
            t_descend, None, q_seed, position_only=True)
        if not ok_d:
            self.get_logger().error(
                f"DESCEND IK 미수렴 err={err_d:.4f} → abort")
            goal_handle.abort()
            return ExecuteArmTask.Result(
                success=False, message=f"IK fail descend err={err_d:.4f}")
        deg_descend = list(np.degrees(q_descend))
        self.get_logger().info(
            f"DESCEND target_local="
            f"({t_descend[0]:.3f},{t_descend[1]:.3f},{t_descend[2]:.3f}) "
            f"q_deg={[round(v,1) for v in deg_descend]}")
        self._goto(cur_deg, deg_descend, goal_handle, "DESCEND", 0.25, 0.4)
        cur_deg = list(deg_descend)

        # ── Phase 4: GRASP_CLOSE — /grasp/command pickup + dwell ─────
        # grasp_dwell 동안 정지하여 FixedJoint snap 이 시각적으로 보이게.
        self._publish_grasp("pickup",
                            x=float(mineral_world[0]),
                            y=float(mineral_world[1]),
                            z=float(mineral_world[2]),
                            target_id=req.target_id)
        fb = ExecuteArmTask.Feedback()
        fb.state = "GRASP_CLOSE"
        fb.progress = 0.42
        fb.message = "pickup published"
        goal_handle.publish_feedback(fb)
        if grasp_dwell > 0.0:
            self.get_logger().info(
                f"GRASP_CLOSE dwell {grasp_dwell:.2f}s — FixedJoint snap 시각 확인 시간")
            # dwell 동안 현재 joint pose 유지 publish (정지 표시 + 안정)
            n_dwell_pub = max(1, int(grasp_dwell * 5.0))  # 5Hz
            for _ in range(n_dwell_pub):
                self._publish_joint_state(cur_deg)
                time.sleep(grasp_dwell / n_dwell_pub)

        # ── Phase 5: ATTACH_LIFT — TCP 를 mineral 위 lift_height 로 ──
        t_lift = target_local + np.array([0.0, 0.0, lift_h])
        q_lift, ok_l, err_l = dls_ik(
            t_lift, None, q_descend, position_only=True)
        if not ok_l:
            self.get_logger().warn(
                f"ATTACH_LIFT IK 마진 err={err_l:.4f} — best effort 진행")
        deg_lift = list(np.degrees(q_lift))
        self.get_logger().info(
            f"ATTACH_LIFT q_deg={[round(v,1) for v in deg_lift]}")
        self._goto(cur_deg, deg_lift, goal_handle, "ATTACH_LIFT", 0.4, 0.55)
        cur_deg = list(deg_lift)

        # ── Phase 6: JS_PRE — PLACE_TRAJ_PRE_DEG 3 waypoints ─────────
        n_pre = max(1, len(PLACE_TRAJ_PRE_DEG))
        seg_pre = (0.75 - 0.55) / n_pre
        for i, wp in enumerate(PLACE_TRAJ_PRE_DEG):
            seg_start = 0.55 + i * seg_pre
            seg_end = seg_start + seg_pre
            self.get_logger().info(
                f"JS_PRE_{i + 1}/{n_pre} → wp_deg={wp}")
            self._goto(cur_deg, wp, goal_handle, f"JS_PRE_{i + 1}",
                       seg_start, seg_end)
            cur_deg = list(wp)

        # ── Phase 7: RELEASE — /grasp/command release ────────────────
        self._publish_grasp("release")
        fb = ExecuteArmTask.Feedback()
        fb.state = "RELEASE"
        fb.progress = 0.8
        fb.message = "release published"
        goal_handle.publish_feedback(fb)

        # ── Phase 8: JS_POST — PLACE_TRAJ_POST_DEG 2 waypoints ───────
        n_post = max(1, len(PLACE_TRAJ_POST_DEG))
        seg_post = (1.0 - 0.8) / n_post
        for i, wp in enumerate(PLACE_TRAJ_POST_DEG):
            seg_start = 0.8 + i * seg_post
            seg_end = seg_start + seg_post
            self.get_logger().info(
                f"JS_POST_{i + 1}/{n_post} → wp_deg={wp}")
            self._goto(cur_deg, wp, goal_handle, f"JS_POST_{i + 1}",
                       seg_start, seg_end)
            cur_deg = list(wp)

        # ── Phase 9: DONE — HOME guarantee ────────────────────────────
        self._publish_joint_state(HOME_DEG)
        goal_handle.succeed()
        self.get_logger().info("pick_mineral (rover_yolo_demo 9-state) done")
        return ExecuteArmTask.Result(
            success=True,
            message="pick_mineral completed (rover_yolo_demo)")

    def _execute_pick_ik(self, goal_handle):
        """pick_mineral with IK + wrist-cam servo (Phase 3b-3, off by default).

        Tune wrist_mount_xyz_link6 / wrist_R_optical_to_link6 before enabling.
        """
        # 1) Ensure HOME first (uses current joint state if available)
        cur_q_rad = (self._current_arm_q_rad
                     if self._current_arm_q_rad is not None
                     else np.radians(HOME_DEG))
        cur_deg = list(np.degrees(cur_q_rad))
        self._goto(cur_deg, HOME_DEG, goal_handle, "HOME_PRE", 0.0, 0.1)
        cur_deg = list(HOME_DEG)

        # 2) Wait for wrist detection
        wait_s = float(self.get_parameter("wrist_detection_wait_sec").value)
        min_conf = float(self.get_parameter("min_confidence").value)
        det = self._best_wrist_detection(wait_s, min_conf)
        if det is None:
            self.get_logger().warn(
                f"No wrist detection within {wait_s}s — falling back to scripted PICK_TRAJ")
            return self._execute_scripted(goal_handle, PICK_TRAJ_DEG, "pick_mineral_fallback")

        # 3) Convert wrist optical XYZ -> link_6 -> arm base; subtract hover
        xyz_optical = np.array([det.world_position.x, det.world_position.y,
                                det.world_position.z], dtype=np.float64)
        mount_xyz = tuple(float(v) for v in self.get_parameter("wrist_mount_xyz_link6").value)
        R_flat = list(self.get_parameter("wrist_R_optical_to_link6").value)
        R_oc = np.array(R_flat, dtype=np.float64).reshape(3, 3)
        p_link6 = _wrist_optical_to_link6(xyz_optical, mount_xyz, R_oc)
        # Use actual current arm joints (post-HOME) for FK transform base.
        q_cur_rad = (self._current_arm_q_rad
                     if self._current_arm_q_rad is not None
                     else np.radians(HOME_DEG))
        p_base_mineral = _link6_to_base(p_link6, q_cur_rad)

        hover = float(self.get_parameter("hover_above_mineral_m").value)
        target_pos = p_base_mineral + np.array([0.0, 0.0, hover])
        self.get_logger().info(
            f"wrist det class={det.class_name} optical=({xyz_optical[0]:.3f},"
            f"{xyz_optical[1]:.3f},{xyz_optical[2]:.3f}) -> base_mineral="
            f"({p_base_mineral[0]:.3f},{p_base_mineral[1]:.3f},{p_base_mineral[2]:.3f}) "
            f"-> target_hover=({target_pos[0]:.3f},{target_pos[1]:.3f},{target_pos[2]:.3f})")

        # 4) DLS-IK
        use_orient = bool(self.get_parameter("ik_use_orientation").value)
        target_R = fk(q_cur_rad)[:3, :3] if use_orient else None
        q_sol, ok, err = dls_ik(target_pos, target_R, q_cur_rad,
                                position_only=not use_orient)
        if not ok:
            self.get_logger().warn(
                f"DLS-IK did not converge (err={err:.4f}) — falling back to scripted PICK_TRAJ")
            return self._execute_scripted(goal_handle, PICK_TRAJ_DEG, "pick_mineral_fallback")

        target_deg = list(np.degrees(q_sol))
        self.get_logger().info(
            f"IK ok err={err:.5f} target_joints_deg = "
            f"[{','.join(f'{v:.1f}' for v in target_deg)}]")

        # 5) Interpolate to IK target (APPROACH_DESCEND)
        self._goto(cur_deg, target_deg, goal_handle, "APPROACH_DESCEND", 0.1, 0.5)
        cur_deg = target_deg

        # 6) GRASP (no real gripper yet — placeholder)
        fb = ExecuteArmTask.Feedback()
        fb.state = "GRASP_CLOSE"
        fb.progress = 0.55
        fb.message = "gripper close (placeholder)"
        goal_handle.publish_feedback(fb)
        time.sleep(0.5)

        # 7) LIFT (return to HOME briefly)
        self._goto(cur_deg, HOME_DEG, goal_handle, "LIFT", 0.6, 0.75)
        cur_deg = list(HOME_DEG)

        # 8) PLACE_BASKET scripted
        for i in range(len(PLACE_BASKET_TRAJ_DEG) - 1):
            self._goto(cur_deg, PLACE_BASKET_TRAJ_DEG[i + 1], goal_handle,
                       f"PLACE_BASKET_{i + 1}",
                       0.75 + i * 0.05, 0.75 + (i + 1) * 0.05)
            cur_deg = list(PLACE_BASKET_TRAJ_DEG[i + 1])

        # 9) HOME guarantee
        self._publish_joint_state(HOME_DEG)
        goal_handle.succeed()
        self.get_logger().info("pick_mineral done")
        return ExecuteArmTask.Result(success=True, message="pick_mineral completed")

    def _execute_scripted(self, goal_handle, trajectory: List[List[float]], tag: str):
        publish_hz = max(5.0, float(self.get_parameter("publish_hz").value))
        step_dur = max(0.2, float(self.get_parameter("step_duration_sec").value))
        n_interp = max(2, int(round(publish_hz * step_dur)))
        dt = 1.0 / publish_hz
        total_steps = max(1, len(trajectory) - 1)

        cur = list(trajectory[0])
        for seg_idx in range(total_steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._publish_joint_state(HOME_DEG)
                return ExecuteArmTask.Result(success=False, message="canceled")
            target = trajectory[seg_idx + 1]
            for k in range(1, n_interp + 1):
                t = k / n_interp
                interp = [cur[i] + (target[i] - cur[i]) * t for i in range(6)]
                self._publish_joint_state(interp)
                fb = ExecuteArmTask.Feedback()
                fb.state = tag
                fb.progress = float((seg_idx + t) / total_steps)
                fb.message = f"seg {seg_idx + 1}/{total_steps}"
                goal_handle.publish_feedback(fb)
                time.sleep(dt)
            cur = list(target)

        self._publish_joint_state(HOME_DEG)
        goal_handle.succeed()
        self.get_logger().info(f"Arm command {tag} done")
        return ExecuteArmTask.Result(success=True, message=f"{tag} completed")

    def _goto(self, cur_deg: List[float], target_deg: List[float], goal_handle,
              tag: str, progress_start: float, progress_end: float) -> None:
        publish_hz = max(5.0, float(self.get_parameter("publish_hz").value))
        step_dur = max(0.2, float(self.get_parameter("step_duration_sec").value))
        n_interp = max(2, int(round(publish_hz * step_dur)))
        dt = 1.0 / publish_hz
        for k in range(1, n_interp + 1):
            t = k / n_interp
            interp = [cur_deg[i] + (target_deg[i] - cur_deg[i]) * t for i in range(6)]
            self._publish_joint_state(interp)
            fb = ExecuteArmTask.Feedback()
            fb.state = tag
            fb.progress = float(progress_start + (progress_end - progress_start) * t)
            fb.message = tag
            goal_handle.publish_feedback(fb)
            time.sleep(dt)

    def _publish_joint_state(self, positions_deg: List[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [math.radians(v) for v in positions_deg]
        self.joint_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ArmExecutorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
