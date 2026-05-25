"""Robot arm action server — M0609 wrist-cam visual servo + DLS-IK (Phase 3b-3).

Pipeline for pick_mineral:
  HOME -> wait for wrist DetectionArray (mineral pixel + optical XYZ)
       -> WRIST_T_LINK6 transform to arm base frame
       -> DLS-IK to (mineral - hover_offset) target
       -> interpolate joint_command -> APPROACH_DESCEND
       -> GRASP_CLOSE (gripper finger joint)
       -> LIFT (TCP up)
       -> PLACE_BASKET (back-of-rover scripted joints)
       -> HOME

Phase 3a fallback: if wrist detection unavailable, falls back to PICK_TRAJ_DEG
script for visual demo only (no real grasp).

⚠️ WRIST_T_LINK6 is best-effort (vehicle_v3 USD geometry not dumped yet).
   Cross-validate by reading /joint_states_raw + USD prim transforms and tune
   the constants below. All four are exposed as parameters for runtime tune.
"""
from __future__ import annotations

import math
import threading
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from isaac_interfaces.action import ExecuteArmTask
from isaac_interfaces.msg import Detection, DetectionArray
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .kinematics import M0609_DH, dls_ik, fk


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
PICK_DOWN_DEG = [0.0, 25.0, 90.0, 0.0, 65.0, 0.0]

# Cargo (back of rover) — single swing for release.
CARGO_DEG = [180.0, 25.0, 90.0, 0.0, 55.0, 0.0]

# Legacy fallback when IK is enabled and wrist detection missing.
PICK_TRAJ_DEG: List[List[float]] = [
    [  0.0,  0.0, 90.0, 0.0, 90.0, 0.0],
    [  0.0, 25.0, 90.0, 0.0, 65.0, 0.0],
    [  0.0,  0.0, 90.0, 0.0, 90.0, 0.0],
]


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
        # Best-effort wrist-cam -> link_6 mount calibration. Tune after USD dump.
        self.declare_parameter("wrist_mount_xyz_link6", [0.10, 0.0, 0.0])
        # Rotation: optical (z fwd, x right, y down) -> link_6 axes (best guess).
        # Stored as 9 flat values, row-major 3x3.
        self.declare_parameter("wrist_R_optical_to_link6", [
            0.0, 0.0, 1.0,
            1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
        ])
        self.declare_parameter("ik_use_orientation", False)
        # Phase 3b-4 simple-ver: IK off by default; rely on scripted PICK_TRAJ.
        # When False, pick_mineral skips wrist-cam wait + IK and runs the
        # scripted DOWN -> GRASP -> UP -> BASKET trajectory directly.
        self.declare_parameter("enable_ik", False)
        self.declare_parameter("grasp_command_topic", "/grasp/command")
        # Step delay between grasp publish and arm continuing — gives
        # vehicle_v3 ScriptNode time to attach the FixedJoint.
        self.declare_parameter("grasp_publish_delay_sec", 0.3)

        self.joint_pub = self.create_publisher(
            JointState, str(self.get_parameter("joint_command_topic").value), 10)
        self.grasp_pub = self.create_publisher(
            String, str(self.get_parameter("grasp_command_topic").value), 10)

        self.create_subscription(
            DetectionArray, str(self.get_parameter("wrist_detections_topic").value),
            self._on_wrist_detections, SENSOR_QOS)

        self.create_subscription(
            JointState, str(self.get_parameter("joint_state_topic").value),
            self._on_joint_states, SENSOR_QOS)

        self._wrist_lock = threading.Lock()
        self._wrist_dets: List[Detection] = []
        self._wrist_stamp_ns: int = 0
        # Current arm joint positions (rad) from /joint_states_raw. None until
        # first message arrives — we fall back to HOME_DEG for IK base then.
        self._current_arm_q_rad: Optional[np.ndarray] = None
        self._joint_state_logged_once = False

        self.action_server = ActionServer(
            self,
            ExecuteArmTask,
            "/execute_arm_task",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self._initial_home_sent = False
        self.create_timer(1.0, self._send_initial_home_once)

        self.get_logger().info(
            "arm_executor_node ready: /execute_arm_task (wrist-cam visual servo + DLS-IK)")
        self.get_logger().warn(
            "WRIST_T_LINK6 is best-effort estimate — tune wrist_mount_xyz_link6 / "
            "wrist_R_optical_to_link6 parameters after USD cross-check.")

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
            if bool(self.get_parameter("enable_ik").value):
                return self._execute_pick_ik(goal_handle)
            return self._execute_pick_scripted(goal_handle)
        # Other commands (place_to_cargo, unload_to_base, deploy_solar_panel)
        # — fall back to scripted PLACE_BASKET sequence as visual demo.
        return self._execute_scripted(goal_handle, PLACE_BASKET_TRAJ_DEG, command)

    def _publish_grasp(self, cmd: str, x: float = 0.0, y: float = 0.0,
                       z: float = 0.0, target_id: str = "") -> None:
        """Publish a single-line grasp command for vehicle_v3 to act on.

        Format: 'pickup x y z target_id' or 'release'. Simple space-separated
        text so the OmniGraph ScriptNode can parse without a custom msg.
        """
        msg = String()
        if cmd == "pickup":
            msg.data = f"pickup {x:.4f} {y:.4f} {z:.4f} {target_id}"
        else:
            msg.data = "release"
        self.grasp_pub.publish(msg)
        delay = float(self.get_parameter("grasp_publish_delay_sec").value)
        if delay > 0.0:
            time.sleep(delay)
        self.get_logger().info(f"grasp/command -> {msg.data}")

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
