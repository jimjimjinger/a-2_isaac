"""Robot arm action server — M0609 scripted trajectory (Phase 3a).

Implements /execute_arm_task by publishing /arm/joint_command (JointState)
through a linearly-interpolated waypoint sequence at publish_hz for
visual smoothness.

No IK, no grasp simulation in this phase — joint trajectory is a static
script taken from T2 standalone rover_yolo_demo:
  HOME -> joint_1=180 -> joint_2=25+joint_5=55 -> return -> HOME

Phase 3b will add real DLS-IK + FixedJoint grasp/release. The action
contract (command, target_x/y/z, success/message) stays stable.
"""
from __future__ import annotations

import math
import time
from typing import List

import rclpy
from isaac_interfaces.action import ExecuteArmTask
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
HOME_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

PICK_TRAJ_DEG: List[List[float]] = [
    [  0.0,  0.0, 90.0, 0.0, 90.0, 0.0],   # HOME (start)
    [180.0,  0.0, 90.0, 0.0, 90.0, 0.0],   # joint_1: base 180 deg (back)
    [180.0, 25.0, 90.0, 0.0, 55.0, 0.0],   # joint_2 dip + joint_5 wrist dump
    [180.0,  0.0, 90.0, 0.0, 90.0, 0.0],   # shoulder/wrist return
    [  0.0,  0.0, 90.0, 0.0, 90.0, 0.0],   # HOME (end)
]


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
        self.declare_parameter("step_duration_sec", 1.8)
        self.declare_parameter("joint_command_topic", "/arm/joint_command")

        self.joint_pub = self.create_publisher(
            JointState, str(self.get_parameter("joint_command_topic").value), 10)

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
            "arm_executor_node ready: /execute_arm_task (scripted trajectory)")

    def _send_initial_home_once(self) -> None:
        if self._initial_home_sent:
            return
        self._publish_joint_state(HOME_DEG)
        self._initial_home_sent = True

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

        publish_hz = max(5.0, float(self.get_parameter("publish_hz").value))
        step_dur = max(0.2, float(self.get_parameter("step_duration_sec").value))
        n_interp = max(2, int(round(publish_hz * step_dur)))
        dt = 1.0 / publish_hz

        trajectory = PICK_TRAJ_DEG
        total_steps = max(1, len(trajectory) - 1)

        cur = list(trajectory[0])
        for seg_idx in range(total_steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info(f"Arm goal {command} canceled")
                self._publish_joint_state(HOME_DEG)
                return ExecuteArmTask.Result(success=False, message="canceled")

            target = trajectory[seg_idx + 1]
            for k in range(1, n_interp + 1):
                t = k / n_interp
                interp = [cur[i] + (target[i] - cur[i]) * t for i in range(6)]
                self._publish_joint_state(interp)
                fb = ExecuteArmTask.Feedback()
                fb.state = "manipulating"
                fb.progress = float((seg_idx + t) / total_steps)
                fb.message = f"seg {seg_idx + 1}/{total_steps}"
                goal_handle.publish_feedback(fb)
                time.sleep(dt)
            cur = list(target)

        self._publish_joint_state(HOME_DEG)
        goal_handle.succeed()
        self.get_logger().info(f"Arm command {command} done")
        return ExecuteArmTask.Result(success=True, message=f"{command} completed")

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
