"""Robot arm action server for manipulation tasks."""

from __future__ import annotations

import time

import rclpy
from isaac_interfaces.action import ExecuteArmTask
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class ArmExecutorNode(Node):
    """Executes high-level arm tasks.

    This is a mock action server for module integration. Replace task bodies with
    MoveIt/Isaac Sim arm control while keeping the action contract stable.
    """

    SUPPORTED_COMMANDS = {
        "pick_mineral",
        "place_to_cargo",
        "unload_to_base",
        "deploy_solar_panel",
    }

    def __init__(self) -> None:
        super().__init__("arm_executor_node")
        self.declare_parameter("mock_duration_sec", 2.0)
        self.declare_parameter("feedback_hz", 5.0)
        self.action_server = ActionServer(
            self,
            ExecuteArmTask,
            "/execute_arm_task",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info("arm_executor_node ready: /execute_arm_task")

    def _goal_callback(self, goal_request: ExecuteArmTask.Goal) -> GoalResponse:
        command = goal_request.command.strip()
        if command not in self.SUPPORTED_COMMANDS:
            self.get_logger().warning(f"Rejecting unsupported arm command: {command}")
            return GoalResponse.REJECT
        self.get_logger().info(f"Accepted arm goal {command} target={goal_request.target_id}")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: object) -> CancelResponse:
        self.get_logger().info("Arm goal cancel requested")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: object) -> ExecuteArmTask.Result:
        goal = goal_handle.request
        duration_sec = max(float(self.get_parameter("mock_duration_sec").value), 0.1)
        feedback_hz = max(float(self.get_parameter("feedback_hz").value), 0.1)
        steps = max(int(duration_sec * feedback_hz), 1)

        # TODO(real manipulation): replace this timed mock loop with calls into
        # manipulation_primitives, MoveIt, gripper drivers, or Isaac Sim
        # articulation control. The supported command names are the stable
        # mission contract: pick_mineral, place_to_cargo, unload_to_base, and
        # deploy_solar_panel.
        for step in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return ExecuteArmTask.Result(success=False, message="arm task canceled")
            feedback = ExecuteArmTask.Feedback()
            feedback.state = "manipulating"
            feedback.progress = float(step + 1) / float(steps)
            feedback.message = f"executing {goal.command}"
            goal_handle.publish_feedback(feedback)
            time.sleep(1.0 / feedback_hz)

        goal_handle.succeed()
        return ExecuteArmTask.Result(success=True, message=f"{goal.command} completed")


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
