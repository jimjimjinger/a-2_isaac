"""Mobile base action server for rover drive tasks."""

from __future__ import annotations

import time

import rclpy
from isaac_interfaces.action import ExecuteDriveTask
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class MobileBaseExecutorNode(Node):
    """Executes high-level drive tasks.

    This stub completes accepted goals after publishing progress feedback. The
    Isaac Sim command bridge can later replace `_execute_callback` internals with
    Ackermann command publishing and target-distance checks.
    """

    def __init__(self) -> None:
        super().__init__("mobile_base_executor_node")
        self.declare_parameter("mock_duration_sec", 3.0)
        self.declare_parameter("feedback_hz", 5.0)
        self.action_server = ActionServer(
            self,
            ExecuteDriveTask,
            "/execute_drive_task",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info("mobile_base_executor_node ready: /execute_drive_task")

    def _goal_callback(self, goal_request: ExecuteDriveTask.Goal) -> GoalResponse:
        if not goal_request.command.strip():
            self.get_logger().warning("Rejecting drive goal with empty command")
            return GoalResponse.REJECT
        self.get_logger().info(
            "Accepted drive goal %s -> (%.2f, %.2f, %.2f)"
            % (
                goal_request.command,
                goal_request.target_x,
                goal_request.target_y,
                goal_request.target_yaw,
            )
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: object) -> CancelResponse:
        self.get_logger().info("Drive goal cancel requested")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: object) -> ExecuteDriveTask.Result:
        goal = goal_handle.request
        duration_sec = max(float(self.get_parameter("mock_duration_sec").value), 0.1)
        feedback_hz = max(float(self.get_parameter("feedback_hz").value), 0.1)
        steps = max(int(duration_sec * feedback_hz), 1)

        # TODO(real driving): replace this timed mock loop with actual rover
        # control. Depending on the team's backend, this is where ExecuteDriveTask
        # should call navigation_primitives, publish /cmd_vel, send a Nav2 goal,
        # or command Isaac Sim wheel/articulation controllers. Keep action
        # feedback/result semantics stable for mission_manager and
        # navigation_manager.
        for step in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return ExecuteDriveTask.Result(success=False, message="drive task canceled")
            feedback = ExecuteDriveTask.Feedback()
            feedback.state = "driving"
            feedback.progress = float(step + 1) / float(steps)
            feedback.message = f"executing {goal.command}"
            goal_handle.publish_feedback(feedback)
            time.sleep(1.0 / feedback_hz)

        goal_handle.succeed()
        return ExecuteDriveTask.Result(success=True, message=f"{goal.command} completed")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MobileBaseExecutorNode()
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
