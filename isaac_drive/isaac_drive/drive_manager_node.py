"""Navigation manager action server for rover navigation tasks."""

from __future__ import annotations

import threading
import time

import rclpy
from isaac_interfaces.action import ExecuteDriveTask, NavigateTask
from isaac_interfaces.msg import MissionState, PerceptionResult, SelectedDriveAction
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class NavigationManagerNode(Node):
    """Plans navigation-level tasks and delegates drive execution.

    Mission manager owns mission intent. This node owns navigation decisions and
    converts those intents into concrete drive tasks for the mobile base
    executor. Real route planning, obstacle avoidance, and RL policy blending
    should be added here instead of in the mission manager.
    """

    def __init__(self) -> None:
        super().__init__("navigation_manager_node")
        self.declare_parameter("drive_server_timeout_sec", 2.0)
        self.callback_group = ReentrantCallbackGroup()

        self.latest_mission_state: MissionState | None = None
        self.latest_perception: PerceptionResult | None = None
        self.latest_policy_action: SelectedDriveAction | None = None
        self.active_navigation_goal_handle: object | None = None
        self.drive_goal_handles: dict[int, object] = {}
        self._goal_lock = threading.Lock()

        self.create_subscription(MissionState, "/mission_state", self._on_mission_state, 10)
        self.create_subscription(PerceptionResult, "/perception_result", self._on_perception, 10)
        self.create_subscription(SelectedDriveAction, "/selected_drive_action", self._on_drive_action, 10)

        self.drive_action_client = ActionClient(
            self,
            ExecuteDriveTask,
            "/execute_drive_task",
            callback_group=self.callback_group,
        )
        self.navigation_action_server = ActionServer(
            self,
            NavigateTask,
            "/navigate_task",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self.get_logger().info("navigation_manager_node ready: /navigate_task -> /execute_drive_task")

    def _on_mission_state(self, msg: MissionState) -> None:
        self.latest_mission_state = msg
        self.get_logger().debug(f"mission state: {msg.state}")

    def _on_perception(self, msg: PerceptionResult) -> None:
        self.latest_perception = msg
        self.get_logger().debug(f"perception mineral_detected={msg.mineral_detected}")

    def _on_drive_action(self, msg: SelectedDriveAction) -> None:
        self.latest_policy_action = msg
        self.get_logger().debug(f"selected drive action: {msg.action}")

    def _goal_callback(self, goal_request: NavigateTask.Goal) -> GoalResponse:
        if not goal_request.command.strip():
            self.get_logger().warning("Rejecting navigation goal with empty command")
            return GoalResponse.REJECT
        self.get_logger().info(
            "Accepted navigation goal %s -> (%.2f, %.2f, %.2f)"
            % (
                goal_request.command,
                goal_request.target_x,
                goal_request.target_y,
                goal_request.target_yaw,
            )
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: object) -> CancelResponse:
        self.get_logger().info("Navigation goal cancel requested")
        with self._goal_lock:
            drive_goal_handle = self.drive_goal_handles.get(id(goal_handle))
        if drive_goal_handle is not None:
            drive_goal_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: object) -> NavigateTask.Result:
        with self._goal_lock:
            self.active_navigation_goal_handle = goal_handle

        try:
            return self._run_navigation_goal(goal_handle)
        finally:
            with self._goal_lock:
                if self.active_navigation_goal_handle is goal_handle:
                    self.active_navigation_goal_handle = None
                self.drive_goal_handles.pop(id(goal_handle), None)

    def _run_navigation_goal(self, goal_handle: object) -> NavigateTask.Result:
        request = goal_handle.request
        timeout_sec = max(float(self.get_parameter("drive_server_timeout_sec").value), 0.1)
        if not self.drive_action_client.wait_for_server(timeout_sec=timeout_sec):
            goal_handle.abort()
            return NavigateTask.Result(success=False, message="/execute_drive_task action server is not ready")

        drive_goal = self._build_drive_goal(request)

        accepted_event = threading.Event()
        result_event = threading.Event()
        state: dict[str, object] = {
            "accepted": False,
            "drive_goal_handle": None,
            "result": None,
            "status": None,
        }

        send_future = self.drive_action_client.send_goal_async(
            drive_goal,
            feedback_callback=lambda feedback: self._relay_drive_feedback(goal_handle, feedback),
        )

        def on_drive_goal_response(future: object) -> None:
            drive_goal_handle = future.result()
            state["accepted"] = bool(drive_goal_handle.accepted)
            accepted_event.set()
            if not drive_goal_handle.accepted:
                result_event.set()
                return
            state["drive_goal_handle"] = drive_goal_handle
            with self._goal_lock:
                self.drive_goal_handles[id(goal_handle)] = drive_goal_handle
            drive_goal_handle.get_result_async().add_done_callback(on_drive_result)

        def on_drive_result(future: object) -> None:
            result_msg = future.result()
            state["result"] = result_msg.result
            state["status"] = result_msg.status
            result_event.set()

        send_future.add_done_callback(on_drive_goal_response)

        while not accepted_event.is_set():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateTask.Result(success=False, message="navigation task canceled before drive start")
            time.sleep(0.02)

        if not bool(state["accepted"]):
            goal_handle.abort()
            return NavigateTask.Result(success=False, message="drive task goal rejected")

        while not result_event.is_set():
            if goal_handle.is_cancel_requested:
                drive_goal_handle = state.get("drive_goal_handle")
                if drive_goal_handle is not None:
                    drive_goal_handle.cancel_goal_async()
                goal_handle.canceled()
                return NavigateTask.Result(success=False, message="navigation task canceled")
            time.sleep(0.02)

        drive_result = state["result"]
        if not getattr(drive_result, "success", False):
            goal_handle.abort()
            return NavigateTask.Result(
                success=False,
                message=f"drive task failed: {getattr(drive_result, 'message', '')}",
            )

        goal_handle.succeed()
        return NavigateTask.Result(
            success=True,
            message=f"navigation task {request.command} completed: {drive_result.message}",
        )

    def _build_drive_goal(self, request: NavigateTask.Goal) -> ExecuteDriveTask.Goal:
        """Convert mission-level navigation intent into a drive executor task."""
        goal = ExecuteDriveTask.Goal()
        goal.command = request.command
        goal.target_x = request.target_x
        goal.target_y = request.target_y
        goal.target_yaw = request.target_yaw
        goal.target_id = request.target_id
        goal.metadata = request.metadata

        # TODO(real navigation): this is the main place for route planning,
        # obstacle avoidance, manual/autonomous mode logic, and RL policy
        # blending. Mission manager should only send intent such as
        # explore_waypoint, drive_to_mineral, or return_to_base; this node should
        # decide the concrete executor command(s).
        #
        # Current placeholder behavior: when exploring, prefer the latest
        # driving_policy_node output if one exists. Otherwise forward the mission
        # target directly to mobile_base_executor_node.
        if request.command == "explore_waypoint" and self.latest_policy_action is not None:
            goal.command = self.latest_policy_action.action or request.command
            goal.target_x = self.latest_policy_action.target_x
            goal.target_y = self.latest_policy_action.target_y
            goal.target_yaw = self.latest_policy_action.target_yaw

        return goal

    def _relay_drive_feedback(self, goal_handle: object, feedback_msg: object) -> None:
        drive_feedback = feedback_msg.feedback
        feedback = NavigateTask.Feedback()
        feedback.state = f"drive:{drive_feedback.state}"
        feedback.progress = drive_feedback.progress
        feedback.message = drive_feedback.message
        goal_handle.publish_feedback(feedback)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NavigationManagerNode()
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
