"""Mission manager node that follows the rover node architecture diagram."""

from __future__ import annotations

import json
from dataclasses import dataclass
from enum import Enum
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from isaac_interfaces.action import ExecuteArmTask, ExecuteDriveTask
from isaac_interfaces.msg import (
    BatteryState,
    MissionState as MissionStateMsg,
    PerceptionResult,
    SelectedDriveAction,
)
from isaac_interfaces.srv import CheckSystemReady, ResetSimulation, SaveExplorationMap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String


class MissionState(str, Enum):
    IDLE = "idle"
    EXPLORING = "exploring"
    NAVIGATING_TO_MINERAL = "navigating_to_mineral"
    COLLECTING = "collecting"
    LOADING_CARGO = "loading_cargo"
    RETURNING_TO_BASE = "returning_to_base"
    UNLOADING = "unloading"
    CHARGING = "charging"
    COMPLETED = "completed"
    PAUSED = "paused"
    ABORTED = "aborted"
    ERROR = "error"


@dataclass
class PendingAction:
    kind: str
    command: str
    started_sec: float
    timeout_sec: float


class MissionManagerNode(Node):
    """Coordinates topics, actions, and services for the whole mission."""

    ACTIVE_STATES = {
        MissionState.EXPLORING,
        MissionState.NAVIGATING_TO_MINERAL,
        MissionState.COLLECTING,
        MissionState.LOADING_CARGO,
        MissionState.RETURNING_TO_BASE,
        MissionState.UNLOADING,
    }

    def __init__(self) -> None:
        super().__init__("mission_manager_node")
        self._declare_parameters()

        self.state = MissionState.IDLE
        self.previous_state = MissionState.IDLE
        self.cargo_count = 0
        self.collected_count = 0
        self.current_waypoint_index = 0
        self.current_target: PerceptionResult | None = None
        self.pending_action: PendingAction | None = None
        self.drive_goal_handle: Any | None = None
        self.arm_goal_handle: Any | None = None
        self.last_drive_action: SelectedDriveAction | None = None
        self.paused_from_state = MissionState.IDLE
        self.last_error = ""

        self.battery_percent = 100.0
        self.low_battery = False
        self.critical_battery = False
        self.system_ready = False

        self.mission_state_pub = self.create_publisher(MissionStateMsg, "/mission_state", 10)
        self.mission_event_pub = self.create_publisher(String, "/mission_event", 10)

        self.create_subscription(String, "/mission_command", self._on_mission_command, 10)
        self.create_subscription(BatteryState, "/battery_state", self._on_battery_state, 10)
        self.create_subscription(PerceptionResult, "/perception_result", self._on_perception_result, 10)
        self.create_subscription(SelectedDriveAction, "/selected_drive_action", self._on_selected_drive_action, 10)

        self.drive_action_client = ActionClient(self, ExecuteDriveTask, "/execute_drive_task")
        self.arm_action_client = ActionClient(self, ExecuteArmTask, "/execute_arm_task")

        self.reset_simulation_client = self.create_client(ResetSimulation, "/reset_simulation")
        self.check_system_ready_client = self.create_client(CheckSystemReady, "/check_system_ready")
        self.save_exploration_map_client = self.create_client(SaveExplorationMap, "/save_exploration_map")

        tick_hz = float(self.get_parameter("tick_hz").value)
        self.create_timer(1.0 / max(tick_hz, 0.1), self._mission_loop)

        self._publish_event("mission_manager_ready")
        self._publish_state()

    def _declare_parameters(self) -> None:
        self.declare_parameter("tick_hz", 2.0)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("cargo_capacity", 3)
        self.declare_parameter("collection_goal", 3)
        self.declare_parameter("low_battery_threshold", 25.0)
        self.declare_parameter("critical_battery_threshold", 10.0)
        self.declare_parameter("resume_battery_threshold", 80.0)
        self.declare_parameter("drive_action_timeout_sec", 120.0)
        self.declare_parameter("arm_action_timeout_sec", 45.0)
        self.declare_parameter("base_pose", '{"x": 0.0, "y": 0.0, "yaw": 0.0}')
        self.declare_parameter(
            "exploration_waypoints",
            (
                '[{"x": 4.0, "y": 0.0, "yaw": 0.0}, '
                '{"x": 4.0, "y": 4.0, "yaw": 1.57}, '
                '{"x": 0.0, "y": 4.0, "yaw": 3.14}]'
            ),
        )

    # Periodically evaluates mission progress, battery policy, and automatic next steps.
    def _mission_loop(self) -> None:
        if bool(self.get_parameter("auto_start").value) and self.state == MissionState.IDLE:
            self._check_ready_then_start()

        self._update_battery_flags()
        self._handle_battery_policy()
        self._handle_action_timeout()

        if self.state == MissionState.EXPLORING and self.pending_action is None:
            if self.last_drive_action is not None:
                self._dispatch_selected_drive_action(self.last_drive_action)
                self.last_drive_action = None
            else:
                self._dispatch_next_exploration_goal()

        if self.state == MissionState.CHARGING and self.battery_percent >= self._resume_battery_threshold:
            self._publish_event("battery_recovered", battery_percent=self.battery_percent)
            if self.cargo_count > 0:
                self._return_to_base()
            else:
                self._transition(MissionState.EXPLORING)

        self._publish_state()

    def _on_mission_command(self, msg: String) -> None:
        command = self._parse_command(msg.data)
        name = str(command.get("command", command.get("type", msg.data))).strip().lower()

        if name == "start":
            self._check_ready_then_start()
        elif name == "pause":
            self._pause_mission()
        elif name == "resume":
            self._resume_mission()
        elif name == "abort":
            self._abort_mission("abort_command")
        elif name == "reset":
            self._request_reset_simulation(command)
        elif name == "return_to_base":
            self._return_to_base()
        elif name == "save_map":
            self._request_save_exploration_map(command)
        elif name == "check_ready":
            self._request_check_system_ready(start_after_ready=False)
        elif name == "set_waypoints":
            waypoints = command.get("waypoints")
            if isinstance(waypoints, list) and waypoints:
                self.set_parameters([Parameter("exploration_waypoints", value=json.dumps(waypoints))])
                self.current_waypoint_index = 0
                self._publish_event("waypoints_updated", count=len(waypoints))
            else:
                self._set_error("set_waypoints requires a non-empty waypoints list")
        else:
            self._set_error(f"unknown mission command: {name}")

    def _on_battery_state(self, msg: BatteryState) -> None:
        self.battery_percent = self._clamp(float(msg.percentage), 0.0, 100.0)
        self.low_battery = bool(msg.is_low) or self.battery_percent <= self._low_battery_threshold
        self.critical_battery = bool(msg.is_critical) or self.battery_percent <= self._critical_battery_threshold
        if self.battery_percent >= self._resume_battery_threshold:
            self.low_battery = False
            self.critical_battery = False

    def _on_selected_drive_action(self, msg: SelectedDriveAction) -> None:
        if self.state != MissionState.EXPLORING or self.pending_action is not None:
            self.last_drive_action = msg
            return
        self._dispatch_selected_drive_action(msg)

    def _on_perception_result(self, msg: PerceptionResult) -> None:
        if not msg.mineral_detected or self.state != MissionState.EXPLORING:
            return

        self.current_target = msg
        self._publish_event(
            "mineral_detected",
            object_id=msg.object_id,
            x=msg.x,
            y=msg.y,
            z=msg.z,
            confidence=msg.confidence,
        )
        self._cancel_active_drive_goal()
        self.pending_action = None
        self._dispatch_drive_task(
            MissionState.NAVIGATING_TO_MINERAL,
            "drive_to_mineral",
            msg.x,
            msg.y,
            0.0,
            target_id=msg.object_id,
            metadata={"source": "perception_result", "confidence": msg.confidence},
        )

    def _check_ready_then_start(self) -> None:
        if self.state in self.ACTIVE_STATES:
            return
        self._request_check_system_ready(start_after_ready=True)

    def _start_mission(self) -> None:
        self._clear_runtime_state()
        self._transition(MissionState.EXPLORING)
        self._publish_event("mission_started")

    def _pause_mission(self) -> None:
        if self.state not in self.ACTIVE_STATES:
            return
        self.paused_from_state = self.state
        self._cancel_active_drive_goal()
        self._cancel_active_arm_goal()
        self.pending_action = None
        self._transition(MissionState.PAUSED)
        self._publish_event("mission_paused")

    def _resume_mission(self) -> None:
        if self.state != MissionState.PAUSED:
            return
        self._publish_event("mission_resumed")
        if self.paused_from_state == MissionState.RETURNING_TO_BASE or self.low_battery:
            self._return_to_base()
        elif self.cargo_count > 0 and self._cargo_full():
            self._return_to_base()
        else:
            self._transition(MissionState.EXPLORING)

    def _abort_mission(self, reason: str) -> None:
        self._cancel_active_drive_goal()
        self._cancel_active_arm_goal()
        self.pending_action = None
        self._transition(MissionState.ABORTED)
        self._publish_event("mission_aborted", reason=reason)

    def _return_to_base(self) -> None:
        base_pose = self._load_json_parameter("base_pose", {"x": 0.0, "y": 0.0, "yaw": 0.0})
        if not self._valid_pose(base_pose):
            self._set_error("base_pose parameter must include x and y")
            return
        self._dispatch_drive_task(
            MissionState.RETURNING_TO_BASE,
            "return_to_base",
            float(base_pose["x"]),
            float(base_pose["y"]),
            float(base_pose.get("yaw", 0.0)),
            metadata={"source": "mission_manager"},
        )

    def _dispatch_next_exploration_goal(self) -> None:
        waypoints = self._load_json_parameter("exploration_waypoints", [])
        if not waypoints:
            self._set_error("exploration_waypoints parameter is empty")
            return

        waypoint = waypoints[self.current_waypoint_index % len(waypoints)]
        self.current_waypoint_index += 1
        if not self._valid_pose(waypoint):
            self._set_error("exploration waypoint must include x and y")
            return

        self._dispatch_drive_task(
            MissionState.EXPLORING,
            "explore_waypoint",
            float(waypoint["x"]),
            float(waypoint["y"]),
            float(waypoint.get("yaw", 0.0)),
            metadata={"source": "mission_waypoint"},
        )

    def _dispatch_selected_drive_action(self, msg: SelectedDriveAction) -> None:
        action = msg.action.strip() or "policy_action"
        self._dispatch_drive_task(
            MissionState.EXPLORING,
            action,
            msg.target_x,
            msg.target_y,
            msg.target_yaw,
            metadata={
                "source": "selected_drive_action",
                "linear_velocity": msg.linear_velocity,
                "angular_velocity": msg.angular_velocity,
                "confidence": msg.confidence,
                "reason": msg.reason,
            },
        )

    def _dispatch_drive_task(
        self,
        next_state: MissionState,
        command: str,
        target_x: float,
        target_y: float,
        target_yaw: float,
        target_id: str = "",
        metadata: dict[str, Any] | None = None,
    ) -> None:
        if self.pending_action is not None:
            return
        if not self.drive_action_client.server_is_ready():
            self._set_error("/execute_drive_task action server is not ready")
            return

        goal = ExecuteDriveTask.Goal()
        goal.command = command
        goal.target_x = float(target_x)
        goal.target_y = float(target_y)
        goal.target_yaw = float(target_yaw)
        goal.target_id = target_id
        goal.metadata = json.dumps(metadata or {})

        self._transition(next_state)
        self.pending_action = PendingAction(
            kind="drive",
            command=command,
            started_sec=self.get_clock().now().nanoseconds / 1e9,
            timeout_sec=float(self.get_parameter("drive_action_timeout_sec").value),
        )
        send_future = self.drive_action_client.send_goal_async(goal, feedback_callback=self._on_drive_feedback)
        send_future.add_done_callback(self._on_drive_goal_response)
        self._publish_event("drive_task_sent", command=command, target_id=target_id)

    def _dispatch_arm_task(
        self,
        next_state: MissionState,
        command: str,
        target: PerceptionResult | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> None:
        if self.pending_action is not None:
            return
        if not self.arm_action_client.server_is_ready():
            self._set_error("/execute_arm_task action server is not ready")
            return

        goal = ExecuteArmTask.Goal()
        goal.command = command
        goal.target_id = target.object_id if target else ""
        goal.target_x = float(target.x) if target else 0.0
        goal.target_y = float(target.y) if target else 0.0
        goal.target_z = float(target.z) if target else 0.0
        goal.metadata = json.dumps(metadata or {})

        self._transition(next_state)
        self.pending_action = PendingAction(
            kind="arm",
            command=command,
            started_sec=self.get_clock().now().nanoseconds / 1e9,
            timeout_sec=float(self.get_parameter("arm_action_timeout_sec").value),
        )
        send_future = self.arm_action_client.send_goal_async(goal, feedback_callback=self._on_arm_feedback)
        send_future.add_done_callback(self._on_arm_goal_response)
        self._publish_event("arm_task_sent", command=command, target_id=goal.target_id)

    def _on_drive_goal_response(self, future: Any) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.pending_action = None
            self._set_error("drive task goal rejected")
            return
        self.drive_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda done, handle=goal_handle: self._on_drive_result(done, handle)
        )

    def _on_drive_feedback(self, feedback_msg: Any) -> None:
        feedback = feedback_msg.feedback
        self._publish_event(
            "drive_task_feedback",
            task_state=feedback.state,
            progress=feedback.progress,
            message=feedback.message,
        )

    def _on_drive_result(self, future: Any, goal_handle: Any) -> None:
        if goal_handle is not self.drive_goal_handle:
            self._publish_event("stale_drive_result_ignored")
            return
        result_msg = future.result()
        result = result_msg.result
        status = result_msg.status
        command = self.pending_action.command if self.pending_action else ""
        self.pending_action = None
        self.drive_goal_handle = None

        if status != GoalStatus.STATUS_SUCCEEDED or not result.success:
            self._set_error(f"drive task {command} failed: {result.message}")
            return

        self._publish_event("drive_task_completed", command=command, message=result.message)
        if command == "drive_to_mineral":
            self._dispatch_arm_task(MissionState.COLLECTING, "pick_mineral", self.current_target)
        elif command == "return_to_base":
            self._dispatch_arm_task(MissionState.UNLOADING, "unload_to_base")
        else:
            self._transition(MissionState.EXPLORING)

    def _on_arm_goal_response(self, future: Any) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.pending_action = None
            self._set_error("arm task goal rejected")
            return
        self.arm_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda done, handle=goal_handle: self._on_arm_result(done, handle)
        )

    def _on_arm_feedback(self, feedback_msg: Any) -> None:
        feedback = feedback_msg.feedback
        self._publish_event(
            "arm_task_feedback",
            task_state=feedback.state,
            progress=feedback.progress,
            message=feedback.message,
        )

    def _on_arm_result(self, future: Any, goal_handle: Any) -> None:
        if goal_handle is not self.arm_goal_handle:
            self._publish_event("stale_arm_result_ignored")
            return
        result_msg = future.result()
        result = result_msg.result
        status = result_msg.status
        command = self.pending_action.command if self.pending_action else ""
        self.pending_action = None
        self.arm_goal_handle = None

        if status != GoalStatus.STATUS_SUCCEEDED or not result.success:
            self._set_error(f"arm task {command} failed: {result.message}")
            return

        self._publish_event("arm_task_completed", command=command, message=result.message)
        if command == "pick_mineral":
            self._dispatch_arm_task(MissionState.LOADING_CARGO, "place_to_cargo", self.current_target)
        elif command == "place_to_cargo":
            self.cargo_count += 1
            self.collected_count += 1
            self.current_target = None
            self._publish_event("mineral_loaded", cargo_count=self.cargo_count, collected_count=self.collected_count)
            if self._mission_goal_reached() or self._cargo_full() or self.low_battery:
                self._return_to_base()
            else:
                self._transition(MissionState.EXPLORING)
        elif command == "unload_to_base":
            unloaded_count = self.cargo_count
            self.cargo_count = 0
            self._publish_event("cargo_unloaded", unloaded_count=unloaded_count)
            if self._mission_goal_reached():
                self._transition(MissionState.COMPLETED)
                self._publish_event("mission_completed", collected_count=self.collected_count)
            elif self.low_battery:
                self._transition(MissionState.CHARGING)
            else:
                self._transition(MissionState.EXPLORING)
        elif command == "deploy_solar_panel":
            self._transition(MissionState.CHARGING)

    def _request_check_system_ready(self, start_after_ready: bool) -> None:
        if not self.check_system_ready_client.service_is_ready():
            self._publish_event("check_system_ready_unavailable")
            if start_after_ready:
                self._set_error("/check_system_ready service is not ready")
            return
        request = CheckSystemReady.Request()
        request.requester = self.get_name()
        future = self.check_system_ready_client.call_async(request)
        future.add_done_callback(lambda done: self._on_check_system_ready(done, start_after_ready))

    def _on_check_system_ready(self, future: Any, start_after_ready: bool) -> None:
        response = future.result()
        self.system_ready = bool(response.ready)
        self._publish_event(
            "check_system_ready_result",
            ready=response.ready,
            missing_nodes=list(response.missing_nodes),
            message=response.message,
        )
        if start_after_ready:
            if response.ready:
                self._start_mission()
            else:
                self._set_error(f"system is not ready: {response.message}")

    def _request_reset_simulation(self, command: dict[str, Any]) -> None:
        if not self.reset_simulation_client.service_is_ready():
            self._set_error("/reset_simulation service is not ready")
            return
        request = ResetSimulation.Request()
        request.reset_world = bool(command.get("reset_world", True))
        request.reset_robot = bool(command.get("reset_robot", True))
        request.reset_mission = bool(command.get("reset_mission", True))
        request.reason = str(command.get("reason", "mission_command"))
        future = self.reset_simulation_client.call_async(request)
        future.add_done_callback(self._on_reset_simulation)

    def _on_reset_simulation(self, future: Any) -> None:
        response = future.result()
        self._publish_event("reset_simulation_result", success=response.success, message=response.message)
        if response.success:
            self._cancel_active_drive_goal()
            self._cancel_active_arm_goal()
            self._clear_runtime_state()
            self._transition(MissionState.IDLE)
        else:
            self._set_error(f"reset_simulation failed: {response.message}")

    def _request_save_exploration_map(self, command: dict[str, Any]) -> None:
        if not self.save_exploration_map_client.service_is_ready():
            self._set_error("/save_exploration_map service is not ready")
            return
        request = SaveExplorationMap.Request()
        request.map_name = str(command.get("map_name", "mars_exploration_map"))
        request.target_path = str(command.get("target_path", ""))
        future = self.save_exploration_map_client.call_async(request)
        future.add_done_callback(self._on_save_exploration_map)

    def _on_save_exploration_map(self, future: Any) -> None:
        response = future.result()
        self._publish_event(
            "save_exploration_map_result",
            success=response.success,
            saved_path=response.saved_path,
            message=response.message,
        )
        if not response.success:
            self._set_error(f"save_exploration_map failed: {response.message}")

    def _handle_battery_policy(self) -> None:
        if self.state not in self.ACTIVE_STATES:
            return

        if self.critical_battery:
            self._publish_event("critical_battery", battery_percent=self.battery_percent)
            self._cancel_active_drive_goal()
            self.pending_action = None
            self._dispatch_arm_task(MissionState.CHARGING, "deploy_solar_panel")
            return

        if self.state in {MissionState.COLLECTING, MissionState.LOADING_CARGO, MissionState.UNLOADING}:
            return

        if self.low_battery and self.state != MissionState.RETURNING_TO_BASE:
            self._publish_event("low_battery_returning_to_base", battery_percent=self.battery_percent)
            self._return_to_base()

    def _handle_action_timeout(self) -> None:
        if self.pending_action is None:
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.pending_action.started_sec <= self.pending_action.timeout_sec:
            return
        timed_out = self.pending_action
        self.pending_action = None
        if timed_out.kind == "drive":
            self._cancel_active_drive_goal()
        elif timed_out.kind == "arm":
            self._cancel_active_arm_goal()
        self._set_error(f"{timed_out.kind} action {timed_out.command} timed out")

    def _cancel_active_drive_goal(self) -> None:
        if self.drive_goal_handle is not None:
            self.drive_goal_handle.cancel_goal_async()
            self.drive_goal_handle = None

    def _cancel_active_arm_goal(self) -> None:
        if self.arm_goal_handle is not None:
            self.arm_goal_handle.cancel_goal_async()
            self.arm_goal_handle = None

    def _transition(self, state: MissionState) -> None:
        if self.state == state:
            return
        self.previous_state = self.state
        self.state = state
        self._publish_event("state_changed", previous=self.previous_state.value, current=self.state.value)

    def _publish_state(self) -> None:
        msg = MissionStateMsg()
        msg.state = self.state.value
        msg.previous_state = self.previous_state.value
        msg.battery_percent = float(self.battery_percent)
        msg.low_battery = bool(self.low_battery)
        msg.critical_battery = bool(self.critical_battery)
        msg.cargo_count = int(self.cargo_count)
        msg.cargo_capacity = int(self.get_parameter("cargo_capacity").value)
        msg.collected_count = int(self.collected_count)
        msg.collection_goal = int(self.get_parameter("collection_goal").value)
        msg.active_task = self.pending_action.command if self.pending_action else ""
        msg.last_error = self.last_error
        self.mission_state_pub.publish(msg)

    def _publish_event(self, event: str, **fields: Any) -> None:
        payload = {"event": event, "state": self.state.value, **fields}
        self.mission_event_pub.publish(String(data=json.dumps(payload)))

    def _clear_runtime_state(self) -> None:
        self.cargo_count = 0
        self.collected_count = 0
        self.current_waypoint_index = 0
        self.current_target = None
        self.pending_action = None
        self.drive_goal_handle = None
        self.arm_goal_handle = None
        self.last_drive_action = None
        self.last_error = ""

    def _set_error(self, message: str) -> None:
        self.last_error = message
        self._cancel_active_drive_goal()
        self._cancel_active_arm_goal()
        self.pending_action = None
        self._transition(MissionState.ERROR)
        self._publish_event("mission_error", message=message)
        self.get_logger().error(message)

    def _update_battery_flags(self) -> None:
        self.low_battery = self.low_battery or self.battery_percent <= self._low_battery_threshold
        self.critical_battery = self.critical_battery or self.battery_percent <= self._critical_battery_threshold
        if self.battery_percent >= self._resume_battery_threshold:
            self.low_battery = False
            self.critical_battery = False

    def _mission_goal_reached(self) -> bool:
        return self.collected_count >= int(self.get_parameter("collection_goal").value)

    def _cargo_full(self) -> bool:
        return self.cargo_count >= int(self.get_parameter("cargo_capacity").value)

    @property
    def _low_battery_threshold(self) -> float:
        return float(self.get_parameter("low_battery_threshold").value)

    @property
    def _critical_battery_threshold(self) -> float:
        return float(self.get_parameter("critical_battery_threshold").value)

    @property
    def _resume_battery_threshold(self) -> float:
        return float(self.get_parameter("resume_battery_threshold").value)

    def _load_json_parameter(self, name: str, fallback: Any) -> Any:
        value = self.get_parameter(name).value
        if isinstance(value, str):
            try:
                return json.loads(value)
            except json.JSONDecodeError:
                self._set_error(f"{name} parameter is not valid JSON")
                return fallback
        return value

    @staticmethod
    def _parse_command(data: str) -> dict[str, Any]:
        text = data.strip()
        if not text:
            return {}
        try:
            payload = json.loads(text)
            return payload if isinstance(payload, dict) else {"value": payload}
        except json.JSONDecodeError:
            return {"command": text}

    @staticmethod
    def _valid_pose(value: Any) -> bool:
        if not isinstance(value, dict):
            return False
        try:
            float(value["x"])
            float(value["y"])
        except (KeyError, TypeError, ValueError):
            return False
        return True

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(value, maximum))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
