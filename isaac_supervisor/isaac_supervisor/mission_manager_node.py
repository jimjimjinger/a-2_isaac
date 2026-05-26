"""Mission manager — supervisor level FSM.

Pattern: cmd_vel mux that never touches coverage_node source.

Modes (orthogonal to phase, controlled via /mission/mode):
- AUTO    : phase-driven autonomy (default)
- MANUAL  : pass-through from /teleop/cmd_vel directly to /cmd_vel
            (phase still tracked for telemetry, but ignored for actuation)

Phases (AUTO mode):
- EXPLORE          : pass-through from /coverage/cmd_vel_raw
- APPROACH         : A* way-point pursuit toward best target
- PICK_READY       : stop within stop_distance, arm action client fires
                     (hysteresis: APPROACH->PICK_READY at stop_dist,
                      PICK_READY->APPROACH at stop_dist + resume_buffer)
- RETURN_TO_BASE   : A* path to (0, 0) basecamp — triggered when
                     collected_count >= collection_goal OR battery critical
- MISSION_COMPLETE : stopped at basecamp, mission ended

Coverage_node must be launched with topic remap:
  ros2 run isaac_drive coverage_node --ros-args -r /cmd_vel:=/coverage/cmd_vel_raw

On shutdown the node publishes zero twist a few times so the rover does
not keep the last angular velocity.
"""
from __future__ import annotations

import math
import os
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Empty, String

from isaac_interfaces.action import ExecuteArmTask
from isaac_interfaces.msg import BatteryState, Detection, DetectionArray, MissionState
from isaac_drive.navigation.path_planner import astar, simplify_path
from isaac_drive.navigation.terrain_loader import load_terrain


def _default_terrain_dir() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.normpath(os.path.join(here, "..", "..", "..", "..", "src",
                                      "a2_isaac", "isaac_sim", "assets",
                                      "generated_terrains", "terrain_00004")),
        os.path.expanduser("~/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/"
                           "generated_terrains/terrain_00004"),
    ]
    for p in candidates:
        if os.path.isdir(p):
            return p
    return candidates[-1]


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


VALUE_PRIORITY = {"yellow_mineral": 50.0, "green_gas": 25.0, "blue_mineral": 10.0}

# Per-class stop distance — balanced for wrist cam visibility (max depth ~1.1m)
# and rover P-control overshoot. T2 standalone uses tighter values (0.25 blue,
# 0.75 yellow/green) but assumes precise rover positioning; we widen here.
CLASS_STOP_DIST_M = {
    "blue_mineral":   0.55,
    "yellow_mineral": 0.70,
    "green_gas":      0.70,
}


def _score(det: Detection) -> float:
    return float(det.value_score) + 0.1 * float(det.confidence)


def _stop_dist_for(class_name: str, default_m: float) -> float:
    return float(CLASS_STOP_DIST_M.get(class_name, default_m))


class MissionManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_manager_node")

        self.declare_parameter("approach_engage_dist_m", 8.0)
        self.declare_parameter("approach_stop_dist_m", 0.75)   # default fallback
        self.declare_parameter("approach_resume_buffer_m", 0.3)
        self.declare_parameter("coverage_cmd_stale_sec", 1.0)
        self.declare_parameter("post_pick_cooldown_sec", 8.0)
        self.declare_parameter("post_pick_skip_radius_m", 1.0)
        self.declare_parameter("arm_action_name", "/execute_arm_task")
        self.declare_parameter("enable_arm_action", True)
        self.declare_parameter("approach_lin_speed", 0.6)
        self.declare_parameter("approach_creep_speed", 0.25)
        self.declare_parameter("steer_gain", 1.5)
        self.declare_parameter("detection_stale_sec", 1.5)
        self.declare_parameter("approach_lock_timeout_sec", 8.0)
        self.declare_parameter("explore_resume_delay_sec", 2.0)
        self.declare_parameter("phase_log_period_sec", 1.0)
        self.declare_parameter("coverage_cmd_topic", "/coverage/cmd_vel_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("teleop_cmd_topic", "/teleop/cmd_vel")
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("odom_topic", "/ground_truth/odom")
        self.declare_parameter("phase_topic", "/mission/phase")
        self.declare_parameter("state_topic", "/mission/state")
        self.declare_parameter("mode_topic", "/mission/mode")
        self.declare_parameter("estop_topic", "/mission/estop")
        self.declare_parameter("battery_topic", "/battery_state")
        self.declare_parameter("min_confidence", 0.6)
        self.declare_parameter("terrain_dir", _default_terrain_dir())
        self.declare_parameter("path_cell_size", 0.2)
        # rover 실제 차폭 (m0609 팔 + 후방 바스켓 포함) ≈ 1.5m. inflated 반경
        # 1.0m 면 path 가 obstacle 중심에서 1m 마진 → rover 차체 끝에서도
        # 0.25m 여유. 이전 0.7m 는 mars rover 크기에 부족해 가장자리 충돌 잦음.
        self.declare_parameter("path_robot_radius", 1.0)
        self.declare_parameter("path_replan_target_delta_m", 0.4)
        self.declare_parameter("waypoint_reach_dist_m", 0.4)
        # Mission termination
        self.declare_parameter("collection_goal", 5)
        self.declare_parameter("cargo_capacity", 5)
        self.declare_parameter("basecamp_x", 0.0)
        self.declare_parameter("basecamp_y", 0.0)
        # Basecamp 자체는 obstacle 로 마킹된 영역 (terrain meta: keepout 반경 6m,
        # visual_footprint 8x8m). path_robot_radius=1.0m inflate 후 obstacle
        # 외곽 ≈ 7m. fringe 는 그보다 살짝 바깥 (7.5m), arrive 는 fringe 가깝게
        # 둬서 fringe 도달이 곧 mission_complete.
        self.declare_parameter("basecamp_fringe_radius_m", 7.5)
        self.declare_parameter("basecamp_arrive_radius_m", 8.0)
        self.declare_parameter("rtb_lin_speed", 0.9)
        self.declare_parameter("teleop_stale_sec", 0.5)
        self.declare_parameter("state_publish_hz", 2.0)

        self.phase = "EXPLORE"
        self.mode = "AUTO"
        self.estop = False
        # Mineral collection / battery
        self.collected_count: int = 0
        self.cargo_count: int = 0
        self.collected_by_class: dict[str, int] = {
            "blue_mineral": 0, "yellow_mineral": 0, "green_gas": 0}
        self.battery_percent: float = 100.0
        self.battery_low: bool = False
        self.battery_critical: bool = False
        # Teleop
        self.last_teleop_cmd: Optional[Twist] = None
        self.last_teleop_ns: int = 0
        # Last error string for MissionState
        self.last_error: str = ""
        self.target: Optional[Detection] = None
        self.last_det_stamp_ns: int = 0
        self.last_coverage_cmd: Optional[Twist] = None
        self.last_coverage_cmd_ns: int = 0
        self.last_odom: Optional[Odometry] = None
        # A* planning state
        self.ogrid = None
        self.waypoints: List[Tuple[float, float]] = []
        self.wp_idx: int = 0
        self.last_plan_target: Optional[Tuple[float, float]] = None
        # Arm action state
        self.arm_in_flight: bool = False
        self.arm_client: Optional[ActionClient] = None
        # Post-pick cooldown
        self.last_pick_world: Optional[Tuple[float, float]] = None
        self.last_pick_done_ns: int = 0
        # Target lock-on (kept while APPROACH, even if detection briefly lost)
        self.lock_target: Optional[Detection] = None
        self.lock_started_ns: int = 0
        # EXPLORE resume cooldown — force stop briefly after any APPROACH/PICK
        # exit so coverage_node can replan from the rover's new position.
        self.explore_entry_ns: int = 0

        terrain_dir = str(self.get_parameter("terrain_dir").value)
        try:
            _meta, self.ogrid, _fog = load_terrain(
                terrain_dir,
                cell_size=float(self.get_parameter("path_cell_size").value),
                robot_radius=float(self.get_parameter("path_robot_radius").value),
            )
            self.get_logger().info(
                f"obstacle_grid loaded from {terrain_dir} "
                f"({self.ogrid.rows}x{self.ogrid.cols} @ {self.ogrid.cell_size:.2f}m)")
        except Exception as e:
            self.get_logger().error(
                f"terrain load FAILED ({e}) — APPROACH/RTB will fall back to straight-line P-control")

        if bool(self.get_parameter("enable_arm_action").value):
            self.arm_client = ActionClient(
                self, ExecuteArmTask, str(self.get_parameter("arm_action_name").value))

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.phase_pub = self.create_publisher(
            String, str(self.get_parameter("phase_topic").value), 10)
        self.state_pub = self.create_publisher(
            MissionState, str(self.get_parameter("state_topic").value), 10)
        # Tell coverage_node to drop its stale DRIVE path and replan from
        # the rover's new post-APPROACH/PICK position.
        self.replan_pub = self.create_publisher(
            Empty, "/coverage/replan_request", 10)
        # Supervisor 가 APPROACH/RTB 시 자체적으로 그린 A* path / target.
        # coverage_node 의 /mission/path / /mission/markers 와 별개 토픽이라
        # 두 source 가 시간상 분리돼서 UI 에 표시됨 (EXPLORE 때는 coverage,
        # 그 외엔 supervisor).
        self.supervisor_path_pub = self.create_publisher(
            Path, "/supervisor/path", 1)
        self.supervisor_target_pub = self.create_publisher(
            PointStamped, "/supervisor/target", 1)

        self.create_subscription(
            DetectionArray, str(self.get_parameter("detections_topic").value),
            self._on_detections, SENSOR_QOS)
        self.create_subscription(
            Twist, str(self.get_parameter("coverage_cmd_topic").value),
            self._on_coverage_cmd, 10)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self._on_odom, SENSOR_QOS)
        self.create_subscription(
            Twist, str(self.get_parameter("teleop_cmd_topic").value),
            self._on_teleop_cmd, 10)
        self.create_subscription(
            String, str(self.get_parameter("mode_topic").value),
            self._on_mode, 10)
        self.create_subscription(
            Empty, str(self.get_parameter("estop_topic").value),
            self._on_estop, 10)
        self.create_subscription(
            BatteryState, str(self.get_parameter("battery_topic").value),
            self._on_battery, 10)

        self.create_timer(0.05, self._tick)
        self.create_timer(
            float(self.get_parameter("phase_log_period_sec").value), self._log_phase)
        state_hz = max(float(self.get_parameter("state_publish_hz").value), 0.5)
        self.create_timer(1.0 / state_hz, self._publish_state)
        self.get_logger().info(
            f"mission_manager_node ready. mode=AUTO phase=EXPLORE goal="
            f"{int(self.get_parameter('collection_goal').value)} minerals.")

    def _on_detections(self, msg: DetectionArray) -> None:
        # While the arm is executing a pick, ignore new detections so the
        # current target/phase does not get swapped mid-action.
        if self.arm_in_flight:
            return
        min_conf = float(self.get_parameter("min_confidence").value)
        cooldown_ns = int(float(self.get_parameter("post_pick_cooldown_sec").value) * 1e9)
        skip_r = float(self.get_parameter("post_pick_skip_radius_m").value)
        now_ns = self.get_clock().now().nanoseconds

        def in_cooldown(d: Detection) -> bool:
            if self.last_pick_world is None:
                return False
            if now_ns - self.last_pick_done_ns >= cooldown_ns:
                return False
            dx = d.world_position.x - self.last_pick_world[0]
            dy = d.world_position.y - self.last_pick_world[1]
            return (dx * dx + dy * dy) <= (skip_r * skip_r)

        candidates = [
            d for d in msg.detections
            if d.confidence >= min_conf
            and not (d.world_position.x == 0.0
                     and d.world_position.y == 0.0
                     and d.world_position.z == 0.0)
            and not in_cooldown(d)
        ]
        if not candidates:
            return

        if self.lock_target is not None:
            # Lock 중 — 같은 mineral 의 detection 만 받아 좌표 + lock timer 갱신.
            # 다른 mineral 은 무시 (lock 핵심). 같은 mineral 판정 =
            # class_name 일치 + 좌표 2m 이내.
            lx = self.lock_target.world_position.x
            ly = self.lock_target.world_position.y
            match_r2 = 2.0 * 2.0
            same = [
                d for d in candidates
                if d.class_name == self.lock_target.class_name
                and (d.world_position.x - lx) ** 2
                    + (d.world_position.y - ly) ** 2 < match_r2
            ]
            if same:
                # Pick the candidate closest to the locked coordinate, not the
                # most confident one — prevents lock swap when a different
                # mineral of the same class happens to land within the match
                # radius with higher confidence.
                best = min(
                    same,
                    key=lambda d: (d.world_position.x - lx) ** 2
                                  + (d.world_position.y - ly) ** 2,
                )
                self.target = best
                self.last_det_stamp_ns = now_ns
                self.lock_target = best
                self.lock_started_ns = now_ns  # 갱신: 시야 안 들어오면 timeout reset
            # 같은 mineral 없으면 lock 유지 (timeout 자연 흐름)
            return

        # Lock 없음 — best target 자유 선택
        best = max(candidates, key=_score)
        self.target = best
        self.last_det_stamp_ns = now_ns

    def _on_coverage_cmd(self, msg: Twist) -> None:
        self.last_coverage_cmd = msg
        self.last_coverage_cmd_ns = self.get_clock().now().nanoseconds

    def _on_odom(self, msg: Odometry) -> None:
        self.last_odom = msg

    def _on_teleop_cmd(self, msg: Twist) -> None:
        self.last_teleop_cmd = msg
        self.last_teleop_ns = self.get_clock().now().nanoseconds

    def _on_mode(self, msg: String) -> None:
        new_mode = (msg.data or "").strip().upper()
        if new_mode not in ("AUTO", "MANUAL"):
            self.get_logger().warn(
                f"ignored unknown mission mode: {msg.data!r} (expected AUTO/MANUAL)")
            return
        if new_mode == self.mode:
            return
        self.get_logger().info(f"mode: {self.mode} -> {new_mode}")
        self.mode = new_mode
        # Reset autonomy lock/target when switching to MANUAL so re-engaging
        # AUTO starts from a clean phase.
        if self.mode == "MANUAL":
            self.target = None
            self.lock_target = None
            self.waypoints = []
            self.wp_idx = 0
            self.last_plan_target = None

    def _on_estop(self, _msg: Empty) -> None:
        if not self.estop:
            self.get_logger().warn("ESTOP received — mission halted.")
        self.estop = True
        self.last_error = "ESTOP triggered"

    def _on_battery(self, msg: BatteryState) -> None:
        self.battery_percent = float(msg.percentage)
        self.battery_low = bool(msg.is_low)
        was_critical = self.battery_critical
        self.battery_critical = bool(msg.is_critical)
        if self.battery_critical and not was_critical:
            self.get_logger().warn(
                f"battery critical ({self.battery_percent:.1f}%) — RTB will engage.")

    def _tick(self) -> None:
        # 1. Hard overrides (highest priority): ESTOP / MANUAL / MISSION_COMPLETE.
        if self.estop:
            self._enter_phase("MISSION_COMPLETE", reason="ESTOP")
            self.cmd_pub.publish(Twist())
            return

        if self.mode == "MANUAL":
            # Phase stays where it was (for telemetry) but we don't transition.
            # Forward fresh teleop cmd; stale -> zero twist.
            stale_sec = float(self.get_parameter("teleop_stale_sec").value)
            age = (self.get_clock().now().nanoseconds - self.last_teleop_ns) / 1e9 \
                if self.last_teleop_ns else float("inf")
            if self.last_teleop_cmd is not None and age < stale_sec:
                self.cmd_pub.publish(self.last_teleop_cmd)
            else:
                self.cmd_pub.publish(Twist())
            return

        if self.phase == "MISSION_COMPLETE":
            self.cmd_pub.publish(Twist())
            return

        # During arm action, hold rover stopped and freeze FSM transitions.
        if self.arm_in_flight:
            self.cmd_pub.publish(Twist())
            return

        # 2. Mission termination gates (AUTO only): collection goal / battery.
        goal = int(self.get_parameter("collection_goal").value)
        if self.phase != "RETURN_TO_BASE" and (
            (goal > 0 and self.collected_count >= goal)
            or self.battery_critical
        ):
            why = (f"goal reached ({self.collected_count}/{goal})"
                   if self.collected_count >= goal
                   else f"battery critical ({self.battery_percent:.1f}%)")
            self._enter_phase("RETURN_TO_BASE", reason=why)

        # 3. RTB drives toward basecamp; arrival -> MISSION_COMPLETE.
        if self.phase == "RETURN_TO_BASE":
            self._drive_to_basecamp()
            return

        now_ns = self.get_clock().now().nanoseconds
        stale_ns = int(float(self.get_parameter("detection_stale_sec").value) * 1e9)
        det_fresh = (self.target is not None) and (now_ns - self.last_det_stamp_ns < stale_ns)

        # Target lock-on: keep approaching cached target if detection briefly
        # lost, until approach_lock_timeout_sec elapses.
        lock_timeout_ns = int(float(self.get_parameter("approach_lock_timeout_sec").value) * 1e9)
        if not det_fresh and self.lock_target is not None:
            if now_ns - self.lock_started_ns < lock_timeout_ns:
                # use lock as virtual target
                self.target = self.lock_target
                det_fresh = True
            else:
                self.lock_target = None
                self.lock_started_ns = 0

        engage_d = float(self.get_parameter("approach_engage_dist_m").value)
        default_stop_d = float(self.get_parameter("approach_stop_dist_m").value)
        target_cls = self.target.class_name if self.target is not None else ""
        stop_d = _stop_dist_for(target_cls, default_stop_d)
        resume_d = stop_d + float(self.get_parameter("approach_resume_buffer_m").value)

        dist = self._distance_to_target() if det_fresh else float("inf")

        prev_phase = self.phase
        if not det_fresh:
            self.phase = "EXPLORE"
        elif prev_phase == "PICK_READY":
            # Hysteresis: stay PICK_READY until rover drifts past resume_d
            self.phase = "PICK_READY" if dist <= resume_d else "APPROACH"
        else:
            if dist <= stop_d:
                self.phase = "PICK_READY"
            elif dist <= engage_d:
                self.phase = "APPROACH"
            else:
                self.phase = "EXPLORE"

        if self.phase != prev_phase:
            self.get_logger().info(
                f"phase: {prev_phase} -> {self.phase}"
                + (f" (target {self.target.class_name} world=("
                   f"{self.target.world_position.x:.2f},"
                   f"{self.target.world_position.y:.2f}), dist={dist:.2f}m, "
                   f"stop_d={stop_d:.2f}m)"
                   if det_fresh else ""))
            if self.phase != "APPROACH":
                self.waypoints = []
                self.wp_idx = 0
                self.last_plan_target = None
            if self.phase == "APPROACH" and prev_phase != "APPROACH":
                # Lock onto this target's coordinates for the duration of APPROACH
                self.lock_target = self.target
                self.lock_started_ns = now_ns
            if self.phase != "APPROACH" and self.phase != "PICK_READY":
                self.lock_target = None
                self.lock_started_ns = 0
            if self.phase == "PICK_READY" and prev_phase != "PICK_READY":
                self._send_arm_goal()
            if self.phase == "EXPLORE" and prev_phase != "EXPLORE":
                # Drop stale coverage cmd, ask coverage to replan from the new
                # rover position, and start a brief cooldown so the replan can
                # complete before we start passing through its cmd_vel.
                self.last_coverage_cmd = None
                self.last_coverage_cmd_ns = 0
                self.explore_entry_ns = now_ns
                self.replan_pub.publish(Empty())

        if self.phase == "EXPLORE":
            # Hold zero twist briefly so coverage can publish a fresh path
            # for the post-APPROACH/PICK rover position.
            resume_delay_ns = int(
                float(self.get_parameter("explore_resume_delay_sec").value) * 1e9)
            in_resume_cooldown = (
                self.explore_entry_ns > 0
                and (now_ns - self.explore_entry_ns) < resume_delay_ns)
            stale_sec = float(self.get_parameter("coverage_cmd_stale_sec").value)
            cmd_age = (now_ns - self.last_coverage_cmd_ns) / 1e9 \
                if self.last_coverage_cmd_ns else float("inf")
            if in_resume_cooldown:
                self.cmd_pub.publish(Twist())
            elif self.last_coverage_cmd is not None and cmd_age < stale_sec:
                self.cmd_pub.publish(self.last_coverage_cmd)
            else:
                self.cmd_pub.publish(Twist())
            # EXPLORE: supervisor path/target 비움 — UI 가 coverage 의 path/
            # marker 만 보이게.
            self.waypoints = []
            self._publish_supervisor_viz(target_xy=None)
        elif self.phase == "APPROACH":
            self.cmd_pub.publish(self._approach_twist(dist))
            # _maybe_replan 후 waypoints + target 발행.
            tgt = (self.target.world_position.x, self.target.world_position.y) \
                  if self.target is not None else None
            self._publish_supervisor_viz(target_xy=tgt)
        else:  # PICK_READY
            self.cmd_pub.publish(Twist())
            tgt = (self.target.world_position.x, self.target.world_position.y) \
                  if self.target is not None else None
            self._publish_supervisor_viz(target_xy=tgt)

    def _distance_to_target(self) -> float:
        if self.target is None or self.last_odom is None:
            return float("inf")
        rx = self.last_odom.pose.pose.position.x
        ry = self.last_odom.pose.pose.position.y
        tx = self.target.world_position.x
        ty = self.target.world_position.y
        return math.hypot(tx - rx, ty - ry)

    def _approach_twist(self, dist: float) -> Twist:
        t = Twist()
        if self.target is None or self.last_odom is None:
            return t

        rx = self.last_odom.pose.pose.position.x
        ry = self.last_odom.pose.pose.position.y
        tx = self.target.world_position.x
        ty = self.target.world_position.y

        self._maybe_replan(rx, ry, tx, ty)

        if self.waypoints:
            wp_reach = float(self.get_parameter("waypoint_reach_dist_m").value)
            while self.wp_idx < len(self.waypoints) - 1:
                wx, wy = self.waypoints[self.wp_idx]
                if math.hypot(wx - rx, wy - ry) < wp_reach:
                    self.wp_idx += 1
                else:
                    break
            goal_x, goal_y = self.waypoints[self.wp_idx]
        else:
            # A* 실패 — 이 mineral 까지 obstacle 회피 path 없음. 직진하면
            # obstacle 통과 위험. 정지 + target/lock 폐기로 다음 tick 에
            # det_fresh=False → 자동 EXPLORE 복귀. 같은 mineral 이 다시
            # 감지되면 rover 위치가 바뀐 상태에서 재시도 가능.
            self.get_logger().warn(
                f"APPROACH A* failed for target ({tx:.2f},{ty:.2f}) — "
                f"dropping target, returning to EXPLORE",
                throttle_duration_sec=2.0)
            self.target = None
            self.lock_target = None
            self.last_det_stamp_ns = 0
            return Twist()

        q = self.last_odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        bearing = math.atan2(goal_y - ry, goal_x - rx)
        heading_err = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))

        creep_d = float(self.get_parameter("approach_stop_dist_m").value) * 2.0
        if dist < creep_d:
            lin = float(self.get_parameter("approach_creep_speed").value)
        else:
            lin = float(self.get_parameter("approach_lin_speed").value)
        if abs(heading_err) > 0.4:
            lin *= 0.3
        t.linear.x = lin
        t.angular.z = float(self.get_parameter("steer_gain").value) * heading_err
        max_ang = 1.2
        if t.angular.z > max_ang:
            t.angular.z = max_ang
        elif t.angular.z < -max_ang:
            t.angular.z = -max_ang
        return t

    def _maybe_replan(self, rx: float, ry: float, tx: float, ty: float) -> None:
        if self.ogrid is None:
            return
        need_replan = (
            not self.waypoints
            or self.last_plan_target is None
            or math.hypot(tx - self.last_plan_target[0],
                          ty - self.last_plan_target[1])
                > float(self.get_parameter("path_replan_target_delta_m").value)
        )
        if not need_replan:
            return

        start = self.ogrid.world_to_cell(rx, ry, clip=True)
        goal = self.ogrid.world_to_cell(tx, ty, clip=True)
        if self.ogrid.grid[start] == 1:
            start = self._nearest_free(start)
        if goal is not None and self.ogrid.grid[goal] == 1:
            goal = self._nearest_free(goal)

        # 1차 시도 — 표준 grid 로.
        path = astar(self.ogrid.grid, start, goal) if start and goal else None

        # 2차 시도 — 실패 시 시작 cell 주변 5×5 일시 free 처리 후 재시도.
        # mission_fsm._free_start_grid 와 동일 패턴. rover 가 PICK 직후
        # inflated obstacle 영역에 있을 때 발동. _nearest_free 의 12-cell
        # 검색이 부족한 케이스 보강.
        if not path and start and goal:
            relaxed = self.ogrid.grid.copy()
            si, sj = start
            for di in range(-2, 3):
                for dj in range(-2, 3):
                    ii, jj = si + di, sj + dj
                    if self.ogrid.in_bounds(ii, jj):
                        relaxed[ii, jj] = 0
            path = astar(relaxed, start, goal)
            if path:
                self.get_logger().warn(
                    f"A* recovered with relaxed start cell "
                    f"(rover was in inflated obstacle area)")

        if not path:
            self.get_logger().warn(
                f"A* no path: start={start} goal={goal} target=({tx:.2f},{ty:.2f})",
                throttle_duration_sec=2.0)
            self.waypoints = []
            self.last_plan_target = (tx, ty)
            return

        path = simplify_path(self.ogrid.grid, path)
        self.waypoints = [self.ogrid.cell_to_world(i, j) for (i, j) in path]
        self.wp_idx = 1 if len(self.waypoints) > 1 else 0
        self.last_plan_target = (tx, ty)
        self.get_logger().info(
            f"A* replanned: {len(self.waypoints)} waypoints "
            f"to ({tx:.2f},{ty:.2f})")

    def _send_arm_goal(self) -> None:
        if self.arm_client is None or self.target is None or self.arm_in_flight:
            return
        if not self.arm_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                "arm action server not available — skipping pick. Will retry on next PICK_READY.",
                throttle_duration_sec=5.0)
            return
        goal = ExecuteArmTask.Goal()
        goal.command = "pick_mineral"
        goal.target_id = str(self.target.mineral_id)
        goal.target_x = float(self.target.world_position.x)
        goal.target_y = float(self.target.world_position.y)
        goal.target_z = float(self.target.world_position.z)
        goal.metadata = self.target.class_name
        self.arm_in_flight = True
        self._pending_pick_world = (goal.target_x, goal.target_y)
        self._pending_pick_class = self.target.class_name
        future = self.arm_client.send_goal_async(goal)
        future.add_done_callback(self._on_arm_goal_accepted)
        self.get_logger().info(
            f"arm action sent: pick_mineral target_id={goal.target_id} "
            f"class={goal.metadata} xyz=({goal.target_x:.2f},{goal.target_y:.2f},"
            f"{goal.target_z:.2f})")

    def _on_arm_goal_accepted(self, future) -> None:
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f"arm goal future failed: {e}")
            self.arm_in_flight = False
            return
        if not handle.accepted:
            self.get_logger().warn("arm goal rejected by server")
            self.arm_in_flight = False
            return
        handle.get_result_async().add_done_callback(self._on_arm_result)

    def _on_arm_result(self, future) -> None:
        try:
            result = future.result().result
            ok = bool(result.success)
            msg_text = result.message
        except Exception as e:
            ok, msg_text = False, f"exception: {e}"
        self.arm_in_flight = False
        self.last_pick_done_ns = self.get_clock().now().nanoseconds
        self.last_pick_world = getattr(self, "_pending_pick_world", None)
        if ok:
            self.collected_count += 1
            cap = int(self.get_parameter("cargo_capacity").value)
            self.cargo_count = min(self.collected_count, cap) if cap > 0 else self.collected_count
            # 종류별 카운트 — arm action 의 metadata 에 class_name 이 들어있음.
            picked_class = (self.target.class_name if self.target is not None
                            else getattr(self, "_pending_pick_class", ""))
            if picked_class in self.collected_by_class:
                self.collected_by_class[picked_class] += 1
            # 직전 실패 메시지는 깨끗하게 — 한 번 실패가 영원히 화면에 박혀
            # 있는 현상 방지. 다음 실패가 발생하면 그때 다시 채워짐.
            self.last_error = ""
        else:
            self.last_error = f"arm failed: {msg_text}"
        # Drop the current target so PICK_READY exits and EXPLORE resumes.
        self.target = None
        self.last_det_stamp_ns = 0
        goal = int(self.get_parameter("collection_goal").value)
        self.get_logger().info(
            f"arm action done success={ok} message={msg_text!r} — "
            f"collected={self.collected_count}/{goal} — cooldown engaged")

    def _enter_phase(self, new_phase: str, *, reason: str = "") -> None:
        if self.phase == new_phase:
            return
        prev = self.phase
        self.phase = new_phase
        # Stale path 폐기 — EXPLORE 로 복귀할 때처럼 RTB 진입 시에도 이전 phase
        # (APPROACH/PICK_READY) 의 waypoint 가 남아있으면 rover 의 새 위치
        # 기준 obstacle 회피 path 가 안 그려져서 일직선으로 가는 사고가 남.
        # 다음 _drive_to_basecamp tick 에서 fresh A* 가 자동 실행됨.
        if new_phase in ("MISSION_COMPLETE", "EXPLORE", "RETURN_TO_BASE"):
            self.waypoints = []
            self.wp_idx = 0
            self.last_plan_target = None
        suffix = f" ({reason})" if reason else ""
        self.get_logger().info(f"phase: {prev} -> {new_phase}{suffix}")

    def _drive_to_basecamp(self) -> None:
        if self.last_odom is None:
            self.cmd_pub.publish(Twist())
            return
        bx = float(self.get_parameter("basecamp_x").value)
        by = float(self.get_parameter("basecamp_y").value)
        rx = self.last_odom.pose.pose.position.x
        ry = self.last_odom.pose.pose.position.y
        dist_to_center = math.hypot(bx - rx, by - ry)

        arrive_r = float(self.get_parameter("basecamp_arrive_radius_m").value)
        if dist_to_center <= arrive_r:
            self._enter_phase("MISSION_COMPLETE",
                              reason=f"basecamp reached at ({rx:.2f},{ry:.2f})")
            self.cmd_pub.publish(Twist())
            return

        # Basecamp 중심 (0,0) 자체가 obstacle 영역 → goal nudge 실패해 A* fail.
        # rover 에서 basecamp 향한 방향의 fringe 점을 target 으로 사용.
        fringe_r = float(self.get_parameter("basecamp_fringe_radius_m").value)
        if dist_to_center > fringe_r:
            ux = (bx - rx) / dist_to_center
            uy = (by - ry) / dist_to_center
            tx = bx - ux * fringe_r
            ty = by - uy * fringe_r
        else:
            tx, ty = bx, by

        # APPROACH 와 동일하게 A* 한 번 만들어두면 다음부터 같은 target 이라
        # replan 안 함. RTB 진입 시 _enter_phase 에서 waypoints/last_plan_target
        # 을 비웠으므로 첫 호출에서 fresh plan 생성.
        self._maybe_replan(rx, ry, tx, ty)

        if not self.waypoints:
            # A* 실패 — basecamp 까지 obstacle 회피 path 없음. 직진하면
            # obstacle 통과 위험. 자리에서 MISSION_COMPLETE 로 종료.
            self._enter_phase(
                "MISSION_COMPLETE",
                reason=f"RTB A* failed at ({rx:.2f},{ry:.2f}) — stopping in place")
            self.last_error = "RTB unreachable"
            self.cmd_pub.publish(Twist())
            return

        wp_reach = float(self.get_parameter("waypoint_reach_dist_m").value)
        while self.wp_idx < len(self.waypoints) - 1:
            wx, wy = self.waypoints[self.wp_idx]
            if math.hypot(wx - rx, wy - ry) < wp_reach:
                self.wp_idx += 1
            else:
                break
        gx, gy = self.waypoints[self.wp_idx]
        # UI 표시 — fringe target 을 분홍 별로.
        self._publish_supervisor_viz(target_xy=(tx, ty))

        q = self.last_odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        bearing = math.atan2(gy - ry, gx - rx)
        heading_err = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))

        t = Twist()
        lin = float(self.get_parameter("rtb_lin_speed").value)
        if abs(heading_err) > 0.4:
            lin *= 0.3
        t.linear.x = lin
        t.angular.z = max(-1.2, min(1.2,
            float(self.get_parameter("steer_gain").value) * heading_err))
        self.cmd_pub.publish(t)

    def _publish_supervisor_viz(self, target_xy=None) -> None:
        """Supervisor 의 현재 path/target 을 UI 용 토픽으로 발행.
        target_xy: (tx, ty) 또는 None. None 이면 target marker 안 그림."""
        stamp = self.get_clock().now().to_msg()
        # Path
        msg = Path()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        for wx, wy in (self.waypoints or []):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.supervisor_path_pub.publish(msg)
        # Target — None 이면 NaN 으로 보내서 dashboard 가 hide.
        tgt = PointStamped()
        tgt.header.stamp = stamp
        tgt.header.frame_id = "map"
        if target_xy is None:
            tgt.point.x = float("nan")
            tgt.point.y = float("nan")
        else:
            tgt.point.x = float(target_xy[0])
            tgt.point.y = float(target_xy[1])
        self.supervisor_target_pub.publish(tgt)

    def _publish_state(self) -> None:
        s = MissionState()
        s.state = self.phase if self.mode == "AUTO" else f"MANUAL/{self.phase}"
        s.previous_state = ""
        s.battery_percent = float(self.battery_percent)
        s.low_battery = bool(self.battery_low)
        s.critical_battery = bool(self.battery_critical)
        s.cargo_count = int(self.cargo_count)
        s.cargo_capacity = int(self.get_parameter("cargo_capacity").value)
        s.collected_count = int(self.collected_count)
        s.collection_goal = int(self.get_parameter("collection_goal").value)
        s.collected_blue   = int(self.collected_by_class.get("blue_mineral", 0))
        s.collected_yellow = int(self.collected_by_class.get("yellow_mineral", 0))
        s.collected_green  = int(self.collected_by_class.get("green_gas", 0))
        s.active_task = ("picking" if self.arm_in_flight
                         else "manual" if self.mode == "MANUAL"
                         else self.phase.lower())
        s.last_error = self.last_error
        self.state_pub.publish(s)

    def _nearest_free(self, cell: Tuple[int, int]) -> Tuple[int, int]:
        # search radius 25 cell (= 5m at cell_size=0.2). 이전 12 cell (2.4m) 은
        # basecamp 같은 큰 obstacle 영역 옆에서 free 못 찾는 케이스가 있었음.
        i0, j0 = cell
        for r in range(1, 25):
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    if max(abs(di), abs(dj)) != r:
                        continue
                    ni, nj = i0 + di, j0 + dj
                    if self.ogrid.in_bounds(ni, nj) and self.ogrid.grid[ni, nj] == 0:
                        return (ni, nj)
        return cell

    def _log_phase(self) -> None:
        s = String()
        s.data = self.phase
        self.phase_pub.publish(s)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish zero twist a few times so the rover does not keep moving
        # after shutdown. Wrap in try/except: under SIGINT the publisher's
        # context may already be invalid by the time we get here.
        for _ in range(5):
            try:
                node.cmd_pub.publish(Twist())
            except Exception:
                break
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
