"""Mission manager — supervisor level FSM.

Pattern: cmd_vel mux that never touches coverage_node source.
- EXPLORE: pass-through from /coverage/cmd_vel_raw to /cmd_vel
- APPROACH: A* way-point pursuit toward best target (mineral or gas).
  Uses isaac_drive's ObstacleGrid + astar (no source modification — just
  imports the same algorithm coverage_node uses).
- PICK_READY: stop within stop_distance (Phase 3 hooks arm trigger here)
  hysteresis: APPROACH->PICK_READY at stop_dist, PICK_READY->APPROACH at
  stop_dist + resume_buffer to prevent flapping.

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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Empty, String

from isaac_interfaces.action import ExecuteArmTask
from isaac_interfaces.msg import Detection, DetectionArray
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
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("odom_topic", "/ground_truth/odom")
        self.declare_parameter("phase_topic", "/mission/phase")
        self.declare_parameter("min_confidence", 0.6)
        self.declare_parameter("terrain_dir", _default_terrain_dir())
        self.declare_parameter("path_cell_size", 0.2)
        self.declare_parameter("path_robot_radius", 0.7)
        self.declare_parameter("path_replan_target_delta_m", 0.4)
        self.declare_parameter("waypoint_reach_dist_m", 0.4)

        self.phase = "EXPLORE"
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
                f"terrain load FAILED ({e}) — APPROACH will fall back to straight-line P-control")

        if bool(self.get_parameter("enable_arm_action").value):
            self.arm_client = ActionClient(
                self, ExecuteArmTask, str(self.get_parameter("arm_action_name").value))

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.phase_pub = self.create_publisher(
            String, str(self.get_parameter("phase_topic").value), 10)
        # Tell coverage_node to drop its stale DRIVE path and replan from
        # the rover's new post-APPROACH/PICK position.
        self.replan_pub = self.create_publisher(
            Empty, "/coverage/replan_request", 10)

        self.create_subscription(
            DetectionArray, str(self.get_parameter("detections_topic").value),
            self._on_detections, SENSOR_QOS)
        self.create_subscription(
            Twist, str(self.get_parameter("coverage_cmd_topic").value),
            self._on_coverage_cmd, 10)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self._on_odom, SENSOR_QOS)

        self.create_timer(0.05, self._tick)
        self.create_timer(
            float(self.get_parameter("phase_log_period_sec").value), self._log_phase)
        self.get_logger().info(
            "mission_manager_node ready. EXPLORE pass-through active.")

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

    def _tick(self) -> None:
        # During arm action, hold rover stopped and freeze FSM transitions.
        if self.arm_in_flight:
            self.cmd_pub.publish(Twist())
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
        elif self.phase == "APPROACH":
            self.cmd_pub.publish(self._approach_twist(dist))
        else:  # PICK_READY
            self.cmd_pub.publish(Twist())

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
            goal_x, goal_y = tx, ty

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
        # If the start or goal cell itself is blocked (mineral sitting on
        # an inflated obstacle, or rover anchored on edge), nudge to the
        # nearest free cell for a best-effort plan.
        if self.ogrid.grid[start] == 1:
            start = self._nearest_free(start)
        if goal is not None and self.ogrid.grid[goal] == 1:
            goal = self._nearest_free(goal)

        path = astar(self.ogrid.grid, start, goal) if start and goal else None
        if not path:
            self.get_logger().warn(
                f"A* no path: start={start} goal={goal} target=({tx:.2f},{ty:.2f}) "
                f"— falling back to straight-line",
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
        # Drop the current target so PICK_READY exits and EXPLORE resumes.
        self.target = None
        self.last_det_stamp_ns = 0
        self.get_logger().info(
            f"arm action done success={ok} message={msg_text!r} — cooldown engaged")

    def _nearest_free(self, cell: Tuple[int, int]) -> Tuple[int, int]:
        i0, j0 = cell
        for r in range(1, 12):
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
