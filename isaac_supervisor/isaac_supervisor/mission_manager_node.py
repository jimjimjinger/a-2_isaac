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
from geometry_msgs.msg import PointStamped, Twist
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
    "blue_mineral":   0.80,
    "yellow_mineral": 0.85,
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

        # ── 다중 rover mineral claim 협조 ──
        # 두 rover 가 동일 mineral 노리지 않도록. /mineral_claims 공유 토픽으로
        # 각자 현재 target XY 를 publish, 다른 rover 의 claim 안 (radius) mineral
        # 은 후보에서 제외. enable_mineral_claim=False (default) 시 비활성 = 단일
        # rover 시연 호환.
        self.declare_parameter("enable_mineral_claim", False)
        self.declare_parameter("claim_topic", "/mineral_claims")
        self.declare_parameter("claim_skip_radius_m", 1.5)
        self.declare_parameter("claim_ttl_sec", 5.0)
        self.declare_parameter("claim_publish_period_sec", 0.5)
        # 비우면 self.get_namespace() 사용 → "/rover_1" → "rover_1"
        self.declare_parameter("claim_rover_id", "")

        # ── 다중 rover 동적 충돌 회피 (Phase C) ──
        # 각자 자신 odom 을 /rover_positions 에 publish (frame_id=rover_id).
        # 다른 rover 위치를 A* 의 obstacle_grid 에 inflate 해서 자연스러운 우회.
        self.declare_parameter("enable_rover_avoid", False)
        self.declare_parameter("rover_positions_topic", "/rover_positions")
        self.declare_parameter("rover_position_publish_period_sec", 0.2)
        self.declare_parameter("rover_position_ttl_sec", 2.0)
        # A* obstacle 으로 박을 다른 rover 주변 반지름 (m)
        self.declare_parameter("rover_avoid_radius_m", 1.2)
        # 다른 rover 가 N m 이상 움직였으면 replan 강제
        self.declare_parameter("rover_replan_trigger_m", 0.8)

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
        # Mineral claim — other_claims[rover_id] = (x, y, stamp_ns)
        self.other_claims: dict = {}
        # 자신 식별자 — param 또는 namespace
        _rid = str(self.get_parameter("claim_rover_id").value).strip()
        if not _rid:
            _rid = self.get_namespace().strip("/") or "rover_solo"
        self._my_rover_id: str = _rid

        # other_rovers[rover_id] = (x, y, stamp_ns) — 동적 obstacle 후보
        self.other_rovers: dict = {}
        # replan trigger 추적 — 마지막 plan 시 다른 rover 위치
        self._last_plan_other_rovers: dict = {}

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
            Empty, "coverage/replan_request", 10)

        self.create_subscription(
            DetectionArray, str(self.get_parameter("detections_topic").value),
            self._on_detections, SENSOR_QOS)
        self.create_subscription(
            Twist, str(self.get_parameter("coverage_cmd_topic").value),
            self._on_coverage_cmd, 10)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self._on_odom, SENSOR_QOS)

        # Rover 위치 공유 pub/sub (절대 토픽 — A* 동적 obstacle 입력)
        if bool(self.get_parameter("enable_rover_avoid").value):
            pos_topic = str(self.get_parameter("rover_positions_topic").value)
            self.rover_pos_pub = self.create_publisher(
                PointStamped, pos_topic, 10)
            self.create_subscription(
                PointStamped, pos_topic, self._on_rover_position, 10)
            self.create_timer(
                float(self.get_parameter(
                    "rover_position_publish_period_sec").value),
                self._publish_rover_position)
            self.get_logger().info(
                f"rover avoidance 활성 — id={self._my_rover_id} "
                f"topic={pos_topic} inflate="
                f"{float(self.get_parameter('rover_avoid_radius_m').value):.2f}m")
        else:
            self.rover_pos_pub = None

        # Mineral claim pub/sub (절대 토픽 — namespace 무관 공유)
        if bool(self.get_parameter("enable_mineral_claim").value):
            claim_topic = str(self.get_parameter("claim_topic").value)
            self.claim_pub = self.create_publisher(PointStamped, claim_topic, 10)
            self.create_subscription(
                PointStamped, claim_topic, self._on_claim, 10)
            self.create_timer(
                float(self.get_parameter("claim_publish_period_sec").value),
                self._publish_claim)
            self.get_logger().info(
                f"mineral claim 협조 활성 — id={self._my_rover_id} "
                f"topic={claim_topic} radius="
                f"{float(self.get_parameter('claim_skip_radius_m').value):.2f}m "
                f"ttl={float(self.get_parameter('claim_ttl_sec').value):.1f}s")
        else:
            self.claim_pub = None

        self.create_timer(0.05, self._tick)
        self.create_timer(
            float(self.get_parameter("phase_log_period_sec").value), self._log_phase)
        self.get_logger().info(
            "mission_manager_node ready. EXPLORE pass-through active.")

    def _on_claim(self, msg: PointStamped) -> None:
        """다른 rover 의 mineral claim 수신. frame_id == rover_id."""
        rid = (msg.header.frame_id or "").strip()
        if not rid or rid == self._my_rover_id:
            return  # 자기 자신 무시
        now_ns = self.get_clock().now().nanoseconds
        self.other_claims[rid] = (
            float(msg.point.x), float(msg.point.y), now_ns)

    def _publish_claim(self) -> None:
        """phase 가 APPROACH/PICK_READY 일 때 자신 target XY publish."""
        if self.claim_pub is None:
            return
        if self.phase not in ("APPROACH", "PICK_READY"):
            return
        tgt = self.target if self.target is not None else self.lock_target
        if tgt is None:
            return
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._my_rover_id
        msg.point.x = float(tgt.world_position.x)
        msg.point.y = float(tgt.world_position.y)
        msg.point.z = 0.0
        self.claim_pub.publish(msg)

    def _on_rover_position(self, msg: PointStamped) -> None:
        """다른 rover 의 odom 위치 수신. frame_id == rover_id."""
        rid = (msg.header.frame_id or "").strip()
        if not rid or rid == self._my_rover_id:
            return
        now_ns = self.get_clock().now().nanoseconds
        self.other_rovers[rid] = (
            float(msg.point.x), float(msg.point.y), now_ns)

    def _publish_rover_position(self) -> None:
        """자신 odom XY 를 /rover_positions 에 publish (frame_id=self id)."""
        if self.rover_pos_pub is None or self.last_odom is None:
            return
        p = self.last_odom.pose.pose.position
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._my_rover_id
        msg.point.x = float(p.x)
        msg.point.y = float(p.y)
        msg.point.z = 0.0
        self.rover_pos_pub.publish(msg)

    def _other_rovers_alive(self) -> dict:
        """TTL 안 살아있는 다른 rover 만 반환 {rid: (x, y)}."""
        ttl_ns = int(float(
            self.get_parameter("rover_position_ttl_sec").value) * 1e9)
        now_ns = self.get_clock().now().nanoseconds
        alive = {}
        for rid, (rx, ry, ts) in list(self.other_rovers.items()):
            if now_ns - ts > ttl_ns:
                self.other_rovers.pop(rid, None)
                continue
            alive[rid] = (rx, ry)
        return alive

    def _build_dynamic_grid(self):
        """static obstacle_grid + 다른 rover 위치를 inflate 한 snapshot grid.

        다른 rover 가 없거나 enable_rover_avoid=False 면 static grid 그대로 반환.
        """
        if self.ogrid is None:
            return None
        if self.rover_pos_pub is None:
            return self.ogrid.grid
        others = self._other_rovers_alive()
        if not others:
            return self.ogrid.grid
        import numpy as np
        g = self.ogrid.grid.copy()
        r_m = float(self.get_parameter("rover_avoid_radius_m").value)
        cells_r = max(1, int(math.ceil(r_m / self.ogrid.cell_size)))
        cells_r2 = cells_r * cells_r
        for rid, (rx, ry) in others.items():
            i0, j0 = self.ogrid.world_to_cell(rx, ry, clip=True)
            for di in range(-cells_r, cells_r + 1):
                for dj in range(-cells_r, cells_r + 1):
                    if di * di + dj * dj > cells_r2:
                        continue
                    ni, nj = i0 + di, j0 + dj
                    if 0 <= ni < self.ogrid.rows and 0 <= nj < self.ogrid.cols:
                        g[ni, nj] = 1
        return g

    def _other_rovers_moved(self) -> bool:
        """다른 rover 가 마지막 plan 후 trigger_m 이상 움직였으면 replan 필요."""
        if self.rover_pos_pub is None:
            return False
        trigger = float(self.get_parameter("rover_replan_trigger_m").value)
        trigger2 = trigger * trigger
        cur = self._other_rovers_alive()
        prev = self._last_plan_other_rovers
        # 새로 나타난 / 사라진 rover 도 replan
        if set(cur.keys()) != set(prev.keys()):
            return True
        for rid, (cx, cy) in cur.items():
            px, py = prev.get(rid, (cx, cy))
            if (cx - px) ** 2 + (cy - py) ** 2 > trigger2:
                return True
        return False

    def _is_claimed_by_other(self, det_x: float, det_y: float) -> bool:
        """다른 rover 가 ttl 안에 claim 한 mineral 좌표 근처면 True."""
        if not self.other_claims:
            return False
        skip_r = float(self.get_parameter("claim_skip_radius_m").value)
        ttl_ns = int(float(self.get_parameter("claim_ttl_sec").value) * 1e9)
        now_ns = self.get_clock().now().nanoseconds
        skip_r2 = skip_r * skip_r
        for rid, (cx, cy, ts) in list(self.other_claims.items()):
            if now_ns - ts > ttl_ns:
                # expired — drop
                self.other_claims.pop(rid, None)
                continue
            dx = det_x - cx
            dy = det_y - cy
            if dx * dx + dy * dy < skip_r2:
                return True
        return False

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
            and not self._is_claimed_by_other(
                float(d.world_position.x), float(d.world_position.y))
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
            or self._other_rovers_moved()
        )
        if not need_replan:
            return

        # 다른 rover 위치를 박은 snapshot grid (없거나 비활성이면 static grid)
        plan_grid = self._build_dynamic_grid()

        start = self.ogrid.world_to_cell(rx, ry, clip=True)
        goal = self.ogrid.world_to_cell(tx, ty, clip=True)
        # If the start or goal cell itself is blocked (mineral sitting on
        # an inflated obstacle, or rover anchored on edge), nudge to the
        # nearest free cell for a best-effort plan.
        if plan_grid[start] == 1:
            start = self._nearest_free(start, plan_grid)
        if goal is not None and plan_grid[goal] == 1:
            goal = self._nearest_free(goal, plan_grid)

        path = astar(plan_grid, start, goal) if start and goal else None
        if not path:
            self.get_logger().warn(
                f"A* no path: start={start} goal={goal} target=({tx:.2f},{ty:.2f}) "
                f"— falling back to straight-line",
                throttle_duration_sec=2.0)
            self.waypoints = []
            self.last_plan_target = (tx, ty)
            return

        path = simplify_path(plan_grid, path)
        self.waypoints = [self.ogrid.cell_to_world(i, j) for (i, j) in path]
        self.wp_idx = 1 if len(self.waypoints) > 1 else 0
        self.last_plan_target = (tx, ty)
        # plan 시점 다른 rover 위치 snapshot (다음 replan trigger 비교용)
        self._last_plan_other_rovers = dict(self._other_rovers_alive())
        self.get_logger().info(
            f"A* replanned: {len(self.waypoints)} waypoints "
            f"to ({tx:.2f},{ty:.2f})"
            + (f" [avoid {len(self._last_plan_other_rovers)} rover(s)]"
               if self._last_plan_other_rovers else ""))

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

    def _nearest_free(self, cell: Tuple[int, int], grid=None) -> Tuple[int, int]:
        """`grid` 가 None 이면 static obstacle_grid 사용. dynamic plan_grid 도 가능."""
        if grid is None:
            grid = self.ogrid.grid
        i0, j0 = cell
        for r in range(1, 12):
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    if max(abs(di), abs(dj)) != r:
                        continue
                    ni, nj = i0 + di, j0 + dj
                    if self.ogrid.in_bounds(ni, nj) and grid[ni, nj] == 0:
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
