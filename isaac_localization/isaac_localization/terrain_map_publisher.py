#!/usr/bin/env python3

import json
import math
import os
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _resolve_a2_root(caller_file: str) -> Path:
    """팀 어느 머신에서도 작동하도록 a2_isaac 패키지 루트 해석.

    우선순위:
    1. A2_ISAAC_ROOT 환경변수 (팀원 별 setup.bash 에서 export 권장)
    2. 같은 colcon workspace 의 src/a2_isaac 가 caller 의 부모로 보이면 그것 사용
       (개발 시 install 된 site-packages 가 아니라 src 의 isaac_sim/assets 가
        최신이므로 이쪽을 우선)
    3. installed 위치 — caller 파일 기준 parents[2] (share/<pkg>/<module> 패턴)
    """
    env = os.environ.get("A2_ISAAC_ROOT")
    if env:
        return Path(env)
    caller_path = Path(caller_file).resolve()
    for parent in caller_path.parents:
        if parent.name == "a2_isaac" and (parent / "isaac_sim").exists():
            return parent
        candidate = parent / "src" / "a2_isaac"
        if (candidate / "isaac_sim").exists():
            return candidate
    return caller_path.parents[2]


class TerrainMapPublisher(Node):
    """Publish generated terrain assets as RViz-friendly map and markers."""

    def __init__(self) -> None:
        super().__init__("terrain_map_publisher")

        self.declare_parameter("terrain_id", "terrain_00001")
        self.declare_parameter("terrain_root", "")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("height_map_topic", "/terrain/height_map")
        self.declare_parameter("safety_map_topic", "/terrain/safety_map")
        self.declare_parameter("mineral_markers_topic", "/terrain/mineral_markers")
        self.declare_parameter("basecamp_marker_topic", "/terrain/basecamp_marker")
        self.declare_parameter("obstacle_markers_topic", "/terrain/obstacle_markers")
        self.declare_parameter("coverage_trail_topic", "/terrain/coverage_trail")
        self.declare_parameter("estimated_odom_topic", "/rover/estimated_odom")
        self.declare_parameter("estimated_marker_topic", "/rover/estimated_marker")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("marker_publish_hz", 2.0)
        self.declare_parameter("static_publish_hz", 0.5)
        self.declare_parameter("rover_marker_scale", 0.7)
        self.declare_parameter("safety_radius_m", 0.7)
        self.declare_parameter("obstacle_marker_stride", 6)
        self.declare_parameter("coverage_trail_radius_m", 1.2)
        self.declare_parameter("coverage_trail_resolution_m", 0.25)

        self.terrain_id = str(self.get_parameter("terrain_id").value)
        self.terrain_root = str(self.get_parameter("terrain_root").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.height_map_topic = str(self.get_parameter("height_map_topic").value)
        self.safety_map_topic = str(self.get_parameter("safety_map_topic").value)
        self.mineral_markers_topic = str(
            self.get_parameter("mineral_markers_topic").value
        )
        self.basecamp_marker_topic = str(
            self.get_parameter("basecamp_marker_topic").value
        )
        self.obstacle_markers_topic = str(
            self.get_parameter("obstacle_markers_topic").value
        )
        self.coverage_trail_topic = str(
            self.get_parameter("coverage_trail_topic").value
        )
        self.estimated_odom_topic = str(
            self.get_parameter("estimated_odom_topic").value
        )
        self.estimated_marker_topic = str(
            self.get_parameter("estimated_marker_topic").value
        )
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.marker_publish_hz = float(self.get_parameter("marker_publish_hz").value)
        self.static_publish_hz = float(self.get_parameter("static_publish_hz").value)
        self.rover_marker_scale = float(self.get_parameter("rover_marker_scale").value)
        self.safety_radius_m = float(self.get_parameter("safety_radius_m").value)
        self.obstacle_marker_stride = max(
            1,
            int(self.get_parameter("obstacle_marker_stride").value),
        )
        self.coverage_trail_radius_m = float(
            self.get_parameter("coverage_trail_radius_m").value
        )
        self.coverage_trail_resolution_m = max(
            0.05,
            float(self.get_parameter("coverage_trail_resolution_m").value),
        )

        self.terrain_dir = self.resolve_terrain_dir()
        self.meta = self.load_meta(self.terrain_dir / "meta.json")
        self.obstacle_grid = np.load(self.terrain_dir / "obstacle_grid.npy")
        self.heightmap = self.load_heightmap(self.terrain_dir / "heightmap.npy")

        self.resolution = float(self.meta.get("resolution_m", 0.05))
        origin = self.meta.get("origin", {})
        self.origin_x = float(origin.get("x", 0.0))
        self.origin_y = float(origin.get("y", 0.0))

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        live_qos = QoSProfile(depth=10)

        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, latched_qos)
        self.height_map_pub = self.create_publisher(
            OccupancyGrid,
            self.height_map_topic,
            latched_qos,
        )
        self.safety_map_pub = self.create_publisher(
            OccupancyGrid,
            self.safety_map_topic,
            latched_qos,
        )
        self.mineral_pub = self.create_publisher(
            MarkerArray,
            self.mineral_markers_topic,
            latched_qos,
        )
        self.basecamp_pub = self.create_publisher(
            MarkerArray,
            self.basecamp_marker_topic,
            latched_qos,
        )
        self.obstacle_marker_pub = self.create_publisher(
            MarkerArray,
            self.obstacle_markers_topic,
            latched_qos,
        )
        self.coverage_trail_pub = self.create_publisher(
            MarkerArray,
            self.coverage_trail_topic,
            live_qos,
        )
        self.rover_marker_pub = self.create_publisher(
            MarkerArray,
            self.estimated_marker_topic,
            live_qos,
        )

        self.latest_odom: Optional[Odometry] = None
        self.coverage_cells = set()
        self.last_coverage_cell: Optional[tuple[int, int]] = None
        self.create_subscription(
            Odometry,
            self.estimated_odom_topic,
            self._on_estimated_odom,
            20,
        )

        period = 1.0 / self.marker_publish_hz if self.marker_publish_hz > 0.0 else 0.5
        self.create_timer(period, self._publish_live_markers)
        static_period = (
            1.0 / self.static_publish_hz if self.static_publish_hz > 0.0 else 2.0
        )
        self.create_timer(static_period, self.publish_static)

        self.map_msg = self.build_occupancy_grid()
        self.height_map_msg = self.build_height_map()
        self.safety_map_msg = self.build_safety_map()
        self.mineral_markers = self.build_mineral_markers()
        self.basecamp_markers = self.build_basecamp_markers()
        self.obstacle_markers = self.build_obstacle_markers()

        self.publish_static()

        self.get_logger().info("Terrain Map Publisher initialized.")
        self.get_logger().info(f"Terrain dir      : {self.terrain_dir}")
        self.get_logger().info(f"Map topic        : {self.map_topic}")
        self.get_logger().info(f"Height map       : {self.height_map_topic}")
        self.get_logger().info(f"Safety map       : {self.safety_map_topic}")
        self.get_logger().info(f"Mineral markers  : {self.mineral_markers_topic}")
        self.get_logger().info(f"Basecamp marker  : {self.basecamp_marker_topic}")
        self.get_logger().info(f"Obstacle markers : {self.obstacle_markers_topic}")
        self.get_logger().info(f"Coverage trail   : {self.coverage_trail_topic}")
        self.get_logger().info(f"Estimated odom   : {self.estimated_odom_topic}")
        self.get_logger().info(f"Rover marker     : {self.estimated_marker_topic}")

    def resolve_terrain_dir(self) -> Path:
        if self.terrain_root:
            root = Path(self.terrain_root)
        else:
            a2_root = _resolve_a2_root(__file__)
            root = a2_root / "isaac_sim" / "assets" / "generated_terrains"

        terrain_id = self.terrain_id
        if terrain_id == "latest":
            terrain_id = self.resolve_latest_terrain_id(root)

        return root / terrain_id

    @staticmethod
    def resolve_latest_terrain_id(root: Path) -> str:
        index_path = root / "index.json"
        if index_path.exists():
            index = json.loads(index_path.read_text(encoding="utf-8"))
            terrains = index.get("terrains", [])
            if terrains:
                return str(terrains[-1]["id"])

        terrain_dirs = sorted(p.name for p in root.glob("terrain_*") if p.is_dir())
        if not terrain_dirs:
            raise FileNotFoundError(f"No terrain_* directories found in {root}")
        return terrain_dirs[-1]

    @staticmethod
    def load_meta(path: Path) -> Dict:
        if not path.exists():
            raise FileNotFoundError(f"Terrain meta.json not found: {path}")
        return json.loads(path.read_text(encoding="utf-8"))

    @staticmethod
    def load_heightmap(path: Path) -> Optional[np.ndarray]:
        if not path.exists():
            return None
        return np.load(path).astype(np.float32)

    def build_occupancy_grid(self) -> OccupancyGrid:
        grid = np.asarray(self.obstacle_grid)
        if grid.ndim != 2:
            raise ValueError(f"obstacle_grid.npy must be 2D, got {grid.shape}")

        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = self.resolution
        msg.info.width = int(grid.shape[1])
        msg.info.height = int(grid.shape[0])
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        occupancy = np.where(grid > 0, 100, 0).astype(np.int8)
        msg.data = occupancy.reshape(-1).astype(int).tolist()
        return msg

    def build_height_map(self) -> OccupancyGrid:
        if self.heightmap is None:
            grid = np.zeros_like(np.asarray(self.obstacle_grid), dtype=np.int8)
        else:
            hm = np.asarray(self.heightmap, dtype=np.float32)
            finite = np.isfinite(hm)
            if not finite.any():
                grid = np.zeros_like(hm, dtype=np.int8)
            else:
                lo, hi = np.percentile(hm[finite], [2.0, 98.0])
                span = max(float(hi - lo), 1e-6)
                grid = np.clip((hm - lo) / span * 100.0, 0.0, 100.0).astype(np.int8)

        msg = self.grid_message_from_array(grid)
        return msg

    def build_safety_map(self) -> OccupancyGrid:
        grid = np.asarray(self.obstacle_grid) > 0
        radius_cells = self.safety_radius_m / max(self.resolution, 1e-6)
        inflated = self.dilate_mask(grid, radius_cells)
        safety_only = inflated & ~grid
        values = np.full(grid.shape, -1, dtype=np.int8)
        values[safety_only] = 70
        msg = self.grid_message_from_array(values)
        return msg

    def grid_message_from_array(self, values: np.ndarray) -> OccupancyGrid:
        grid = np.asarray(values)
        if grid.ndim != 2:
            raise ValueError(f"grid must be 2D, got {grid.shape}")

        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = self.resolution
        msg.info.width = int(grid.shape[1])
        msg.info.height = int(grid.shape[0])
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.reshape(-1).astype(int).tolist()
        return msg

    @staticmethod
    def dilate_mask(mask: np.ndarray, radius_cells: float) -> np.ndarray:
        src = np.asarray(mask, dtype=bool)
        if radius_cells < 1.0:
            return src.copy()

        out = src.copy()
        height, width = src.shape
        r = int(math.ceil(radius_cells))
        r2 = radius_cells * radius_cells
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r2 or (dx == 0 and dy == 0):
                    continue
                sy_src = slice(max(0, -dy), height - max(0, dy))
                sy_dst = slice(max(0, dy), height - max(0, -dy))
                sx_src = slice(max(0, -dx), width - max(0, dx))
                sx_dst = slice(max(0, dx), width - max(0, -dx))
                out[sy_dst, sx_dst] |= src[sy_src, sx_src]
        return out

    def build_mineral_markers(self) -> MarkerArray:
        markers: List[Marker] = []
        minerals = self.meta.get("minerals", [])
        for idx, mineral in enumerate(minerals):
            position = mineral.get("position", {})
            mineral_type = str(mineral.get("type", "unknown"))
            mineral_id = int(mineral.get("id", idx + 1))

            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.ns = "minerals"
            sphere.id = mineral_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(position.get("x", 0.0))
            sphere.pose.position.y = float(position.get("y", 0.0))
            sphere.pose.position.z = float(position.get("z", 0.0)) + 0.9
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 1.25
            sphere.scale.y = 1.25
            sphere.scale.z = 1.25
            sphere.color = self.mineral_color(mineral_type)
            sphere.lifetime = Duration(seconds=0.0).to_msg()
            markers.append(sphere)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.ns = "mineral_labels"
            label.id = 1000 + mineral_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = sphere.pose.position.x
            label.pose.position.y = sphere.pose.position.y
            label.pose.position.z = sphere.pose.position.z + 1.15
            label.pose.orientation.w = 1.0
            label.scale.z = 0.75
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = f"M{mineral_id}:{mineral_type}"
            label.lifetime = Duration(seconds=0.0).to_msg()
            markers.append(label)

        return MarkerArray(markers=markers)

    def build_obstacle_markers(self) -> MarkerArray:
        grid = np.asarray(self.obstacle_grid) > 0
        stride = self.obstacle_marker_stride
        rows, cols = np.where(grid[::stride, ::stride])

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = "collision_obstacles"
        marker.id = 1
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(self.resolution * stride * 0.85, 0.12)
        marker.scale.y = max(self.resolution * stride * 0.85, 0.12)
        marker.scale.z = 0.55
        marker.color = ColorRGBA(r=1.0, g=0.1, b=0.0, a=0.95)
        marker.lifetime = Duration(seconds=0.0).to_msg()

        for row, col in zip(rows.tolist(), cols.tolist()):
            x = self.origin_x + (col * stride + 0.5) * self.resolution
            y = self.origin_y + (row * stride + 0.5) * self.resolution
            marker.points.append(Point(x=float(x), y=float(y), z=0.35))

        label = Marker()
        label.header.frame_id = self.frame_id
        label.ns = "collision_obstacles_label"
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = self.origin_x + 1.0
        label.pose.position.y = self.origin_y + 1.0
        label.pose.position.z = 1.5
        label.pose.orientation.w = 1.0
        label.scale.z = 0.55
        label.color = ColorRGBA(r=1.0, g=0.35, b=0.15, a=1.0)
        label.text = "collision obstacles"
        label.lifetime = Duration(seconds=0.0).to_msg()

        markers: List[Marker] = [marker, label]
        markers.extend(self.build_epic_obstacle_markers())
        return MarkerArray(markers=markers)

    def build_epic_obstacle_markers(self) -> List[Marker]:
        markers: List[Marker] = []
        scale_factor = 0.72
        epic_obstacles = self.meta.get("epic_obstacles", [])

        for idx, obstacle in enumerate(epic_obstacles):
            position = obstacle.get("position", {})
            footprint = obstacle.get("footprint_m", [3.0, 3.0])
            obstacle_type = str(obstacle.get("type", "obstacle"))
            yaw = float(obstacle.get("yaw", 0.0))
            x = float(position.get("x", 0.0))
            y = float(position.get("y", 0.0))
            color = self.epic_obstacle_color(obstacle_type)

            footprint_marker = Marker()
            footprint_marker.header.frame_id = self.frame_id
            footprint_marker.ns = "epic_obstacles"
            footprint_marker.id = 100 + idx
            footprint_marker.type = Marker.CUBE
            footprint_marker.action = Marker.ADD
            footprint_marker.pose.position.x = x
            footprint_marker.pose.position.y = y
            footprint_marker.pose.position.z = 0.9
            self.set_marker_yaw(footprint_marker, yaw)
            footprint_marker.scale.x = max(float(footprint[0]) * scale_factor, 1.0)
            footprint_marker.scale.y = max(float(footprint[1]) * scale_factor, 1.0)
            footprint_marker.scale.z = 0.35
            footprint_marker.color = color
            footprint_marker.lifetime = Duration(seconds=0.0).to_msg()
            markers.append(footprint_marker)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.ns = "epic_obstacle_labels"
            label.id = 200 + idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 2.2
            label.pose.orientation.w = 1.0
            label.scale.z = 0.58
            label.color = ColorRGBA(
                r=min(color.r + 0.18, 1.0),
                g=min(color.g + 0.18, 1.0),
                b=min(color.b + 0.18, 1.0),
                a=1.0,
            )
            label.text = self.epic_obstacle_label(obstacle_type)
            label.lifetime = Duration(seconds=0.0).to_msg()
            markers.append(label)

        return markers

    def build_basecamp_markers(self) -> MarkerArray:
        markers: List[Marker] = []
        basecamp = self.meta.get("basecamp", {})
        center = basecamp.get("center", {})
        radius = float(basecamp.get("radius", 3.0))

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = "basecamp"
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(center.get("x", 0.0))
        marker.pose.position.y = float(center.get("y", 0.0))
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = 0.5
        marker.color = ColorRGBA(r=1.0, g=0.68, b=0.0, a=0.88)
        marker.lifetime = Duration(seconds=0.0).to_msg()
        markers.append(marker)

        ring = Marker()
        ring.header.frame_id = self.frame_id
        ring.ns = "basecamp_outline"
        ring.id = 3
        ring.type = Marker.LINE_STRIP
        ring.action = Marker.ADD
        ring.pose.orientation.w = 1.0
        ring.scale.x = 0.16
        ring.color = ColorRGBA(r=1.0, g=0.95, b=0.15, a=1.0)
        for i in range(65):
            theta = 2.0 * math.pi * i / 64.0
            ring.points.append(
                Point(
                    x=marker.pose.position.x + radius * math.cos(theta),
                    y=marker.pose.position.y + radius * math.sin(theta),
                    z=0.7,
                )
            )
        ring.lifetime = Duration(seconds=0.0).to_msg()
        markers.append(ring)

        label = Marker()
        label.header.frame_id = self.frame_id
        label.ns = "basecamp_label"
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = marker.pose.position.x
        label.pose.position.y = marker.pose.position.y
        label.pose.position.z = 3.2
        label.pose.orientation.w = 1.0
        label.scale.z = 1.15
        label.color = ColorRGBA(r=1.0, g=0.95, b=0.2, a=1.0)
        label.text = "BASE CAMP"
        label.lifetime = Duration(seconds=0.0).to_msg()
        markers.append(label)

        return MarkerArray(markers=markers)

    def _on_estimated_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.reveal_coverage_at(
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def publish_static(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.map_msg.header.stamp = stamp
        self.height_map_msg.header.stamp = stamp
        self.safety_map_msg.header.stamp = stamp
        self.height_map_pub.publish(self.height_map_msg)
        self.map_pub.publish(self.map_msg)
        self.safety_map_pub.publish(self.safety_map_msg)
        self.stamp_markers(self.mineral_markers, stamp)
        self.stamp_markers(self.basecamp_markers, stamp)
        self.stamp_markers(self.obstacle_markers, stamp)
        self.mineral_pub.publish(self.mineral_markers)
        self.basecamp_pub.publish(self.basecamp_markers)
        self.obstacle_marker_pub.publish(self.obstacle_markers)

    def _publish_live_markers(self) -> None:
        if self.latest_odom is None:
            return
        self.coverage_trail_pub.publish(self.build_coverage_trail_marker())
        self.rover_marker_pub.publish(self.build_rover_marker(self.latest_odom))

    def reveal_coverage_at(self, x: float, y: float) -> None:
        res = self.coverage_trail_resolution_m
        cx = int(math.floor((x - self.origin_x) / res))
        cy = int(math.floor((y - self.origin_y) / res))
        cell = (cx, cy)
        if cell == self.last_coverage_cell:
            return
        self.last_coverage_cell = cell

        radius_cells = int(math.ceil(self.coverage_trail_radius_m / res))
        radius_sq = self.coverage_trail_radius_m * self.coverage_trail_radius_m
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                wx = self.origin_x + (cx + dx + 0.5) * res
                wy = self.origin_y + (cy + dy + 0.5) * res
                if (wx - x) * (wx - x) + (wy - y) * (wy - y) <= radius_sq:
                    self.coverage_cells.add((cx + dx, cy + dy))

    def build_coverage_trail_marker(self) -> MarkerArray:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "coverage_trail"
        marker.id = 1
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.coverage_trail_resolution_m * 0.96
        marker.scale.y = self.coverage_trail_resolution_m * 0.96
        marker.scale.z = 0.04
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.78)
        marker.lifetime = Duration(seconds=0.0).to_msg()

        for cx, cy in sorted(self.coverage_cells):
            marker.points.append(
                Point(
                    x=self.origin_x + (cx + 0.5) * self.coverage_trail_resolution_m,
                    y=self.origin_y + (cy + 0.5) * self.coverage_trail_resolution_m,
                    z=0.12,
                )
            )

        return MarkerArray(markers=[marker])

    def build_rover_marker(self, odom: Odometry) -> MarkerArray:
        pose = odom.pose.pose
        yaw = self.yaw_from_quaternion(pose.orientation)
        scale = self.rover_marker_scale

        body = Marker()
        body.header.frame_id = self.frame_id
        body.header.stamp = self.get_clock().now().to_msg()
        body.ns = "estimated_rover"
        body.id = 1
        body.type = Marker.ARROW
        body.action = Marker.ADD
        body.pose = pose
        body.pose.position.z = max(float(pose.position.z), 0.1) + 0.4
        body.scale.x = scale
        body.scale.y = scale * 0.25
        body.scale.z = scale * 0.25
        body.color = ColorRGBA(r=0.0, g=1.0, b=0.25, a=1.0)

        text = Marker()
        text.header = body.header
        text.ns = "estimated_rover_label"
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(pose.position.x)
        text.pose.position.y = float(pose.position.y)
        text.pose.position.z = body.pose.position.z + 0.6
        text.pose.orientation.w = 1.0
        text.scale.z = 0.35
        text.color = ColorRGBA(r=0.7, g=1.0, b=0.7, a=1.0)
        text.text = (
            f"estimated\nx={pose.position.x:.2f}, "
            f"y={pose.position.y:.2f}, yaw={yaw:.2f}"
        )

        return MarkerArray(markers=[body, text])

    @staticmethod
    def stamp_markers(markers: MarkerArray, stamp) -> None:
        for marker in markers.markers:
            marker.header.stamp = stamp

    @staticmethod
    def mineral_color(mineral_type: str) -> ColorRGBA:
        mineral_type = mineral_type.lower()
        if "blue" in mineral_type:
            return ColorRGBA(r=0.0, g=0.75, b=1.0, a=1.0)
        if "green" in mineral_type:
            return ColorRGBA(r=0.05, g=1.0, b=0.25, a=1.0)
        if "red" in mineral_type:
            return ColorRGBA(r=1.0, g=0.15, b=0.05, a=1.0)
        if "yellow" in mineral_type:
            return ColorRGBA(r=1.0, g=0.85, b=0.05, a=1.0)
        return ColorRGBA(r=0.8, g=0.8, b=0.8, a=1.0)

    @staticmethod
    def epic_obstacle_label(obstacle_type: str) -> str:
        labels = {
            "barracks": "Barracks",
            "battlecruiser": "Battlecruiser",
            "goliath": "Goliath",
            "scv": "SCV",
        }
        return labels.get(obstacle_type.lower(), obstacle_type.title())

    @staticmethod
    def epic_obstacle_color(obstacle_type: str) -> ColorRGBA:
        obstacle_type = obstacle_type.lower()
        if "battlecruiser" in obstacle_type:
            return ColorRGBA(r=0.20, g=0.45, b=1.0, a=0.82)
        if "barracks" in obstacle_type:
            return ColorRGBA(r=1.0, g=0.45, b=0.08, a=0.82)
        if "goliath" in obstacle_type:
            return ColorRGBA(r=0.95, g=0.18, b=0.95, a=0.82)
        if "scv" in obstacle_type:
            return ColorRGBA(r=0.0, g=0.90, b=0.78, a=0.82)
        return ColorRGBA(r=0.7, g=0.7, b=0.7, a=0.82)

    @staticmethod
    def set_marker_yaw(marker: Marker, yaw: float) -> None:
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(yaw * 0.5)
        marker.pose.orientation.w = math.cos(yaw * 0.5)

    @staticmethod
    def yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainMapPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
