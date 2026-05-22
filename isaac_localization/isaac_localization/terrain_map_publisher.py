#!/usr/bin/env python3

import json
import math
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

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


class TerrainMapPublisher(Node):
    """Publish generated terrain assets as RViz-friendly map and markers."""

    def __init__(self) -> None:
        super().__init__("terrain_map_publisher")

        self.declare_parameter("terrain_id", "terrain_00001")
        self.declare_parameter("terrain_root", "")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("mineral_markers_topic", "/terrain/mineral_markers")
        self.declare_parameter("basecamp_marker_topic", "/terrain/basecamp_marker")
        self.declare_parameter("estimated_odom_topic", "/rover/estimated_odom")
        self.declare_parameter("estimated_marker_topic", "/rover/estimated_marker")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("marker_publish_hz", 2.0)
        self.declare_parameter("rover_marker_scale", 0.7)

        self.terrain_id = str(self.get_parameter("terrain_id").value)
        self.terrain_root = str(self.get_parameter("terrain_root").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.mineral_markers_topic = str(
            self.get_parameter("mineral_markers_topic").value
        )
        self.basecamp_marker_topic = str(
            self.get_parameter("basecamp_marker_topic").value
        )
        self.estimated_odom_topic = str(
            self.get_parameter("estimated_odom_topic").value
        )
        self.estimated_marker_topic = str(
            self.get_parameter("estimated_marker_topic").value
        )
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.marker_publish_hz = float(self.get_parameter("marker_publish_hz").value)
        self.rover_marker_scale = float(self.get_parameter("rover_marker_scale").value)

        self.terrain_dir = self.resolve_terrain_dir()
        self.meta = self.load_meta(self.terrain_dir / "meta.json")
        self.obstacle_grid = np.load(self.terrain_dir / "obstacle_grid.npy")

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
        self.rover_marker_pub = self.create_publisher(
            MarkerArray,
            self.estimated_marker_topic,
            live_qos,
        )

        self.latest_odom: Optional[Odometry] = None
        self.create_subscription(
            Odometry,
            self.estimated_odom_topic,
            self._on_estimated_odom,
            20,
        )

        period = 1.0 / self.marker_publish_hz if self.marker_publish_hz > 0.0 else 0.5
        self.create_timer(period, self._publish_live_markers)

        self.map_msg = self.build_occupancy_grid()
        self.mineral_markers = self.build_mineral_markers()
        self.basecamp_markers = self.build_basecamp_markers()

        self.publish_static()

        self.get_logger().info("Terrain Map Publisher initialized.")
        self.get_logger().info(f"Terrain dir      : {self.terrain_dir}")
        self.get_logger().info(f"Map topic        : {self.map_topic}")
        self.get_logger().info(f"Mineral markers  : {self.mineral_markers_topic}")
        self.get_logger().info(f"Basecamp marker  : {self.basecamp_marker_topic}")
        self.get_logger().info(f"Estimated odom   : {self.estimated_odom_topic}")
        self.get_logger().info(f"Rover marker     : {self.estimated_marker_topic}")

    def resolve_terrain_dir(self) -> Path:
        if self.terrain_root:
            root = Path(self.terrain_root)
        else:
            env_a2_root = os.environ.get("A2_ISAAC_ROOT")
            source_a2_root = Path("/home/rokey/dev_ws/rover_ws/src/a2_isaac")
            installed_a2_root = Path(__file__).resolve().parents[2]

            if env_a2_root:
                a2_root = Path(env_a2_root)
            elif (source_a2_root / "isaac_sim").exists():
                a2_root = source_a2_root
            else:
                a2_root = installed_a2_root

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
            sphere.pose.position.z = float(position.get("z", 0.0)) + 0.35
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.45
            sphere.scale.y = 0.45
            sphere.scale.z = 0.45
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
            label.pose.position.z = sphere.pose.position.z + 0.45
            label.pose.orientation.w = 1.0
            label.scale.z = 0.35
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = f"{mineral_id}:{mineral_type}"
            label.lifetime = Duration(seconds=0.0).to_msg()
            markers.append(label)

        return MarkerArray(markers=markers)

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
        marker.pose.position.z = 0.03
        marker.pose.orientation.w = 1.0
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = 0.06
        marker.color = ColorRGBA(r=0.1, g=0.8, b=1.0, a=0.35)
        marker.lifetime = Duration(seconds=0.0).to_msg()
        markers.append(marker)

        label = Marker()
        label.header.frame_id = self.frame_id
        label.ns = "basecamp_label"
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = marker.pose.position.x
        label.pose.position.y = marker.pose.position.y
        label.pose.position.z = 1.0
        label.pose.orientation.w = 1.0
        label.scale.z = 0.45
        label.color = ColorRGBA(r=0.7, g=1.0, b=1.0, a=1.0)
        label.text = "BASE"
        label.lifetime = Duration(seconds=0.0).to_msg()
        markers.append(label)

        return MarkerArray(markers=markers)

    def _on_estimated_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def publish_static(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.map_msg.header.stamp = stamp
        self.map_pub.publish(self.map_msg)
        self.stamp_markers(self.mineral_markers, stamp)
        self.stamp_markers(self.basecamp_markers, stamp)
        self.mineral_pub.publish(self.mineral_markers)
        self.basecamp_pub.publish(self.basecamp_markers)

    def _publish_live_markers(self) -> None:
        self.publish_static()
        if self.latest_odom is None:
            return
        self.rover_marker_pub.publish(self.build_rover_marker(self.latest_odom))

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
        if mineral_type == "blue":
            return ColorRGBA(r=0.15, g=0.45, b=1.0, a=1.0)
        if mineral_type == "red":
            return ColorRGBA(r=1.0, g=0.15, b=0.05, a=1.0)
        if mineral_type == "yellow":
            return ColorRGBA(r=1.0, g=0.85, b=0.05, a=1.0)
        return ColorRGBA(r=0.8, g=0.8, b=0.8, a=1.0)

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
