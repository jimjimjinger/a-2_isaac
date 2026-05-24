#!/usr/bin/env python3

import math
from typing import Optional

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class LocalHeightPatchNode(Node):
    """Build a map-aligned local height patch from the rover depth camera."""

    def __init__(self) -> None:
        super().__init__("local_height_patch_node")

        self.declare_parameter("depth_topic", "/camera/rover/depth")
        self.declare_parameter("camera_info_topic", "/camera/rover/camera_info")
        self.declare_parameter("odom_topic", "/rover/wheel_odom")
        self.declare_parameter("patch_topic", "/rover/local_height_patch")

        self.declare_parameter("patch_rows", 121)
        self.declare_parameter("patch_cols", 121)
        self.declare_parameter("patch_resolution", 0.05)
        self.declare_parameter("publish_rate_hz", 2.0)

        # Patch center in the rover base frame. The rover camera mainly sees
        # terrain several meters ahead, so centering the patch in front of the
        # rover keeps observed cells away from the patch boundary.
        self.declare_parameter("patch_center_x", 3.0)
        self.declare_parameter("patch_center_y", 0.0)

        # build_rover_m0609_scene.py rover camera mount.
        self.declare_parameter("camera_x", 0.35)
        self.declare_parameter("camera_y", 0.0)
        self.declare_parameter("camera_z", 0.30)

        self.declare_parameter("min_depth_m", 0.05)
        self.declare_parameter("max_depth_m", 8.0)
        self.declare_parameter("pixel_stride", 4)
        self.declare_parameter("min_valid_cells", 100)

        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.patch_topic = self.get_parameter("patch_topic").value

        self.patch_rows = int(self.get_parameter("patch_rows").value)
        self.patch_cols = int(self.get_parameter("patch_cols").value)
        self.patch_resolution = float(self.get_parameter("patch_resolution").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.patch_center_x = float(self.get_parameter("patch_center_x").value)
        self.patch_center_y = float(self.get_parameter("patch_center_y").value)

        self.camera_x = float(self.get_parameter("camera_x").value)
        self.camera_y = float(self.get_parameter("camera_y").value)
        self.camera_z = float(self.get_parameter("camera_z").value)

        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.pixel_stride = max(1, int(self.get_parameter("pixel_stride").value))
        self.min_valid_cells = int(self.get_parameter("min_valid_cells").value)

        self.latest_camera_info: Optional[CameraInfo] = None
        self.latest_odom: Optional[Odometry] = None
        self.last_publish_ns = 0
        self.warned_encoding = False
        self.warned_sparse = False

        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self._on_odom,
            10,
        )
        self.create_subscription(
            Image,
            self.depth_topic,
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.patch_pub = self.create_publisher(Float32MultiArray, self.patch_topic, 10)

        self.get_logger().info("Local Height Patch Node initialized.")
        self.get_logger().info(f"Depth topic      : {self.depth_topic}")
        self.get_logger().info(f"CameraInfo topic : {self.camera_info_topic}")
        self.get_logger().info(f"Odom topic       : {self.odom_topic}")
        self.get_logger().info(f"Patch topic      : {self.patch_topic}")
        self.get_logger().info(
            f"Patch shape/res  : {self.patch_rows}x{self.patch_cols} @ "
            f"{self.patch_resolution:.3f} m/cell"
        )
        self.get_logger().info(
            f"Patch center     : base x={self.patch_center_x:.3f}, "
            f"y={self.patch_center_y:.3f}"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _on_depth(self, msg: Image) -> None:
        if self.latest_camera_info is None or self.latest_odom is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self.publish_rate_hz > 0.0:
            period_ns = int(1.0e9 / self.publish_rate_hz)
            if now_ns - self.last_publish_ns < period_ns:
                return

        depth = self._depth_image_to_array(msg)
        if depth is None:
            return

        patch = self._build_patch(depth, self.latest_camera_info, self.latest_odom)
        valid_count = int(np.count_nonzero(np.isfinite(patch)))
        if valid_count < self.min_valid_cells:
            if not self.warned_sparse:
                self.get_logger().warn(
                    f"Local height patch is sparse: {valid_count} valid cells. "
                    "Check camera pose, terrain visibility, and patch size."
                )
                self.warned_sparse = True
            return

        self.patch_pub.publish(self._patch_to_msg(patch))
        self.last_publish_ns = now_ns

    def _depth_image_to_array(self, msg: Image) -> Optional[np.ndarray]:
        encoding = msg.encoding.upper()
        if encoding == "32FC1":
            dtype = np.float32
            scale = 1.0
        elif encoding == "16UC1":
            dtype = np.uint16
            scale = 0.001
        else:
            if not self.warned_encoding:
                self.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}")
                self.warned_encoding = True
            return None

        depth = np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, msg.width))
        return depth.astype(np.float32) * scale

    def _build_patch(
        self,
        depth: np.ndarray,
        camera_info: CameraInfo,
        odom: Odometry,
    ) -> np.ndarray:
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])

        odom_x = odom.pose.pose.position.x
        odom_y = odom.pose.pose.position.y
        odom_z = odom.pose.pose.position.z
        yaw = self._yaw_from_quaternion(odom.pose.pose.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        patch = np.full((self.patch_rows, self.patch_cols), np.nan, dtype=np.float32)
        sums = np.zeros_like(patch, dtype=np.float64)
        counts = np.zeros_like(patch, dtype=np.int32)

        half_rows = self.patch_rows // 2
        half_cols = self.patch_cols // 2

        height, width = depth.shape
        for v in range(0, height, self.pixel_stride):
            row = depth[v]
            for u in range(0, width, self.pixel_stride):
                z_opt = float(row[u])
                if not math.isfinite(z_opt):
                    continue
                if z_opt < self.min_depth_m or z_opt > self.max_depth_m:
                    continue

                x_opt = (u - cx) * z_opt / fx
                y_opt = (v - cy) * z_opt / fy

                # ROS optical frame: x right, y down, z forward.
                x_base = self.camera_x + z_opt
                y_base = self.camera_y - x_opt
                z_base = self.camera_z - y_opt

                world_x = odom_x + cos_yaw * x_base - sin_yaw * y_base
                world_y = odom_y + sin_yaw * x_base + cos_yaw * y_base
                world_z = odom_z + z_base

                center_world_x = (
                    odom_x
                    + cos_yaw * self.patch_center_x
                    - sin_yaw * self.patch_center_y
                )
                center_world_y = (
                    odom_y
                    + sin_yaw * self.patch_center_x
                    + cos_yaw * self.patch_center_y
                )

                rel_x = world_x - center_world_x
                rel_y = world_y - center_world_y

                col = int(round(rel_x / self.patch_resolution)) + half_cols
                row_idx = int(round(rel_y / self.patch_resolution)) + half_rows

                if row_idx < 0 or row_idx >= self.patch_rows:
                    continue
                if col < 0 or col >= self.patch_cols:
                    continue

                sums[row_idx, col] += world_z
                counts[row_idx, col] += 1

        valid = counts > 0
        patch[valid] = (sums[valid] / counts[valid]).astype(np.float32)
        return patch

    def _patch_to_msg(self, patch: np.ndarray) -> Float32MultiArray:
        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(
                label="rows",
                size=self.patch_rows,
                stride=self.patch_rows * self.patch_cols,
            ),
            MultiArrayDimension(
                label="cols",
                size=self.patch_cols,
                stride=self.patch_cols,
            ),
        ]
        msg.data = patch.reshape(-1).astype(float).tolist()
        return msg

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalHeightPatchNode()
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
