#!/usr/bin/env python3

import math
from typing import Dict, Optional, Tuple

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
        self.declare_parameter("obstacle_patch_topic", "/rover/local_obstacle_patch")

        self.declare_parameter("patch_rows", 121)
        self.declare_parameter("patch_cols", 121)
        self.declare_parameter("patch_resolution", 0.05)
        self.declare_parameter("publish_rate_hz", 2.0)

        # Patch center in the rover base frame. The rover camera mainly sees
        # terrain several meters ahead, so centering the patch in front of the
        # rover keeps observed cells away from the patch boundary.
        self.declare_parameter("patch_center_x", 3.0)
        self.declare_parameter("patch_center_y", 0.0)

        # Camera extrinsic in rover base frame. By default this matches the
        # original build_rover_m0609_scene.py assumption:
        # optical +Z forward -> base +X, +X right -> base -Y, +Y down -> base -Z.
        self.declare_parameter("camera_x", 0.35)
        self.declare_parameter("camera_y", 0.0)
        self.declare_parameter("camera_z", 0.30)
        self.declare_parameter("camera_forward_x", 1.0)
        self.declare_parameter("camera_forward_y", 0.0)
        self.declare_parameter("camera_forward_z", 0.0)
        self.declare_parameter("camera_right_x", 0.0)
        self.declare_parameter("camera_right_y", -1.0)
        self.declare_parameter("camera_right_z", 0.0)
        self.declare_parameter("camera_down_x", 0.0)
        self.declare_parameter("camera_down_y", 0.0)
        self.declare_parameter("camera_down_z", -1.0)

        self.declare_parameter("min_depth_m", 0.05)
        self.declare_parameter("max_depth_m", 8.0)
        self.declare_parameter("pixel_stride", 4)
        self.declare_parameter("min_valid_cells", 100)
        self.declare_parameter("accumulate_window_s", 0.0)
        self.declare_parameter("accumulate_max_cells", 80000)
        self.declare_parameter("fill_sparse_holes", False)
        self.declare_parameter("obstacle_ground_percentile", 35.0)
        self.declare_parameter("obstacle_height_delta_m", 0.35)
        self.declare_parameter("obstacle_slope_deg", 20.0)
        self.declare_parameter("obstacle_dilation_cells", 1)

        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.patch_topic = self.get_parameter("patch_topic").value
        self.obstacle_patch_topic = self.get_parameter("obstacle_patch_topic").value

        self.patch_rows = int(self.get_parameter("patch_rows").value)
        self.patch_cols = int(self.get_parameter("patch_cols").value)
        self.patch_resolution = float(self.get_parameter("patch_resolution").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.patch_center_x = float(self.get_parameter("patch_center_x").value)
        self.patch_center_y = float(self.get_parameter("patch_center_y").value)

        self.camera_x = float(self.get_parameter("camera_x").value)
        self.camera_y = float(self.get_parameter("camera_y").value)
        self.camera_z = float(self.get_parameter("camera_z").value)
        self.camera_forward = np.array(
            [
                float(self.get_parameter("camera_forward_x").value),
                float(self.get_parameter("camera_forward_y").value),
                float(self.get_parameter("camera_forward_z").value),
            ],
            dtype=np.float64,
        )
        self.camera_right = np.array(
            [
                float(self.get_parameter("camera_right_x").value),
                float(self.get_parameter("camera_right_y").value),
                float(self.get_parameter("camera_right_z").value),
            ],
            dtype=np.float64,
        )
        self.camera_down = np.array(
            [
                float(self.get_parameter("camera_down_x").value),
                float(self.get_parameter("camera_down_y").value),
                float(self.get_parameter("camera_down_z").value),
            ],
            dtype=np.float64,
        )

        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.pixel_stride = max(1, int(self.get_parameter("pixel_stride").value))
        self.min_valid_cells = int(self.get_parameter("min_valid_cells").value)
        self.accumulate_window_s = float(
            self.get_parameter("accumulate_window_s").value
        )
        self.accumulate_max_cells = int(
            self.get_parameter("accumulate_max_cells").value
        )
        self.fill_sparse_holes = bool(
            self.get_parameter("fill_sparse_holes").value
        )
        self.obstacle_ground_percentile = float(
            self.get_parameter("obstacle_ground_percentile").value
        )
        self.obstacle_height_delta_m = float(
            self.get_parameter("obstacle_height_delta_m").value
        )
        self.obstacle_slope_deg = float(
            self.get_parameter("obstacle_slope_deg").value
        )
        self.obstacle_dilation_cells = max(
            0,
            int(self.get_parameter("obstacle_dilation_cells").value),
        )

        self.latest_camera_info: Optional[CameraInfo] = None
        self.latest_odom: Optional[Odometry] = None
        self.last_publish_ns = 0
        self.warned_encoding = False
        self.warned_sparse = False
        self.accum_cells: Dict[Tuple[int, int], Tuple[float, int, int]] = {}

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
        self.obstacle_patch_pub = self.create_publisher(
            Float32MultiArray,
            self.obstacle_patch_topic,
            10,
        )

        self.get_logger().info("Local Height Patch Node initialized.")
        self.get_logger().info(f"Depth topic      : {self.depth_topic}")
        self.get_logger().info(f"CameraInfo topic : {self.camera_info_topic}")
        self.get_logger().info(f"Odom topic       : {self.odom_topic}")
        self.get_logger().info(f"Patch topic      : {self.patch_topic}")
        self.get_logger().info(f"Obstacle topic   : {self.obstacle_patch_topic}")
        self.get_logger().info(
            f"Patch shape/res  : {self.patch_rows}x{self.patch_cols} @ "
            f"{self.patch_resolution:.3f} m/cell"
        )
        self.get_logger().info(
            f"Patch center     : base x={self.patch_center_x:.3f}, "
            f"y={self.patch_center_y:.3f}"
        )
        self.get_logger().info(
            "Camera extrinsic : "
            f"pos=({self.camera_x:.3f}, {self.camera_y:.3f}, {self.camera_z:.3f}), "
            f"forward={self.camera_forward.tolist()}, "
            f"right={self.camera_right.tolist()}, down={self.camera_down.tolist()}"
        )
        self.get_logger().info(
            f"Patch accumulation: window={self.accumulate_window_s:.2f}s, "
            f"max_cells={self.accumulate_max_cells}, fill={self.fill_sparse_holes}"
        )
        self.get_logger().info(
            "Obstacle patch   : "
            f"height_delta={self.obstacle_height_delta_m:.2f}m, "
            f"slope={self.obstacle_slope_deg:.1f}deg, "
            f"dilate={self.obstacle_dilation_cells} cells"
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

        patch, obstacle_patch = self._build_patches(
            depth,
            self.latest_camera_info,
            self.latest_odom,
            now_ns,
        )
        if self.fill_sparse_holes:
            patch = self._fill_sparse_holes(patch)
            obstacle_patch = self._fill_binary_holes(obstacle_patch)

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
        self.obstacle_patch_pub.publish(self._patch_to_msg(obstacle_patch))
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

    def _build_patches(
        self,
        depth: np.ndarray,
        camera_info: CameraInfo,
        odom: Odometry,
        now_ns: int,
    ) -> Tuple[np.ndarray, np.ndarray]:
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
        max_heights = np.full_like(patch, np.nan, dtype=np.float32)

        half_rows = self.patch_rows // 2
        half_cols = self.patch_cols // 2

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
                # Convert through the configured fixed camera extrinsic.
                p_base = (
                    np.array([self.camera_x, self.camera_y, self.camera_z])
                    + self.camera_forward * z_opt
                    + self.camera_right * x_opt
                    + self.camera_down * y_opt
                )
                x_base = float(p_base[0])
                y_base = float(p_base[1])
                z_base = float(p_base[2])

                world_x = odom_x + cos_yaw * x_base - sin_yaw * y_base
                world_y = odom_y + sin_yaw * x_base + cos_yaw * y_base
                world_z = odom_z + z_base

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
                if not np.isfinite(max_heights[row_idx, col]) or world_z > max_heights[row_idx, col]:
                    max_heights[row_idx, col] = world_z
                self._accumulate_world_cell(world_x, world_y, world_z, now_ns)

        valid = counts > 0
        patch[valid] = (sums[valid] / counts[valid]).astype(np.float32)

        if self.accumulate_window_s > 0.0:
            self._prune_accumulated_cells(now_ns)
            accumulated = self._crop_accumulated_patch(center_world_x, center_world_y)
            if np.count_nonzero(np.isfinite(accumulated)) >= np.count_nonzero(valid):
                patch = accumulated
                max_heights = accumulated

        obstacle_patch = self._height_to_obstacle_patch(patch, max_heights)
        return patch, obstacle_patch

    def _accumulate_world_cell(
        self,
        world_x: float,
        world_y: float,
        world_z: float,
        now_ns: int,
    ) -> None:
        if self.accumulate_window_s <= 0.0:
            return
        key = (
            int(round(world_x / self.patch_resolution)),
            int(round(world_y / self.patch_resolution)),
        )
        prev = self.accum_cells.get(key)
        if prev is None:
            self.accum_cells[key] = (world_z, 1, now_ns)
            return
        z_sum, count, _ = prev
        self.accum_cells[key] = (z_sum + world_z, count + 1, now_ns)

    def _prune_accumulated_cells(self, now_ns: int) -> None:
        if not self.accum_cells:
            return
        cutoff_ns = now_ns - int(self.accumulate_window_s * 1.0e9)
        stale = [key for key, value in self.accum_cells.items() if value[2] < cutoff_ns]
        for key in stale:
            self.accum_cells.pop(key, None)

        if self.accumulate_max_cells > 0 and len(self.accum_cells) > self.accumulate_max_cells:
            overflow = len(self.accum_cells) - self.accumulate_max_cells
            oldest = sorted(self.accum_cells.items(), key=lambda item: item[1][2])
            for key, _ in oldest[:overflow]:
                self.accum_cells.pop(key, None)

    def _crop_accumulated_patch(
        self,
        center_world_x: float,
        center_world_y: float,
    ) -> np.ndarray:
        patch = np.full((self.patch_rows, self.patch_cols), np.nan, dtype=np.float32)
        half_rows = self.patch_rows // 2
        half_cols = self.patch_cols // 2

        center_i = int(round(center_world_x / self.patch_resolution))
        center_j = int(round(center_world_y / self.patch_resolution))

        for (cell_i, cell_j), (z_sum, count, _) in self.accum_cells.items():
            if count <= 0:
                continue
            col = cell_i - center_i + half_cols
            row = cell_j - center_j + half_rows
            if row < 0 or row >= self.patch_rows:
                continue
            if col < 0 or col >= self.patch_cols:
                continue
            patch[row, col] = float(z_sum / count)
        return patch

    @staticmethod
    def _fill_sparse_holes(patch: np.ndarray) -> np.ndarray:
        filled = patch.copy()
        missing = ~np.isfinite(filled)
        if not np.any(missing):
            return filled

        sums = np.zeros_like(filled, dtype=np.float64)
        counts = np.zeros_like(filled, dtype=np.int32)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                shifted = np.full_like(filled, np.nan)
                src_y0 = max(0, -dy)
                src_y1 = filled.shape[0] - max(0, dy)
                src_x0 = max(0, -dx)
                src_x1 = filled.shape[1] - max(0, dx)
                dst_y0 = max(0, dy)
                dst_y1 = dst_y0 + (src_y1 - src_y0)
                dst_x0 = max(0, dx)
                dst_x1 = dst_x0 + (src_x1 - src_x0)
                shifted[dst_y0:dst_y1, dst_x0:dst_x1] = filled[
                    src_y0:src_y1,
                    src_x0:src_x1,
                ]
                valid = np.isfinite(shifted)
                sums[valid] += shifted[valid]
                counts[valid] += 1

        fillable = missing & (counts >= 4)
        filled[fillable] = (sums[fillable] / counts[fillable]).astype(np.float32)
        return filled

    def _height_to_obstacle_patch(
        self,
        height_patch: np.ndarray,
        max_height_patch: np.ndarray,
    ) -> np.ndarray:
        obstacle = np.full_like(height_patch, np.nan, dtype=np.float32)
        finite = np.isfinite(height_patch)
        if not np.any(finite):
            return obstacle

        ground = float(np.nanpercentile(
            height_patch[finite],
            self.obstacle_ground_percentile,
        ))
        high = np.isfinite(max_height_patch) & (
            max_height_patch > ground + self.obstacle_height_delta_m
        )
        slope = self._compute_slope_deg(height_patch)
        steep = np.isfinite(slope) & (slope > self.obstacle_slope_deg)

        obstacle[finite] = np.where(high[finite] | steep[finite], 1.0, 0.0)
        if self.obstacle_dilation_cells > 0:
            obstacle = self._dilate_binary_obstacle(obstacle, self.obstacle_dilation_cells)
        return obstacle

    def _compute_slope_deg(self, patch: np.ndarray) -> np.ndarray:
        slope = np.full_like(patch, np.nan, dtype=np.float32)
        if patch.size == 0:
            return slope
        arr = patch.astype(np.float32)
        dz_dy, dz_dx = np.gradient(arr, self.patch_resolution, self.patch_resolution)
        raw = np.degrees(np.arctan(np.sqrt(dz_dx * dz_dx + dz_dy * dz_dy)))
        finite = np.isfinite(arr) & np.isfinite(raw)
        slope[finite] = raw[finite].astype(np.float32)
        return slope

    @staticmethod
    def _dilate_binary_obstacle(patch: np.ndarray, radius_cells: int) -> np.ndarray:
        valid = np.isfinite(patch)
        source = valid & (patch > 0.5)
        if radius_cells <= 0 or not np.any(source):
            return patch

        padded = np.pad(source, radius_cells, mode="constant", constant_values=False)
        dilated = np.zeros_like(source, dtype=bool)
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                y0 = radius_cells + dy
                x0 = radius_cells + dx
                dilated |= padded[
                    y0:y0 + source.shape[0],
                    x0:x0 + source.shape[1],
                ]

        out = patch.copy()
        out[valid] = np.where(dilated[valid], 1.0, out[valid])
        return out

    @staticmethod
    def _fill_binary_holes(patch: np.ndarray) -> np.ndarray:
        filled = patch.copy()
        missing = ~np.isfinite(filled)
        if not np.any(missing):
            return filled

        sums = np.zeros_like(filled, dtype=np.float64)
        counts = np.zeros_like(filled, dtype=np.int32)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                shifted = np.full_like(filled, np.nan)
                src_y0 = max(0, -dy)
                src_y1 = filled.shape[0] - max(0, dy)
                src_x0 = max(0, -dx)
                src_x1 = filled.shape[1] - max(0, dx)
                dst_y0 = max(0, dy)
                dst_y1 = dst_y0 + (src_y1 - src_y0)
                dst_x0 = max(0, dx)
                dst_x1 = dst_x0 + (src_x1 - src_x0)
                shifted[dst_y0:dst_y1, dst_x0:dst_x1] = filled[
                    src_y0:src_y1,
                    src_x0:src_x1,
                ]
                valid = np.isfinite(shifted)
                sums[valid] += shifted[valid]
                counts[valid] += 1

        fillable = missing & (counts >= 4)
        filled[fillable] = np.where(
            (sums[fillable] / counts[fillable]) >= 0.5,
            1.0,
            0.0,
        ).astype(np.float32)
        return filled

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
