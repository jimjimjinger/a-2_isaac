#!/usr/bin/env python3
"""Estimate rover yaw from the observed sun direction.

This node deliberately does not read Isaac Sim light prims. It follows the
real-rover pattern:

  known world sun azimuth + observed camera-frame sun bearing -> rover yaw

The first implementation is intentionally conservative. It publishes a yaw
observation with a large covariance and a confidence topic, so the estimate can
be inspected before it is fused into the EKF.

Default image source is `/camera/sun/*` (the dedicated sun-tracking camera that
vehicle_v3 publishes from Body+0.5m, +z up). Override `image_topic` /
`camera_info_topic` parameters to use a different camera (e.g. body front cam).
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32


class SunYawNode(Node):
    """Image-based sun compass observation node."""

    def __init__(self) -> None:
        super().__init__("sun_yaw_node")

        self.declare_parameter("image_topic", "/camera/sun/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/sun/camera_info")
        self.declare_parameter("sun_yaw_topic", "/rover/sun_yaw")
        self.declare_parameter("confidence_topic", "/rover/sun_yaw_confidence")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("world_sun_yaw", math.radians(-25.0))
        self.declare_parameter("camera_yaw_offset", 0.0)
        self.declare_parameter("min_confidence", 0.12)
        self.declare_parameter("base_yaw_variance", 0.35)
        self.declare_parameter("max_yaw_variance", 9.0)
        self.declare_parameter("bright_percentile", 99.2)
        self.declare_parameter("top_crop_ratio", 0.65)
        self.declare_parameter("publish_rejected", False)
        self.declare_parameter("max_publish_hz", 10.0)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.sun_yaw_topic = str(self.get_parameter("sun_yaw_topic").value)
        self.confidence_topic = str(self.get_parameter("confidence_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.world_sun_yaw = float(self.get_parameter("world_sun_yaw").value)
        self.camera_yaw_offset = float(
            self.get_parameter("camera_yaw_offset").value
        )
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.base_yaw_variance = float(self.get_parameter("base_yaw_variance").value)
        self.max_yaw_variance = float(self.get_parameter("max_yaw_variance").value)
        self.bright_percentile = float(
            self.get_parameter("bright_percentile").value
        )
        self.top_crop_ratio = float(self.get_parameter("top_crop_ratio").value)
        self.publish_rejected = bool(self.get_parameter("publish_rejected").value)
        self.max_publish_hz = float(self.get_parameter("max_publish_hz").value)

        self.latest_camera_info: Optional[CameraInfo] = None
        self.last_publish_time = None

        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._on_camera_info,
            10,
        )
        self.create_subscription(Image, self.image_topic, self._on_image, 10)

        self.yaw_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.sun_yaw_topic,
            10,
        )
        self.conf_pub = self.create_publisher(Float32, self.confidence_topic, 10)

        self.get_logger().info("Sun Yaw Node initialized.")
        self.get_logger().info(f"Image topic       : {self.image_topic}")
        self.get_logger().info(f"CameraInfo topic  : {self.camera_info_topic}")
        self.get_logger().info(f"Sun yaw topic     : {self.sun_yaw_topic}")
        self.get_logger().info(
            f"World sun yaw     : {self.world_sun_yaw:.3f} rad"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def _on_image(self, msg: Image) -> None:
        if self.latest_camera_info is None:
            return
        if not self._publish_due():
            return

        gray = self._image_to_luma(msg)
        if gray is None:
            return

        observed_bearing, confidence = self._estimate_sun_bearing(
            gray,
            self.latest_camera_info,
        )
        self.conf_pub.publish(Float32(data=float(confidence)))

        if confidence < self.min_confidence and not self.publish_rejected:
            self.last_publish_time = self.get_clock().now()
            return

        rover_yaw = self.normalize_angle(self.world_sun_yaw - observed_bearing)
        variance = self._variance_from_confidence(confidence)
        self.yaw_pub.publish(self._build_yaw_msg(msg, rover_yaw, variance))
        self.last_publish_time = self.get_clock().now()

    def _publish_due(self) -> bool:
        if self.max_publish_hz <= 0.0 or self.last_publish_time is None:
            return True
        elapsed = self.get_clock().now() - self.last_publish_time
        return elapsed.nanoseconds >= int(1e9 / self.max_publish_hz)

    def _image_to_luma(self, msg: Image) -> Optional[np.ndarray]:
        encoding = msg.encoding.lower()
        height, width = int(msg.height), int(msg.width)

        try:
            if encoding in ("rgb8", "bgr8"):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
                if encoding == "rgb8":
                    r = arr[:, :, 0].astype(np.float32)
                    g = arr[:, :, 1].astype(np.float32)
                    b = arr[:, :, 2].astype(np.float32)
                else:
                    b = arr[:, :, 0].astype(np.float32)
                    g = arr[:, :, 1].astype(np.float32)
                    r = arr[:, :, 2].astype(np.float32)
                return 0.2126 * r + 0.7152 * g + 0.0722 * b
            if encoding in ("rgba8", "bgra8"):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 4)
                if encoding == "rgba8":
                    r = arr[:, :, 0].astype(np.float32)
                    g = arr[:, :, 1].astype(np.float32)
                    b = arr[:, :, 2].astype(np.float32)
                else:
                    b = arr[:, :, 0].astype(np.float32)
                    g = arr[:, :, 1].astype(np.float32)
                    r = arr[:, :, 2].astype(np.float32)
                return 0.2126 * r + 0.7152 * g + 0.0722 * b
            if encoding in ("mono8", "8uc1"):
                return np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    height, width
                ).astype(np.float32)
        except ValueError as exc:
            self.get_logger().warning(f"Invalid image buffer: {exc}")
            return None

        self.get_logger().warning_once(
            f"Unsupported image encoding for sun yaw: {msg.encoding}"
        )
        return None

    def _estimate_sun_bearing(
        self,
        gray: np.ndarray,
        camera_info: CameraInfo,
    ) -> tuple[float, float]:
        height, width = gray.shape
        crop_h = max(1, int(height * max(0.05, min(self.top_crop_ratio, 1.0))))
        crop = gray[:crop_h, :]

        threshold = np.percentile(crop, self.bright_percentile)
        bright = crop >= threshold
        if not np.any(bright):
            return 0.0, 0.0

        weights = np.maximum(crop - threshold, 1.0) * bright
        total = float(np.sum(weights))
        if total <= 0.0:
            return 0.0, 0.0

        xs = np.arange(width, dtype=np.float32)[None, :]
        u = float(np.sum(weights * xs) / total)

        fx = float(camera_info.k[0]) if camera_info.k[0] else 0.0
        cx = float(camera_info.k[2]) if camera_info.k[2] else 0.5 * (width - 1)
        if fx <= 1e-6:
            fx = 0.5 * width

        x_over_z = (u - cx) / fx
        bearing_camera = -math.atan(x_over_z)
        bearing_base = self.normalize_angle(
            self.camera_yaw_offset + bearing_camera
        )

        contrast = float(np.std(crop) / (np.mean(crop) + 1e-6))
        saturation = float(np.mean(crop[bright]) / 255.0)
        area_ratio = float(np.count_nonzero(bright) / bright.size)
        focus = min(1.0, max(0.0, 0.02 / max(area_ratio, 1e-6)))
        confidence = max(0.0, min(1.0, contrast * saturation * focus * 2.5))

        return bearing_base, confidence

    def _variance_from_confidence(self, confidence: float) -> float:
        confidence = max(0.0, min(1.0, confidence))
        if confidence <= 0.0:
            return self.max_yaw_variance
        variance = self.base_yaw_variance / confidence
        return min(self.max_yaw_variance, max(self.base_yaw_variance, variance))

    def _build_yaw_msg(
        self,
        image_msg: Image,
        yaw: float,
        yaw_variance: float,
    ) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = image_msg.header.stamp
        msg.header.frame_id = self.frame_id
        msg.pose.pose.orientation = self.quaternion_from_yaw(yaw)
        covariance = [0.0] * 36
        covariance[0] = 999.0
        covariance[7] = 999.0
        covariance[14] = 999.0
        covariance[21] = 999.0
        covariance[28] = 999.0
        covariance[35] = float(yaw_variance)
        msg.pose.covariance = covariance
        return msg

    @staticmethod
    def quaternion_from_yaw(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(0.5 * yaw)
        q.w = math.cos(0.5 * yaw)
        return q

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SunYawNode()
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
