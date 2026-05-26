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
from typing import Dict, List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import Float32


class SunYawNode(Node):
    """Image-based sun compass observation node."""

    def __init__(self) -> None:
        super().__init__("sun_yaw_node")

        self.declare_parameter("image_topic", "/camera/sun/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/sun/camera_info")
        self.declare_parameter("sun_yaw_topic", "/rover/sun_yaw")
        self.declare_parameter("confidence_topic", "/rover/sun_yaw_confidence")
        self.declare_parameter("publish_confidence", True)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("world_sun_yaw", math.radians(-25.0))
        self.declare_parameter("camera_yaw_offset", 0.0)
        self.declare_parameter("camera_elevation", 0.0)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("use_imu_tilt_compensation", False)
        self.declare_parameter("min_confidence", 0.12)
        self.declare_parameter("base_yaw_variance", 0.35)
        self.declare_parameter("max_yaw_variance", 9.0)
        self.declare_parameter("bright_percentile", 99.2)
        self.declare_parameter("top_crop_ratio", 0.65)
        self.declare_parameter("min_peak_luma", 190.0)
        self.declare_parameter("min_area_ratio", 1.0e-5)
        self.declare_parameter("max_area_ratio", 0.025)
        self.declare_parameter("max_blob_width_ratio", 0.22)
        self.declare_parameter("max_blob_height_ratio", 0.22)
        self.declare_parameter("min_dominance", 18.0)
        self.declare_parameter("temporal_alpha", 0.35)
        self.declare_parameter("max_bearing_jump", 0.45)
        self.declare_parameter("publish_rejected", False)
        self.declare_parameter("max_publish_hz", 10.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("auto_align_initial_yaw", False)
        self.declare_parameter("alignment_samples", 6)
        self.declare_parameter("alignment_min_confidence", 0.20)
        self.declare_parameter("alignment_max_residual", math.pi)
        self.declare_parameter("debug_log_hz", 0.5)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.sun_yaw_topic = str(self.get_parameter("sun_yaw_topic").value)
        self.confidence_topic = str(self.get_parameter("confidence_topic").value)
        self.publish_confidence = bool(
            self.get_parameter("publish_confidence").value
        )
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.world_sun_yaw = float(self.get_parameter("world_sun_yaw").value)
        self.camera_yaw_offset = float(
            self.get_parameter("camera_yaw_offset").value
        )
        self.camera_elevation = float(self.get_parameter("camera_elevation").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.use_imu_tilt_compensation = bool(
            self.get_parameter("use_imu_tilt_compensation").value
        )
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.base_yaw_variance = float(self.get_parameter("base_yaw_variance").value)
        self.max_yaw_variance = float(self.get_parameter("max_yaw_variance").value)
        self.bright_percentile = float(
            self.get_parameter("bright_percentile").value
        )
        self.top_crop_ratio = float(self.get_parameter("top_crop_ratio").value)
        self.min_peak_luma = float(self.get_parameter("min_peak_luma").value)
        self.min_area_ratio = float(self.get_parameter("min_area_ratio").value)
        self.max_area_ratio = float(self.get_parameter("max_area_ratio").value)
        self.max_blob_width_ratio = float(
            self.get_parameter("max_blob_width_ratio").value
        )
        self.max_blob_height_ratio = float(
            self.get_parameter("max_blob_height_ratio").value
        )
        self.min_dominance = float(self.get_parameter("min_dominance").value)
        self.temporal_alpha = float(self.get_parameter("temporal_alpha").value)
        self.max_bearing_jump = float(self.get_parameter("max_bearing_jump").value)
        self.publish_rejected = bool(self.get_parameter("publish_rejected").value)
        self.max_publish_hz = float(self.get_parameter("max_publish_hz").value)
        self.initial_yaw = float(self.get_parameter("initial_yaw").value)
        self.auto_align_initial_yaw = bool(
            self.get_parameter("auto_align_initial_yaw").value
        )
        self.alignment_samples = int(self.get_parameter("alignment_samples").value)
        self.alignment_min_confidence = float(
            self.get_parameter("alignment_min_confidence").value
        )
        self.alignment_max_residual = float(
            self.get_parameter("alignment_max_residual").value
        )
        self.debug_log_hz = float(self.get_parameter("debug_log_hz").value)

        self.latest_camera_info: Optional[CameraInfo] = None
        self.latest_imu: Optional[Imu] = None
        self.last_publish_time = None
        self.last_debug_log_time = None
        self.filtered_bearing: Optional[float] = None
        self.last_raw_bearing: Optional[float] = None
        self.last_published_yaw: Optional[float] = None
        self.selected_candidate_name: Optional[str] = None
        self.alignment_correction = 0.0
        self.alignment_buffer: List[Dict[str, float]] = []
        self.last_sun_debug: Dict[str, float] = {}

        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._on_camera_info,
            10,
        )
        self.create_subscription(Image, self.image_topic, self._on_image, 10)
        if self.use_imu_tilt_compensation:
            self.create_subscription(Imu, self.imu_topic, self._on_imu, 10)

        self.yaw_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.sun_yaw_topic,
            10,
        )
        self.conf_pub = (
            self.create_publisher(Float32, self.confidence_topic, 10)
            if self.publish_confidence
            else None
        )

        self.get_logger().info("Sun Yaw Node initialized.")
        self.get_logger().info(f"Image topic       : {self.image_topic}")
        self.get_logger().info(f"CameraInfo topic  : {self.camera_info_topic}")
        self.get_logger().info(f"Sun yaw topic     : {self.sun_yaw_topic}")
        self.get_logger().info(
            f"World sun yaw     : {self.world_sun_yaw:.3f} rad"
        )
        if self.auto_align_initial_yaw:
            self.get_logger().info(
                "Initial-yaw sun alignment enabled: "
                f"initial_yaw={self.initial_yaw:.3f} rad, "
                f"samples={self.alignment_samples}"
            )
        if self.use_imu_tilt_compensation:
            self.get_logger().info(f"IMU tilt topic    : {self.imu_topic}")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def _on_imu(self, msg: Imu) -> None:
        self.latest_imu = msg

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
        observed_bearing, confidence = self._filter_bearing(
            observed_bearing,
            confidence,
        )
        if self.conf_pub is not None:
            self.conf_pub.publish(Float32(data=float(confidence)))

        if confidence < self.min_confidence and not self.publish_rejected:
            self.last_publish_time = self.get_clock().now()
            self._maybe_log_debug(observed_bearing, confidence, None, "low_confidence")
            return

        rover_yaw = self._resolve_rover_yaw(observed_bearing, confidence)
        if rover_yaw is None:
            self.last_publish_time = self.get_clock().now()
            self._maybe_log_debug(observed_bearing, confidence, None, "aligning")
            return

        variance = self._variance_from_confidence(confidence)
        self.yaw_pub.publish(self._build_yaw_msg(msg, rover_yaw, variance))
        self.last_published_yaw = rover_yaw
        self.last_publish_time = self.get_clock().now()
        self._maybe_log_debug(observed_bearing, confidence, rover_yaw, "published")

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

        peak_luma = float(np.max(crop))
        if peak_luma < self.min_peak_luma:
            return 0.0, 0.0

        threshold = float(np.percentile(crop, self.bright_percentile))
        if peak_luma - threshold < self.min_dominance:
            threshold = max(0.0, peak_luma - self.min_dominance)

        bright = crop >= threshold
        if not np.any(bright):
            return 0.0, 0.0

        peak_y, peak_x = np.unravel_index(int(np.argmax(crop)), crop.shape)
        component = self._connected_component_from_seed(
            bright,
            int(peak_y),
            int(peak_x),
        )
        if np.any(component):
            bright = component

        weights = np.maximum(crop - threshold, 1.0) * bright
        total = float(np.sum(weights))
        if total <= 0.0:
            return 0.0, 0.0

        xs = np.arange(width, dtype=np.float32)[None, :]
        ys = np.arange(crop_h, dtype=np.float32)[:, None]
        u = float(np.sum(weights * xs) / total)
        v = float(np.sum(weights * ys) / total)
        dx = xs - u
        dy = ys - v
        std_x = math.sqrt(float(np.sum(weights * dx * dx) / total))
        std_y = math.sqrt(float(np.sum(weights * dy * dy) / total))

        fx = float(camera_info.k[0]) if camera_info.k[0] else 0.0
        fy = float(camera_info.k[4]) if camera_info.k[4] else 0.0
        cx = float(camera_info.k[2]) if camera_info.k[2] else 0.5 * (width - 1)
        cy = float(camera_info.k[5]) if camera_info.k[5] else 0.5 * (height - 1)
        if fx <= 1e-6:
            fx = 0.5 * width
        if fy <= 1e-6:
            fy = fx

        x_over_z = (u - cx) / fx
        y_over_z = (v - cy) / fy
        ray_base = self._ray_from_optical_ray(x_over_z, y_over_z)
        bearing_base = self._bearing_from_base_ray(ray_base)
        if self.use_imu_tilt_compensation and self.latest_imu is not None:
            bearing_base = self._tilt_compensated_bearing(ray_base)

        area_ratio = float(np.count_nonzero(bright) / bright.size)
        if area_ratio < self.min_area_ratio or area_ratio > self.max_area_ratio:
            return bearing_base, 0.0

        width_ratio = std_x / max(1.0, float(width))
        height_ratio = std_y / max(1.0, float(crop_h))
        if width_ratio > self.max_blob_width_ratio:
            return bearing_base, 0.0
        if height_ratio > self.max_blob_height_ratio:
            return bearing_base, 0.0

        mean_luma = float(np.mean(crop))
        dominance = max(0.0, peak_luma - threshold) / max(1.0, self.min_dominance)
        brightness = max(0.0, min(1.0, (peak_luma - self.min_peak_luma) / 65.0))
        contrast = max(0.0, min(1.0, (peak_luma - mean_luma) / 180.0))
        area_score = self._trapezoid_score(
            area_ratio,
            self.min_area_ratio,
            self.min_area_ratio * 8.0,
            self.max_area_ratio * 0.55,
            self.max_area_ratio,
        )
        compact_x = max(0.0, 1.0 - width_ratio / self.max_blob_width_ratio)
        compact_y = max(0.0, 1.0 - height_ratio / self.max_blob_height_ratio)
        top_score = max(0.0, 1.0 - (v / max(1.0, float(crop_h))) * 0.7)
        confidence = (
            brightness
            * contrast
            * min(1.0, dominance)
            * area_score
            * math.sqrt(compact_x * compact_y)
            * top_score
        )
        confidence = max(0.0, min(1.0, confidence))
        self.last_sun_debug = {
            "u": u,
            "v": v,
            "peak_luma": peak_luma,
            "threshold": threshold,
            "area_ratio": area_ratio,
            "width_ratio": width_ratio,
            "height_ratio": height_ratio,
        }

        return bearing_base, confidence

    def _resolve_rover_yaw(
        self,
        observed_bearing: float,
        confidence: float,
    ) -> Optional[float]:
        candidates = self._candidate_yaws(observed_bearing)

        if self.auto_align_initial_yaw and self.selected_candidate_name is None:
            if confidence >= self.alignment_min_confidence:
                self.alignment_buffer.append(candidates)
                if len(self.alignment_buffer) >= max(1, self.alignment_samples):
                    self._finish_initial_alignment()

            if self.selected_candidate_name is None:
                return None

        if self.selected_candidate_name is not None:
            yaw = candidates[self.selected_candidate_name]
            yaw = self.normalize_angle(yaw + self.alignment_correction)
            return yaw

        return candidates["world_minus_bearing"]

    def _finish_initial_alignment(self) -> None:
        names = list(self.alignment_buffer[0].keys())
        scores = {}
        residuals_by_name = {}
        for name in names:
            residuals = [
                self.normalize_angle(self.initial_yaw - sample[name])
                for sample in self.alignment_buffer
            ]
            residuals_by_name[name] = residuals
            scores[name] = float(np.median(np.abs(residuals)))

        best_name = min(scores, key=scores.get)
        correction = self._circular_mean(residuals_by_name[best_name])
        if abs(correction) > self.alignment_max_residual:
            self.get_logger().warning(
                "Sun yaw initial alignment residual too large: "
                f"candidate={best_name}, correction={correction:.3f} rad. "
                "Keeping uncorrected world_minus_bearing output."
            )
            self.selected_candidate_name = "world_minus_bearing"
            self.alignment_correction = 0.0
            return

        self.selected_candidate_name = best_name
        self.alignment_correction = correction
        score_text = ", ".join(
            f"{name}={scores[name]:.3f}" for name in sorted(scores)
        )
        self.get_logger().info(
            "Sun yaw initial alignment selected: "
            f"{best_name}, correction={correction:.3f} rad, scores[{score_text}]"
        )

    def _candidate_yaws(self, observed_bearing: float) -> Dict[str, float]:
        return {
            "world_minus_bearing": self.normalize_angle(
                self.world_sun_yaw - observed_bearing
            ),
            "world_plus_bearing": self.normalize_angle(
                self.world_sun_yaw + observed_bearing
            ),
            "world_minus_bearing_pi": self.normalize_angle(
                self.world_sun_yaw - observed_bearing + math.pi
            ),
            "world_plus_bearing_pi": self.normalize_angle(
                self.world_sun_yaw + observed_bearing + math.pi
            ),
        }

    @staticmethod
    def _circular_mean(angles: List[float]) -> float:
        if not angles:
            return 0.0
        s = float(np.sum(np.sin(angles)))
        c = float(np.sum(np.cos(angles)))
        return math.atan2(s, c)

    def _maybe_log_debug(
        self,
        observed_bearing: float,
        confidence: float,
        rover_yaw: Optional[float],
        status: str,
    ) -> None:
        if self.debug_log_hz <= 0.0:
            return
        now = self.get_clock().now()
        if self.last_debug_log_time is not None:
            elapsed = now - self.last_debug_log_time
            if elapsed.nanoseconds < int(1e9 / self.debug_log_hz):
                return
        self.last_debug_log_time = now

        dbg = self.last_sun_debug
        yaw_text = "none" if rover_yaw is None else f"{rover_yaw:.3f}"
        self.get_logger().info(
            "sun_yaw debug: "
            f"status={status}, conf={confidence:.3f}, "
            f"bearing={observed_bearing:.3f}, yaw={yaw_text}, "
            f"candidate={self.selected_candidate_name}, "
            f"corr={self.alignment_correction:.3f}, "
            f"u={dbg.get('u', float('nan')):.1f}, "
            f"v={dbg.get('v', float('nan')):.1f}, "
            f"area={dbg.get('area_ratio', float('nan')):.5f}, "
            f"peak={dbg.get('peak_luma', float('nan')):.1f}"
        )

    @staticmethod
    def _connected_component_from_seed(
        mask: np.ndarray,
        seed_y: int,
        seed_x: int,
    ) -> np.ndarray:
        height, width = mask.shape
        component = np.zeros_like(mask, dtype=bool)
        if not (0 <= seed_y < height and 0 <= seed_x < width):
            return component
        if not bool(mask[seed_y, seed_x]):
            return component

        stack = [(seed_y, seed_x)]
        component[seed_y, seed_x] = True
        while stack:
            y, x = stack.pop()
            for ny in range(max(0, y - 1), min(height, y + 2)):
                for nx in range(max(0, x - 1), min(width, x + 2)):
                    if component[ny, nx] or not bool(mask[ny, nx]):
                        continue
                    component[ny, nx] = True
                    stack.append((ny, nx))
        return component

    def _ray_from_optical_ray(
        self,
        x_over_z: float,
        y_over_z: float,
    ) -> np.ndarray:
        yaw = self.camera_yaw_offset
        elev = self.camera_elevation
        cyaw = math.cos(yaw)
        syaw = math.sin(yaw)
        ce = math.cos(elev)
        se = math.sin(elev)

        forward_x = ce * cyaw
        forward_y = ce * syaw
        forward_z = se

        right_x = syaw
        right_y = -cyaw
        right_z = 0.0

        down_x = forward_y * right_z - forward_z * right_y
        down_y = forward_z * right_x - forward_x * right_z
        down_z = forward_x * right_y - forward_y * right_x

        ray_x = forward_x + x_over_z * right_x + y_over_z * down_x
        ray_y = forward_y + x_over_z * right_y + y_over_z * down_y
        ray_z = forward_z + x_over_z * right_z + y_over_z * down_z
        ray = np.array([ray_x, ray_y, ray_z], dtype=np.float64)
        norm = float(np.linalg.norm(ray))
        if norm <= 1e-9:
            return ray
        return ray / norm

    def _bearing_from_base_ray(self, ray_base: np.ndarray) -> float:
        return self.normalize_angle(math.atan2(float(ray_base[1]), float(ray_base[0])))

    def _tilt_compensated_bearing(self, ray_base: np.ndarray) -> float:
        if self.latest_imu is None:
            return self._bearing_from_base_ray(ray_base)

        roll, pitch = self._roll_pitch_from_quaternion(
            self.latest_imu.orientation.x,
            self.latest_imu.orientation.y,
            self.latest_imu.orientation.z,
            self.latest_imu.orientation.w,
        )
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)

        x, y, z = float(ray_base[0]), float(ray_base[1]), float(ray_base[2])
        rx_x = x
        rx_y = cr * y - sr * z
        rx_z = sr * y + cr * z
        world0_x = cp * rx_x + sp * rx_z
        world0_y = rx_y
        return self.normalize_angle(math.atan2(world0_y, world0_x))

    @staticmethod
    def _roll_pitch_from_quaternion(
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> tuple[float, float]:
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        return roll, pitch

    def _filter_bearing(
        self,
        observed_bearing: float,
        confidence: float,
    ) -> tuple[float, float]:
        confidence = max(0.0, min(1.0, float(confidence)))
        observed_bearing = self.normalize_angle(observed_bearing)

        if confidence <= 0.0:
            return observed_bearing, 0.0

        if self.last_raw_bearing is not None:
            jump = abs(self.normalize_angle(observed_bearing - self.last_raw_bearing))
            if jump > self.max_bearing_jump:
                confidence *= max(0.0, self.max_bearing_jump / jump)

        self.last_raw_bearing = observed_bearing

        if self.filtered_bearing is None:
            self.filtered_bearing = observed_bearing
            return observed_bearing, confidence

        alpha = max(0.0, min(1.0, self.temporal_alpha))
        delta = self.normalize_angle(observed_bearing - self.filtered_bearing)
        self.filtered_bearing = self.normalize_angle(
            self.filtered_bearing + alpha * delta
        )
        smoothed_jump = abs(delta)
        if smoothed_jump > self.max_bearing_jump:
            confidence *= max(0.0, self.max_bearing_jump / smoothed_jump)

        return self.filtered_bearing, confidence

    @staticmethod
    def _trapezoid_score(
        value: float,
        hard_min: float,
        soft_min: float,
        soft_max: float,
        hard_max: float,
    ) -> float:
        if value <= hard_min or value >= hard_max:
            return 0.0
        if soft_min <= value <= soft_max:
            return 1.0
        if value < soft_min:
            return max(0.0, (value - hard_min) / max(1e-9, soft_min - hard_min))
        return max(0.0, (hard_max - value) / max(1e-9, hard_max - soft_max))

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
