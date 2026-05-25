"""ROS2 node: subscribe to Isaac Sim camera topics, run YOLO mineral
detection, publish DetectionArray on /perception/detections.

Phase 1 (this file): RGB + YOLO + DetectionArray publish. If depth /
camera_info / odom are present they are used for naive world-XYZ
estimation; otherwise world_position is zero. Refined 3D pose, mineral_id
matching against T1 meta, and bbox_size_m come in later phases.

Contract: docs/interfaces/INTERFACE_CONTRACTS.md I2 (T2 -> T3, T4).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from isaac_interfaces.msg import Detection, DetectionArray


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


VALUE_SCORE_BY_NAME = {
    "blue_mineral":   10.0,
    "yellow_mineral": 50.0,
    "green_gas":      25.0,
}


@dataclass
class CamIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


def _image_to_bgr(msg: Image) -> np.ndarray:
    """Decode sensor_msgs/Image (rgb8 or bgr8) to BGR uint8 numpy array
    without depending on cv_bridge."""
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"unsupported image encoding: {msg.encoding}")
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    img = buf.reshape(msg.height, msg.width, 3)
    if msg.encoding == "rgb8":
        img = img[..., ::-1]
    return np.ascontiguousarray(img)


def _depth_to_meters(msg: Image) -> Optional[np.ndarray]:
    """Decode Isaac Sim depth (32FC1 meters, or 16UC1 mm) to float32 meters."""
    if msg.encoding == "32FC1":
        arr = np.frombuffer(bytes(msg.data), dtype=np.float32)
        return arr.reshape(msg.height, msg.width).copy()
    if msg.encoding == "16UC1":
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint16)
        return (arr.reshape(msg.height, msg.width).astype(np.float32) * 1e-3).copy()
    return None


class YoloPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_perception_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("rgb_topic",   "/camera/rover/image_raw")
        self.declare_parameter("depth_topic", "/camera/rover/depth")
        self.declare_parameter("info_topic",  "/camera/rover/camera_info")
        self.declare_parameter("odom_topic",  "/ground_truth/odom")
        self.declare_parameter("out_topic",   "/perception/detections")
        self.declare_parameter("annotated_topic", "/perception/image_annotated")
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("frame_id",    "world")
        self.declare_parameter("camera_offset_xyz", [0.37, 0.0, 0.27])
        self.declare_parameter("log_period_sec", 2.0)

        model_path = str(self.get_parameter("model_path").value).strip()
        if not model_path:
            from pathlib import Path
            here = Path(__file__).resolve().parent.parent
            model_path = str(here / "models" / "mineral_yolo_best.pt")

        from .yolo_mineral_detector import YoloMineralDetector
        self.det = YoloMineralDetector(
            model_path,
            conf=float(self.get_parameter("conf_threshold").value),
            iou=float(self.get_parameter("iou_threshold").value),
        )
        self.get_logger().info(f"YOLO loaded: {model_path}")

        self.intr: Optional[CamIntrinsics] = None
        self.last_rgb: Optional[Image] = None
        self.last_depth_m: Optional[np.ndarray] = None
        self.last_odom: Optional[Odometry] = None
        self.frames_seen = 0
        self.frames_published = 0
        self.detections_total = 0

        self.create_subscription(
            Image, str(self.get_parameter("rgb_topic").value),
            self._on_rgb, SENSOR_QOS)
        self.create_subscription(
            Image, str(self.get_parameter("depth_topic").value),
            self._on_depth, SENSOR_QOS)
        self.create_subscription(
            CameraInfo, str(self.get_parameter("info_topic").value),
            self._on_info, SENSOR_QOS)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self._on_odom, SENSOR_QOS)

        self.pub = self.create_publisher(
            DetectionArray, str(self.get_parameter("out_topic").value), 10)
        self.pub_annotated = None
        if bool(self.get_parameter("publish_annotated").value):
            self.pub_annotated = self.create_publisher(
                Image, str(self.get_parameter("annotated_topic").value), SENSOR_QOS)

        rate_hz = max(0.5, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate_hz, self._tick)
        log_period = max(0.5, float(self.get_parameter("log_period_sec").value))
        self.create_timer(log_period, self._log_status)
        self.get_logger().info(
            f"yolo_perception_node ready, publishing /perception/detections @ {rate_hz:.1f} Hz")

    def _on_rgb(self, msg: Image) -> None:
        self.last_rgb = msg
        self.frames_seen += 1

    def _on_depth(self, msg: Image) -> None:
        first = self.last_depth_m is None
        if first:
            self.get_logger().info(
                f"DIAG depth msg: encoding={msg.encoding} {msg.width}x{msg.height} step={msg.step}")
        d = _depth_to_meters(msg)
        if d is None:
            if first:
                self.get_logger().warn(f"DIAG depth decode FAILED for encoding={msg.encoding}")
            return
        if first:
            finite = np.isfinite(d)
            n_finite = int(finite.sum())
            if n_finite > 0:
                mn, mx, med = float(d[finite].min()), float(d[finite].max()), float(np.median(d[finite]))
            else:
                mn = mx = med = float('nan')
            self.get_logger().info(
                f"DIAG depth array: dtype={d.dtype} shape={d.shape} "
                f"finite={n_finite}/{d.size} min={mn:.4f} max={mx:.4f} median={med:.4f}")
        self.last_depth_m = d

    def _on_info(self, msg: CameraInfo) -> None:
        first = self.intr is None
        k = msg.k
        self.intr = CamIntrinsics(
            fx=float(k[0]), fy=float(k[4]),
            cx=float(k[2]), cy=float(k[5]),
            width=int(msg.width), height=int(msg.height),
        )
        if first:
            self.get_logger().info(
                f"DIAG camera_info: {msg.width}x{msg.height} fx={self.intr.fx:.1f} fy={self.intr.fy:.1f} "
                f"cx={self.intr.cx:.1f} cy={self.intr.cy:.1f}")

    def _on_odom(self, msg: Odometry) -> None:
        first = self.last_odom is None
        self.last_odom = msg
        if first:
            p = msg.pose.pose.position
            self.get_logger().info(
                f"DIAG odom first: pos=({p.x:.2f},{p.y:.2f},{p.z:.2f}) frame={msg.header.frame_id}")

    def _tick(self) -> None:
        msg = self.last_rgb
        if msg is None:
            return
        try:
            bgr = _image_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f"RGB decode failed: {e}", throttle_duration_sec=5.0)
            return

        dets = self.det.detect(bgr)
        out = DetectionArray()
        out.header = Header()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = str(self.get_parameter("frame_id").value)

        for d in dets:
            vscore = VALUE_SCORE_BY_NAME.get(d.cls_name, 0.0)
            world_xyz = self._estimate_world_xyz(d.cx, d.cy)
            x1, y1, x2, y2 = [int(round(v)) for v in d.bbox]
            det_msg = Detection()
            det_msg.class_name = d.cls_name
            det_msg.world_position = Point(
                x=float(world_xyz[0]), y=float(world_xyz[1]), z=float(world_xyz[2]))
            det_msg.confidence = float(d.conf)
            det_msg.value_score = float(vscore)
            det_msg.mineral_id = -1
            det_msg.bbox_size_m = Vector3(x=0.0, y=0.0, z=0.0)
            det_msg.bbox_xmin = x1
            det_msg.bbox_ymin = y1
            det_msg.bbox_xmax = x2
            det_msg.bbox_ymax = y2
            out.detections.append(det_msg)

        out.detections.sort(key=lambda d_: d_.confidence, reverse=True)
        self.pub.publish(out)
        self.frames_published += 1
        self.detections_total += len(out.detections)

        if self.pub_annotated is not None:
            annotated = self.det.draw_overlay(bgr, dets)
            img_msg = Image()
            img_msg.header.stamp = msg.header.stamp
            img_msg.header.frame_id = msg.header.frame_id
            img_msg.height = annotated.shape[0]
            img_msg.width = annotated.shape[1]
            img_msg.encoding = "bgr8"
            img_msg.is_bigendian = 0
            img_msg.step = annotated.shape[1] * 3
            img_msg.data = annotated.tobytes()
            self.pub_annotated.publish(img_msg)

    def _estimate_world_xyz(self, px: float, py: float) -> tuple[float, float, float]:
        if self.intr is None or self.last_depth_m is None or self.last_odom is None:
            return (0.0, 0.0, 0.0)
        h, w = self.last_depth_m.shape
        ix = int(np.clip(round(px), 0, w - 1))
        iy = int(np.clip(round(py), 0, h - 1))
        z = float(self.last_depth_m[iy, ix])
        if not np.isfinite(z) or z <= 0.05 or z > 50.0:
            self.get_logger().warn(
                f"DIAG depth invalid at px=({ix},{iy}) value={z:.4f} (img {w}x{h})",
                throttle_duration_sec=2.0)
            return (0.0, 0.0, 0.0)

        # Camera optical frame XYZ (z forward, x right, y down)
        x_c = (px - self.intr.cx) * z / self.intr.fx
        y_c = (py - self.intr.cy) * z / self.intr.fy
        z_c = z

        # Optical -> ROS camera body (x forward, y left, z up)
        x_cam = z_c
        y_cam = -x_c
        z_cam = -y_c

        # Camera body -> rover body (static offset; orientation = identity placeholder)
        ox, oy, oz = [float(v) for v in self.get_parameter("camera_offset_xyz").value]
        x_r = x_cam + ox
        y_r = y_cam + oy
        z_r = z_cam + oz

        # Rover -> world (yaw only from odom quaternion)
        p = self.last_odom.pose.pose.position
        q = self.last_odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = float(np.arctan2(siny_cosp, cosy_cosp))
        c, s = float(np.cos(yaw)), float(np.sin(yaw))
        x_w = p.x + c * x_r - s * y_r
        y_w = p.y + s * x_r + c * y_r
        z_w = p.z + z_r
        return (x_w, y_w, z_w)

    def _log_status(self) -> None:
        have = []
        if self.last_rgb is not None: have.append("rgb")
        if self.last_depth_m is not None: have.append("depth")
        if self.intr is not None: have.append("info")
        if self.last_odom is not None: have.append("odom")
        self.get_logger().info(
            f"frames_seen={self.frames_seen} pub={self.frames_published} "
            f"dets_total={self.detections_total} have=[{','.join(have) or '-'}]")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = YoloPerceptionNode()
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
