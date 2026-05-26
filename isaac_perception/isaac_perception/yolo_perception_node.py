"""ROS2 perception node: YOLO mineral detection on rover (nav) camera and
optionally wrist camera. Publishes DetectionArray + annotated Image per
channel.

Two channels:
- nav (rover body cam): world-frame mineral coordinates (depth + odom +
  camera_offset). Used by mission_manager for APPROACH planning.
- wrist (gripper cam): camera optical frame mineral coordinates only.
  Used by arm_executor for precision visual servoing + IK target.

Contract: docs/interfaces/INTERFACE_CONTRACTS.md I2.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

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

# Annotated image 만 RELIABLE 로 발행 — web_video_server (RELIABLE subscribe)
# 와 QoS 호환을 위해. detection FSM 자체는 SENSOR_QOS 그대로 유지.
ANNOTATED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
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
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"unsupported image encoding: {msg.encoding}")
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    img = buf.reshape(msg.height, msg.width, 3)
    if msg.encoding == "rgb8":
        img = img[..., ::-1]
    return np.ascontiguousarray(img)


def _depth_to_meters(msg: Image) -> Optional[np.ndarray]:
    if msg.encoding == "32FC1":
        arr = np.frombuffer(bytes(msg.data), dtype=np.float32)
        return arr.reshape(msg.height, msg.width).copy()
    if msg.encoding == "16UC1":
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint16)
        return (arr.reshape(msg.height, msg.width).astype(np.float32) * 1e-3).copy()
    return None


class CamChannel:
    """Per-camera processing: subscribes to RGB/Depth/Info(/Odom), runs YOLO
    on the latest RGB frame at the channel's tick, and publishes
    DetectionArray + (optional) annotated Image.

    mode="world":   compute world-frame XYZ for each detection using
                    depth + intrinsics + camera_offset + odom (yaw only).
                    Needs odom_topic.
    mode="optical": compute camera optical frame XYZ only (z forward,
                    x right, y down). Downstream node (e.g. arm IK) does
                    its own frame transform.
    """

    def __init__(self, *, node: Node, name: str, detector,
                 rgb_topic: str, depth_topic: str, info_topic: str,
                 det_topic: str, ann_topic: str, frame_id: str,
                 mode: str, publish_annotated: bool,
                 odom_topic: Optional[str] = None,
                 camera_offset_xyz: Optional[Tuple[float, float, float]] = None):
        assert mode in ("world", "optical")
        self.node = node
        self.name = name
        self.detector = detector
        self.frame_id = frame_id
        self.mode = mode
        self.publish_annotated_flag = publish_annotated
        self.camera_offset_xyz = camera_offset_xyz or (0.0, 0.0, 0.0)

        self.intr: Optional[CamIntrinsics] = None
        self.last_rgb: Optional[Image] = None
        self.last_depth_m: Optional[np.ndarray] = None
        self.last_odom: Optional[Odometry] = None

        self.frames_seen = 0
        self.frames_published = 0
        self.detections_total = 0

        node.create_subscription(Image, rgb_topic, self._on_rgb, SENSOR_QOS)
        node.create_subscription(Image, depth_topic, self._on_depth, SENSOR_QOS)
        node.create_subscription(CameraInfo, info_topic, self._on_info, SENSOR_QOS)
        if mode == "world" and odom_topic:
            node.create_subscription(Odometry, odom_topic, self._on_odom, SENSOR_QOS)

        self.pub_det = node.create_publisher(DetectionArray, det_topic, 10)
        self.pub_ann = None
        if publish_annotated:
            self.pub_ann = node.create_publisher(Image, ann_topic, ANNOTATED_QOS)

    def _log(self, msg: str, level: str = "info") -> None:
        # rclpy logger caches severity per source line, so call distinct
        # methods from distinct lines.
        full = f"[{self.name}] {msg}"
        logger = self.node.get_logger()
        if level == "warn":
            logger.warn(full)
        elif level == "error":
            logger.error(full)
        else:
            logger.info(full)

    def _on_rgb(self, msg: Image) -> None:
        self.last_rgb = msg
        self.frames_seen += 1

    def _on_depth(self, msg: Image) -> None:
        first = self.last_depth_m is None
        if first:
            self._log(f"depth msg: encoding={msg.encoding} {msg.width}x{msg.height}")
        d = _depth_to_meters(msg)
        if d is None:
            if first:
                self._log(f"depth decode FAILED for encoding={msg.encoding}", "warn")
            return
        if first:
            finite = np.isfinite(d)
            n_finite = int(finite.sum())
            if n_finite > 0:
                mn = float(d[finite].min())
                mx = float(d[finite].max())
                med = float(np.median(d[finite]))
            else:
                mn = mx = med = float('nan')
            self._log(
                f"depth array: shape={d.shape} finite={n_finite}/{d.size} "
                f"min={mn:.3f} max={mx:.3f} med={med:.3f}")
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
            self._log(
                f"camera_info: {msg.width}x{msg.height} fx={self.intr.fx:.1f} "
                f"cx={self.intr.cx:.1f} cy={self.intr.cy:.1f}")

    def _on_odom(self, msg: Odometry) -> None:
        first = self.last_odom is None
        self.last_odom = msg
        if first:
            p = msg.pose.pose.position
            self._log(f"odom first: pos=({p.x:.2f},{p.y:.2f},{p.z:.2f})")

    def tick(self) -> None:
        msg = self.last_rgb
        if msg is None:
            return
        try:
            bgr = _image_to_bgr(msg)
        except Exception as e:
            self._log(f"RGB decode failed: {e}", "warn")
            return

        dets = self.detector.detect(bgr)
        out = DetectionArray()
        out.header = Header()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id

        for d in dets:
            vscore = VALUE_SCORE_BY_NAME.get(d.cls_name, 0.0)
            xyz = self._estimate_xyz(d.cx, d.cy)
            x1, y1, x2, y2 = [int(round(v)) for v in d.bbox]
            det_msg = Detection()
            det_msg.class_name = d.cls_name
            det_msg.world_position = Point(
                x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
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
        self.pub_det.publish(out)
        self.frames_published += 1
        self.detections_total += len(out.detections)

        if self.pub_ann is not None:
            annotated = self.detector.draw_overlay(bgr, dets)
            img_msg = Image()
            img_msg.header.stamp = msg.header.stamp
            img_msg.header.frame_id = msg.header.frame_id
            img_msg.height = annotated.shape[0]
            img_msg.width = annotated.shape[1]
            img_msg.encoding = "bgr8"
            img_msg.is_bigendian = 0
            img_msg.step = annotated.shape[1] * 3
            img_msg.data = annotated.tobytes()
            self.pub_ann.publish(img_msg)

    def _estimate_xyz(self, px: float, py: float) -> Tuple[float, float, float]:
        if self.intr is None or self.last_depth_m is None:
            return (0.0, 0.0, 0.0)
        if self.mode == "world" and self.last_odom is None:
            return (0.0, 0.0, 0.0)

        h, w = self.last_depth_m.shape
        ix = int(np.clip(round(px), 0, w - 1))
        iy = int(np.clip(round(py), 0, h - 1))
        z = float(self.last_depth_m[iy, ix])
        if not np.isfinite(z) or z <= 0.05 or z > 50.0:
            self._log(
                f"depth invalid at px=({ix},{iy}) value={z:.4f}",
                "warn")
            return (0.0, 0.0, 0.0)

        # Camera optical frame (z forward, x right, y down)
        x_c = (px - self.intr.cx) * z / self.intr.fx
        y_c = (py - self.intr.cy) * z / self.intr.fy
        z_c = z

        if self.mode == "optical":
            return (x_c, y_c, z_c)

        # Optical -> ROS camera body (x forward, y left, z up)
        x_cam, y_cam, z_cam = z_c, -x_c, -y_c

        # Camera body -> rover body (static offset)
        ox, oy, oz = self.camera_offset_xyz
        x_r, y_r, z_r = x_cam + ox, y_cam + oy, z_cam + oz

        # Rover -> world (yaw from odom)
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

    def have_str(self) -> str:
        flags: List[str] = []
        if self.last_rgb is not None: flags.append("rgb")
        if self.last_depth_m is not None: flags.append("depth")
        if self.intr is not None: flags.append("info")
        if self.mode == "world" and self.last_odom is not None: flags.append("odom")
        return ",".join(flags) or "-"


class YoloPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_perception_node")

        # Shared YOLO config
        self.declare_parameter("model_path", "")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("log_period_sec", 2.0)

        # Nav (rover body) cam — world frame output for mission_manager
        self.declare_parameter("nav_rgb_topic",   "/camera/rover/image_raw")
        self.declare_parameter("nav_depth_topic", "/camera/rover/depth")
        self.declare_parameter("nav_info_topic",  "/camera/rover/camera_info")
        self.declare_parameter("odom_topic",      "/ground_truth/odom")
        self.declare_parameter("nav_det_topic",   "/perception/detections")
        self.declare_parameter("nav_ann_topic",   "/perception/image_annotated")
        self.declare_parameter("nav_frame_id",    "world")
        self.declare_parameter("nav_camera_offset_xyz", [0.37, 0.0, 0.27])
        self.declare_parameter("nav_publish_annotated", True)

        # Wrist (gripper) cam — optical frame output for arm_executor
        self.declare_parameter("enable_wrist", True)
        self.declare_parameter("wrist_rgb_topic",   "/camera/wrist/image_raw")
        self.declare_parameter("wrist_depth_topic", "/camera/wrist/depth")
        self.declare_parameter("wrist_info_topic",  "/camera/wrist/camera_info")
        self.declare_parameter("wrist_det_topic",   "/perception/wrist_detections")
        self.declare_parameter("wrist_ann_topic",   "/perception/wrist_image_annotated")
        self.declare_parameter("wrist_frame_id",    "wrist_camera_optical")
        self.declare_parameter("wrist_publish_annotated", True)

        model_path = str(self.get_parameter("model_path").value).strip()
        if not model_path:
            from pathlib import Path
            here = Path(__file__).resolve().parent.parent
            model_path = str(here / "models" / "mineral_yolo_best.pt")

        from .yolo_mineral_detector import YoloMineralDetector
        self.detector = YoloMineralDetector(
            model_path,
            conf=float(self.get_parameter("conf_threshold").value),
            iou=float(self.get_parameter("iou_threshold").value),
        )
        self.get_logger().info(f"YOLO loaded: {model_path}")

        self.nav = CamChannel(
            node=self, name="nav", detector=self.detector,
            rgb_topic=str(self.get_parameter("nav_rgb_topic").value),
            depth_topic=str(self.get_parameter("nav_depth_topic").value),
            info_topic=str(self.get_parameter("nav_info_topic").value),
            odom_topic=str(self.get_parameter("odom_topic").value),
            det_topic=str(self.get_parameter("nav_det_topic").value),
            ann_topic=str(self.get_parameter("nav_ann_topic").value),
            frame_id=str(self.get_parameter("nav_frame_id").value),
            mode="world",
            publish_annotated=bool(self.get_parameter("nav_publish_annotated").value),
            camera_offset_xyz=tuple(
                float(v) for v in self.get_parameter("nav_camera_offset_xyz").value),
        )

        self.wrist: Optional[CamChannel] = None
        if bool(self.get_parameter("enable_wrist").value):
            self.wrist = CamChannel(
                node=self, name="wrist", detector=self.detector,
                rgb_topic=str(self.get_parameter("wrist_rgb_topic").value),
                depth_topic=str(self.get_parameter("wrist_depth_topic").value),
                info_topic=str(self.get_parameter("wrist_info_topic").value),
                odom_topic=None,
                det_topic=str(self.get_parameter("wrist_det_topic").value),
                ann_topic=str(self.get_parameter("wrist_ann_topic").value),
                frame_id=str(self.get_parameter("wrist_frame_id").value),
                mode="optical",
                publish_annotated=bool(self.get_parameter("wrist_publish_annotated").value),
            )

        rate_hz = max(0.5, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate_hz, self._tick)
        log_period = max(0.5, float(self.get_parameter("log_period_sec").value))
        self.create_timer(log_period, self._log_status)
        self.get_logger().info(
            f"yolo_perception_node ready @ {rate_hz:.1f} Hz "
            f"(nav: world, wrist: {'optical' if self.wrist else 'disabled'})")

    def _tick(self) -> None:
        self.nav.tick()
        if self.wrist is not None:
            self.wrist.tick()

    def _log_status(self) -> None:
        line = (f"nav: seen={self.nav.frames_seen} pub={self.nav.frames_published} "
                f"dets={self.nav.detections_total} have=[{self.nav.have_str()}]")
        if self.wrist is not None:
            line += (f" | wrist: seen={self.wrist.frames_seen} "
                     f"pub={self.wrist.frames_published} "
                     f"dets={self.wrist.detections_total} "
                     f"have=[{self.wrist.have_str()}]")
        self.get_logger().info(line)


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
