"""Mission web node — Flask + Socket.IO dashboard for Mars rover ops.

화성↔지구 통신 비유: 브라우저(지구)가 ROS2 토픽(화성 rover)을 web 으로
구독한다. ROS2 콜백 -> Socket.IO emit -> 브라우저가 SC2 풍 HUD 갱신.

이 노드는 read-only 출력만 담당 (당장은 버튼 액션 미연결 — 디자인 단계).
카메라 영상은 web_video_server (MJPEG over HTTP) 가 별도 포트에서 서빙.

Endpoints:
  GET /         -> templates/index.html
  GET /static/* -> CSS/JS/이미지
  Socket.IO     -> 'state' (mission_state), 'odom', 'minimap', 'cmd_vel'

Run:
  ros2 run isaac_supervisor mission_web_node
  → http://localhost:8088 접속.
"""
from __future__ import annotations

import math
import os
import threading
from typing import Optional

# Async mode: we use "threading" (Flask-SocketIO's built-in werkzeug + thread
# worker) rather than eventlet. eventlet.monkey_patch() rewires the standard
# threading/socket modules, which deadlocks rclpy's executor when we spin it
# in a daemon thread (the spinner stops cooperatively yielding to Flask). For
# our telemetry-only load (a handful of clients, low message rate) threading
# mode is plenty.
_ASYNC_MODE = "threading"

import rclpy
from ament_index_python.packages import get_package_share_directory
from flask import Flask, abort, render_template, send_file
from flask_socketio import SocketIO
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from visualization_msgs.msg import MarkerArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from isaac_interfaces.msg import MissionState


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MissionWebRosNode(Node):
    def __init__(self, sio: SocketIO) -> None:
        super().__init__("mission_web_node")
        self._sio = sio
        self.declare_parameter("body_cam_topic_for_url",
                               "/perception/image_annotated")
        self.declare_parameter("wrist_cam_topic_for_url",
                               "/perception/wrist_image_annotated")
        self.declare_parameter("overview_cam_topic_for_url",
                               "/camera/overview/image_raw")
        self.declare_parameter("chase_cam_topic_for_url",
                               "/camera/chase/image_raw")
        # Overview 슬롯을 정적 이미지로 대체할 때 사용. terrain_NNNNN/preview.png.
        self.declare_parameter(
            "terrain_preview_path",
            os.path.expanduser(
                "~/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/"
                "generated_terrains/terrain_00004/preview.png"))
        # Cached last messages so a late-connecting browser can be primed.
        self._last_state: Optional[dict] = None
        self._last_odom: Optional[dict] = None
        self._last_minimap: Optional[dict] = None
        self._last_path: Optional[dict] = None
        self._last_target: Optional[dict] = None

        self.create_subscription(
            MissionState, "/mission/state", self._on_state, 10)
        self.create_subscription(
            Odometry, "/ground_truth/odom", self._on_odom, SENSOR_QOS)
        self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, 10)
        # minimap_publisher 가 VOLATILE/depth=1 으로 발행하므로 호환 QoS 사용.
        # 첫 프레임 놓치면 다음 publish 까지 awaiting 으로 보일 수 있지만,
        # minimap 은 1~2 Hz 로 갱신되므로 곧 들어옴.
        self.create_subscription(
            OccupancyGrid, "/mission/minimap", self._on_minimap,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ))
        # coverage_node 의 현재 계획 경로 + target anchor — SC2 풍 minimap.
        self.create_subscription(Path, "/mission/path", self._on_path, 10)
        self.create_subscription(
            MarkerArray, "/mission/markers", self._on_markers, 10)
        # mission_manager (supervisor) 의 APPROACH/RTB path/target. 같은 socket
        # event 로 emit 해서 dashboard 가 시간상 최신 source 그리도록.
        self.create_subscription(
            Path, "/supervisor/path", self._on_supervisor_path, 10)
        self.create_subscription(
            PointStamped, "/supervisor/target", self._on_supervisor_target, 10)
        self.get_logger().info("mission_web_node ROS interface ready.")

    def topics_for_camera_urls(self) -> dict:
        return {
            "body":     str(self.get_parameter("body_cam_topic_for_url").value),
            "wrist":    str(self.get_parameter("wrist_cam_topic_for_url").value),
            "overview": str(self.get_parameter("overview_cam_topic_for_url").value),
            "chase":    str(self.get_parameter("chase_cam_topic_for_url").value),
        }

    def last_snapshot(self) -> dict:
        return {
            "state":   self._last_state,
            "odom":    self._last_odom,
            "minimap": self._last_minimap,
            "path":    self._last_path,
            "target":  self._last_target,
        }

    def _on_state(self, msg: MissionState) -> None:
        self._last_state = {
            "state": msg.state,
            "previous_state": msg.previous_state,
            "battery_percent": float(msg.battery_percent),
            "low_battery": bool(msg.low_battery),
            "critical_battery": bool(msg.critical_battery),
            "cargo_count": int(msg.cargo_count),
            "cargo_capacity": int(msg.cargo_capacity),
            "collected_count": int(msg.collected_count),
            "collection_goal": int(msg.collection_goal),
            "collected_blue":   int(msg.collected_blue),
            "collected_yellow": int(msg.collected_yellow),
            "collected_green":  int(msg.collected_green),
            "active_task": msg.active_task,
            "last_error": msg.last_error,
        }
        self._sio.emit("state", self._last_state)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        # vehicle_v3 의 GT_SCRIPT 는 PubGtOdom 에 position/orientation 만 보내고
        # twist 는 채우지 않는다 → 수신측 v.x/v.y 가 항상 0. 그래서 position
        # 변화율로 numerical velocity 를 계산해 SPEED 슬롯이 의미있는 값을
        # 갖게 한다. 짧은 dt 는 0 으로 두어 노이즈/divide-by-zero 방지.
        now_ns = self.get_clock().now().nanoseconds
        speed = 0.0
        prev = getattr(self, "_prev_odom_for_speed", None)
        if prev is not None:
            dt = (now_ns - prev["ns"]) * 1e-9
            if dt > 0.02:
                dx = float(p.x) - prev["x"]
                dy = float(p.y) - prev["y"]
                speed = math.hypot(dx, dy) / dt
        self._prev_odom_for_speed = {"x": float(p.x), "y": float(p.y), "ns": now_ns}
        self._last_odom = {
            "x": float(p.x), "y": float(p.y), "z": float(p.z),
            "yaw_deg": math.degrees(yaw),
            "vx": float(v.x), "vy": float(v.y),
            "speed": float(speed),
            "wz": float(w.z),
        }
        self._sio.emit("odom", self._last_odom)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._sio.emit("cmd_vel", {
            "lin": float(msg.linear.x),
            "ang": float(msg.angular.z),
        })

    def _on_path(self, msg: Path) -> None:
        pts = [(float(ps.pose.position.x), float(ps.pose.position.y))
               for ps in msg.poses]
        self._last_path = {"pts": pts}
        self._sio.emit("path", self._last_path)

    def _on_supervisor_path(self, msg: Path) -> None:
        pts = [(float(ps.pose.position.x), float(ps.pose.position.y))
               for ps in msg.poses]
        # supervisor 가 빈 path 발행하면 (EXPLORE 시) lastPath 그대로 두지 않고
        # 비워서 화면에서 사라지게.
        payload = {"pts": pts}
        self._last_path = payload
        self._sio.emit("path", payload)

    def _on_supervisor_target(self, msg: PointStamped) -> None:
        import math as _math
        x, y = float(msg.point.x), float(msg.point.y)
        if not (_math.isfinite(x) and _math.isfinite(y)):
            # NaN → target 없음. None 으로 emit.
            self._last_target = None
            self._sio.emit("target", {})
            return
        target = {"x": x, "y": y}
        self._last_target = target
        self._sio.emit("target", target)

    def _on_markers(self, msg: MarkerArray) -> None:
        # minimap_publisher 의 ns="target" SPHERE 만 추려서 분홍색 별 위치로.
        # action == 2 (DELETE) 면 target 없음 표시.
        target = None
        for m in msg.markers:
            if m.ns == "target":
                if m.action == 2:
                    target = None
                else:
                    target = {"x": float(m.pose.position.x),
                              "y": float(m.pose.position.y)}
                break
        self._last_target = target  # None or {x,y}
        self._sio.emit("target", target or {})

    def _on_minimap(self, msg: OccupancyGrid) -> None:
        # OccupancyGrid -> compact dict the browser can paint on a canvas.
        # data is row-major int8 in [-1, 100]; we keep as plain list (a few
        # hundred kB) since minimap is published low-rate.
        self._last_minimap = {
            "w": int(msg.info.width),
            "h": int(msg.info.height),
            "res": float(msg.info.resolution),
            "ox": float(msg.info.origin.position.x),
            "oy": float(msg.info.origin.position.y),
            "data": list(msg.data),
        }
        self._sio.emit("minimap", self._last_minimap)


def _spin_ros(ros_node: Node) -> None:
    try:
        rclpy.spin(ros_node)
    except Exception:
        pass


def _resolve_web_dir() -> str:
    """Locate share/isaac_supervisor/web (installed) or src/.../web (dev).
    Prefer installed share so colcon install --symlink-install picks up
    edits live."""
    try:
        share = get_package_share_directory("isaac_supervisor")
        cand = os.path.join(share, "web")
        if os.path.isdir(cand):
            return cand
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "web"))


def main(args: list[str] | None = None) -> None:
    web_dir = _resolve_web_dir()
    app = Flask(
        __name__,
        template_folder=os.path.join(web_dir, "templates"),
        static_folder=os.path.join(web_dir, "static"),
        static_url_path="/static",
    )
    app.config["SECRET_KEY"] = "mars-rover-ops"
    sio = SocketIO(app, async_mode=_ASYNC_MODE, cors_allowed_origins="*")

    rclpy.init(args=args)
    ros_node = MissionWebRosNode(sio)

    @app.route("/")
    def index():
        host = os.environ.get("WEB_VIDEO_HOST", "localhost")
        port = int(os.environ.get("WEB_VIDEO_PORT", "8080"))
        topics = ros_node.topics_for_camera_urls()
        def mjpeg(topic: str) -> str:
            return (f"http://{host}:{port}/stream?topic={topic}"
                    f"&type=mjpeg&quality=70")
        return render_template(
            "index.html",
            cam_wrist=mjpeg(topics["wrist"]),
            cam_chase=mjpeg(topics["chase"]),
            terrain_preview="/terrain/preview",
        )

    @app.route("/terrain/preview")
    def terrain_preview():
        path = str(ros_node.get_parameter("terrain_preview_path").value)
        if not os.path.isfile(path):
            abort(404)
        return send_file(path, mimetype="image/png")

    @sio.on("connect")
    def _on_connect():
        snap = ros_node.last_snapshot()
        if snap["state"]:   sio.emit("state",   snap["state"])
        if snap["odom"]:    sio.emit("odom",    snap["odom"])
        if snap["minimap"]: sio.emit("minimap", snap["minimap"])
        if snap["path"]:    sio.emit("path",    snap["path"])
        if snap["target"]:  sio.emit("target",  snap["target"])

    spinner = threading.Thread(target=_spin_ros, args=(ros_node,), daemon=True)
    spinner.start()

    host = os.environ.get("WEB_HOST", "0.0.0.0")
    port = int(os.environ.get("WEB_PORT", "8088"))
    ros_node.get_logger().info(
        f"web server listening on http://{host}:{port}  "
        f"(async={_ASYNC_MODE}, web_dir={web_dir})")
    try:
        sio.run(app, host=host, port=port,
                debug=False, use_reloader=False,
                allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
