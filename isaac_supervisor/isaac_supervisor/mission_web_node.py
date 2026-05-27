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
from typing import Dict, List, Optional

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
        # rover_namespaces — 멀티 rover 추적용. 빈 리스트 또는 [""] 면 단일
        # rover (absolute 토픽 sub). 예: ["rover_1", "rover_2"] 면 각 namespace
        # 의 토픽을 모두 sub 하고 payload 에 rover_id 필드를 박아 emit.
        self.declare_parameter("rover_namespaces", [""])
        nss_raw = self.get_parameter("rover_namespaces").value
        # ROS2 string_array param 이 빈 list 면 None 으로 들어오는 케이스 방어
        self.rover_namespaces: List[str] = (
            [str(s) for s in nss_raw] if nss_raw else [""]
        )
        # Per-rover cache so a late-connecting browser can be primed.
        # 단일 모드는 rover_id="" 키로 저장됨.
        self._last_state: Dict[str, dict] = {}
        self._last_odom: Dict[str, dict] = {}
        self._last_minimap: Dict[str, dict] = {}
        self._last_path: Dict[str, dict] = {}
        # explore path (coverage) 와 supervisor path (A* APPROACH/RTB) 를
        # 분리 캐싱. 같은 slot 공유 시 두 source 가 번갈아 덮어쓰며 미니맵
        # 경로가 진동했음 (2026-05-27 디버깅).
        self._last_path_explore: Dict[str, dict] = {}
        self._last_path_supervisor: Dict[str, dict] = {}
        self._last_target: Dict[str, Optional[dict]] = {}
        # target slot 도 path 와 동일 패턴 — explore (coverage markers) vs
        # supervisor (APPROACH/RTB) 분리 라우팅. 같은 slot 공유 시 EXPLORE
        # 중에도 supervisor 의 옛 mineral 별이 stale 하게 보이거나 두 source
        # 가 번갈아 진동.
        self._last_target_explore: Dict[str, Optional[dict]] = {}
        self._last_target_supervisor: Dict[str, Optional[dict]] = {}

        minimap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for ns in self.rover_namespaces:
            self._subscribe_for_rover(ns, minimap_qos)
        self.get_logger().info(
            f"mission_web_node ROS interface ready — "
            f"rover_namespaces={self.rover_namespaces}.")

    def _topic(self, ns: str, relative: str) -> str:
        """ns="" 이면 absolute /<relative>. 그 외엔 /<ns>/<relative>."""
        rel = relative.lstrip("/")
        if not ns:
            return "/" + rel
        return f"/{ns.strip('/')}/{rel}"

    def _subscribe_for_rover(self, ns: str, minimap_qos) -> None:
        """단일 rover (ns="" 또는 "rover_1" 등) 의 모든 토픽 sub."""
        from functools import partial
        self.create_subscription(
            MissionState, self._topic(ns, "mission/state"),
            partial(self._on_state, ns), 10)
        self.create_subscription(
            Odometry, self._topic(ns, "ground_truth/odom"),
            partial(self._on_odom, ns), SENSOR_QOS)
        self.create_subscription(
            Twist, self._topic(ns, "cmd_vel"),
            partial(self._on_cmd_vel, ns), 10)
        self.create_subscription(
            OccupancyGrid, self._topic(ns, "mission/minimap"),
            partial(self._on_minimap, ns), minimap_qos)
        self.create_subscription(
            Path, self._topic(ns, "mission/path"),
            partial(self._on_path, ns), 10)
        self.create_subscription(
            MarkerArray, self._topic(ns, "mission/markers"),
            partial(self._on_markers, ns), 10)
        self.create_subscription(
            Path, self._topic(ns, "supervisor/path"),
            partial(self._on_supervisor_path, ns), 10)
        self.create_subscription(
            PointStamped, self._topic(ns, "supervisor/target"),
            partial(self._on_supervisor_target, ns), 10)

    def topics_for_camera_urls(self) -> dict:
        return {
            "body":     str(self.get_parameter("body_cam_topic_for_url").value),
            "wrist":    str(self.get_parameter("wrist_cam_topic_for_url").value),
            "overview": str(self.get_parameter("overview_cam_topic_for_url").value),
            "chase":    str(self.get_parameter("chase_cam_topic_for_url").value),
        }

    def last_snapshot(self) -> dict:
        """Per-rover cache snapshot — dict-by-ns 형태.
        예: {"state": {"rover_1": {...}, "rover_2": {...}}, "odom": {...}, ...}
        client 가 connect 시 모든 rover 의 latest 값을 prime 받을 수 있다.
        """
        return {
            "rover_namespaces": list(self.rover_namespaces),
            "state":   dict(self._last_state),
            "odom":    dict(self._last_odom),
            "minimap": dict(self._last_minimap),
            "path":    dict(self._last_path),
            "target":  dict(self._last_target),
        }

    def _emit(self, event: str, ns: str, payload: dict) -> None:
        """ns 를 payload 의 rover_id 필드로 박아 emit. 단일 모드는 빈 문자열."""
        payload["rover_id"] = ns
        self._sio.emit(event, payload)

    def _on_state(self, ns: str, msg: MissionState) -> None:
        payload = {
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
        prev_cached_state = (
            self._last_state[ns]["state"] if ns in self._last_state else "")
        self._last_state[ns] = payload
        self._emit("state", ns, payload)
        # MISSION_COMPLETE 진입 시 미니맵 잔재 즉시 청소.
        # coverage_node 가 mission state 를 모르고 sector path/markers 를
        # 계속 발행하므로 backend 에서 차단해야 minimap 의 옛 sector anchor
        # 별 + 점선 path 가 사라짐 (2026-05-27 시연 캡처 디버깅).
        if payload["state"] == "MISSION_COMPLETE":
            self._last_path_explore[ns] = {"pts": []}
            self._last_path_supervisor[ns] = {"pts": []}
            self._last_target_explore[ns] = None
            self._last_target_supervisor[ns] = None
            self._emit_active_path(ns)
            self._emit_active_target(ns)
        # EXPLORE 진입 시 explore + supervisor slot 모두 청소.
        # supervisor 도 같이 비우는 이유: state msg 가 mission_manager 의
        # supervisor NaN msg 보다 먼저 backend 에 도달하는 race 가 있으면
        # supervisor 의 옛 mineral target 이 fallback 으로 쓰여 별이 옛
        # APPROACH 위치를 고수함 (2026-05-27 사용자 보고: PICK 후 EXPLORE
        # 재시작 시 별이 안 바뀌고 path 만 바뀌는 현상).
        elif payload["state"] == "EXPLORE" and prev_cached_state != "EXPLORE":
            self._last_path_explore[ns] = {"pts": []}
            self._last_path_supervisor[ns] = {"pts": []}
            self._last_target_explore[ns] = None
            self._last_target_supervisor[ns] = None
            self._emit_active_path(ns)
            self._emit_active_target(ns)

    def _on_odom(self, ns: str, msg: Odometry) -> None:
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        # vehicle_v3 의 GT_SCRIPT 는 PubGtOdom 에 position/orientation 만 보내고
        # twist 는 채우지 않는다 → 수신측 v.x/v.y 가 항상 0. position 변화율로
        # numerical velocity 계산 (per-rover prev cache).
        now_ns = self.get_clock().now().nanoseconds
        speed = 0.0
        prev_map = getattr(self, "_prev_odom_for_speed", None)
        if prev_map is None:
            prev_map = {}
            self._prev_odom_for_speed = prev_map
        prev = prev_map.get(ns)
        if prev is not None:
            dt = (now_ns - prev["ns"]) * 1e-9
            if dt > 0.02:
                dx = float(p.x) - prev["x"]
                dy = float(p.y) - prev["y"]
                speed = math.hypot(dx, dy) / dt
        prev_map[ns] = {"x": float(p.x), "y": float(p.y), "ns": now_ns}
        payload = {
            "x": float(p.x), "y": float(p.y), "z": float(p.z),
            "yaw_deg": math.degrees(yaw),
            "vx": float(v.x), "vy": float(v.y),
            "speed": float(speed),
            "wz": float(w.z),
        }
        self._last_odom[ns] = payload
        self._emit("odom", ns, payload)

    def _on_cmd_vel(self, ns: str, msg: Twist) -> None:
        self._emit("cmd_vel", ns, {
            "lin": float(msg.linear.x),
            "ang": float(msg.angular.z),
        })

    def _emit_active_path(self, ns: str) -> None:
        """Phase-aware path 라우터.

        Supervisor (mission_manager) 가 APPROACH/PICK_READY/RTB 시 path 를
        채워 발행하고 EXPLORE 진입 시 빈 path 를 발행한다. 따라서 supervisor
        path 가 non-empty 면 그게 active, 빈 path 면 coverage(EXPLORE) path
        가 active. 두 source 가 같은 slot 을 덮어쓰지 않도록 라우팅.
        """
        sup = self._last_path_supervisor.get(ns, {"pts": []})
        active = sup if sup.get("pts") else self._last_path_explore.get(
            ns, {"pts": []})
        self._last_path[ns] = active
        self._emit("path", ns, active)

    def _on_path(self, ns: str, msg: Path) -> None:
        # MISSION_COMPLETE 후엔 coverage 가 stale path 계속 보내도 무시.
        if self._last_state.get(ns, {}).get("state") == "MISSION_COMPLETE":
            return
        pts = [(float(ps.pose.position.x), float(ps.pose.position.y))
               for ps in msg.poses]
        self._last_path_explore[ns] = {"pts": pts}
        self._emit_active_path(ns)

    def _on_supervisor_path(self, ns: str, msg: Path) -> None:
        pts = [(float(ps.pose.position.x), float(ps.pose.position.y))
               for ps in msg.poses]
        # supervisor 가 빈 path 발행하면 (EXPLORE 시) supervisor slot 비워서
        # _emit_active_path 가 coverage path 로 fallback.
        self._last_path_supervisor[ns] = {"pts": pts}
        self._emit_active_path(ns)

    def _emit_active_target(self, ns: str) -> None:
        """supervisor 가 채운 target 우선, 비어있으면 explore (coverage marker)
        target fallback. path 와 같은 분리 라우팅 패턴."""
        sup = self._last_target_supervisor.get(ns)
        active = sup if sup else self._last_target_explore.get(ns)
        self._last_target[ns] = active
        self._emit("target", ns, dict(active) if active else {})

    def _on_supervisor_target(self, ns: str, msg: PointStamped) -> None:
        import math as _math
        x, y = float(msg.point.x), float(msg.point.y)
        if not (_math.isfinite(x) and _math.isfinite(y)):
            # EXPLORE 진입 시 mission_manager 가 NaN 발행 → supervisor slot 비움.
            self._last_target_supervisor[ns] = None
        else:
            self._last_target_supervisor[ns] = {"x": x, "y": y}
        self._emit_active_target(ns)

    def _on_markers(self, ns: str, msg: MarkerArray) -> None:
        # MISSION_COMPLETE 후엔 coverage 의 stale anchor marker 무시.
        if self._last_state.get(ns, {}).get("state") == "MISSION_COMPLETE":
            return
        # minimap_publisher 의 ns="target" SPHERE 만 추려서 EXPLORE 의 sector
        # anchor 별 위치로. APPROACH/RTB 중엔 supervisor 가 채워둔 게 우선.
        target = None
        for m in msg.markers:
            if m.ns == "target":
                if m.action == 2:
                    target = None
                else:
                    target = {"x": float(m.pose.position.x),
                              "y": float(m.pose.position.y)}
                break
        self._last_target_explore[ns] = target
        self._emit_active_target(ns)

    def _on_minimap(self, ns: str, msg: OccupancyGrid) -> None:
        # OccupancyGrid -> compact dict the browser can paint on a canvas.
        payload = {
            "w": int(msg.info.width),
            "h": int(msg.info.height),
            "res": float(msg.info.resolution),
            "ox": float(msg.info.origin.position.x),
            "oy": float(msg.info.origin.position.y),
            "data": list(msg.data),
        }
        self._last_minimap[ns] = payload
        self._emit("minimap", ns, payload)


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
            # 시연 단계: wrist 슬롯의 stream 을 body(nav) YOLO 로 변경
            # (cam_wrist 변수명은 viewport-wrist class/css 호환 위해 유지).
            cam_wrist=mjpeg(topics["body"]),
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
        # rover namespace 리스트 먼저 — JS 가 selector UI 구성 시 사용
        sio.emit("rovers", {"namespaces": snap["rover_namespaces"]})
        # Per-rover cache 를 ns 별로 re-emit. payload 마다 rover_id 박혀있음
        # (사실 캐시 시점 emit 의 payload 와 동일). 늦게 들어온 client 도
        # 모든 rover 의 latest state 받음.
        for event_key in ("state", "odom", "minimap", "path", "target"):
            for ns, payload in snap[event_key].items():
                if payload is None:
                    continue
                p = dict(payload)
                p["rover_id"] = ns
                sio.emit(event_key, p)

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
