#!/usr/bin/env python3
"""Racing game web controller — Isaac Sim rover.

WebSocket 기반으로 ROS2 토픽을 브라우저에 연결한다:
  /camera/rover/image_raw  →  WS /ws/camera   (JPEG 바이너리 스트림)
  브라우저 WASD           →  WS /ws/control  →  /cmd_vel (Twist)
  /imu/data               →  WS /ws/status   (JSON 10 Hz)

실행:
  # ROS2 환경 포함 (vehicle_v2_scene.usd 가 Isaac Sim 에서 실행 중이어야 함)
  source /opt/ros/humble/setup.bash
  source ~/dev_ws/rover_ws/install/setup.bash
  cd src/a2_isaac/isaac_sim/web_controller
  python3 main.py

  # 브라우저: http://localhost:8001
"""
from __future__ import annotations

import asyncio
import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np

try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Image, Imu
    _HAS_ROS = True
except ImportError:
    _HAS_ROS = False

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse

try:
    import uvicorn
except ImportError:
    uvicorn = None

# ── 파라미터 ──────────────────────────────────────────────────────────────────
MAX_LINEAR_VEL  = 2.5   # m/s — 최대 전진 속도
MAX_ANGULAR_VEL = 1.5   # rad/s — 최대 회전 속도
CAMERA_FPS      = 30    # 목표 카메라 프레임레이트
JPEG_QUALITY    = 80    # JPEG 압축 품질 (0-100)
SERVER_PORT     = 8001

# ── ROS2 브리지 노드 ──────────────────────────────────────────────────────────

class RoverBridgeNode(Node if _HAS_ROS else object):
    def __init__(self):
        if _HAS_ROS:
            super().__init__("rover_web_bridge")
        self._lock = threading.Lock()
        self._latest_jpeg: bytes | None = None
        self._frame_id: int = 0          # 새 프레임마다 증가 — 중복 전송 방지
        self._speed = 0.0
        self._angular = 0.0
        self._key_state: dict = {}
        self._imu_data: dict = {
            "ax": 0.0, "ay": 0.0, "az": 0.0,
            "gx": 0.0, "gy": 0.0, "gz": 0.0,
        }
        self._cam_fps = 0.0
        self._frame_count = 0
        self._fps_reset_time = time.time()
        # 속도 smoothing — 순간 반전이 PhysX 불안정 유발 방지
        self._cur_linear  = 0.0
        self._cur_angular = 0.0
        self._LINEAR_ACCEL  = 4.0   # m/s²  — 가속/감속 기울기
        self._ANGULAR_ACCEL = 3.0   # rad/s²

        if _HAS_ROS:
            # Isaac Sim ROS2 Bridge는 BEST_EFFORT QoS로 발행 —
            # subscriber도 동일하게 맞춰야 메시지가 전달됨.
            _sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.create_subscription(
                Image, "/camera/rover/image_raw", self._on_image, _sensor_qos)
            self.create_subscription(
                Imu, "/imu/data", self._on_imu, _sensor_qos)
            # 20 Hz cmd_vel 발행
            self.create_timer(0.05, self._publish_cmd)
            self.get_logger().info(
                f"rover_web_bridge 준비 완료 — http://localhost:{SERVER_PORT}")

    # ── 카메라 콜백 ──────────────────────────────────────────────────────────
    def _on_image(self, msg: Image):
        enc = msg.encoding.lower()
        try:
            if enc in ("rgb8",):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif enc in ("bgr8",):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
            elif enc in ("rgba8",):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 4)
                arr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
            elif enc in ("bgra8",):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 4)
                arr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
            else:
                return
        except Exception:
            return

        _, buf = cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        jpeg = buf.tobytes()

        now = time.time()
        with self._lock:
            self._latest_jpeg = jpeg
            self._frame_id += 1          # 새 프레임 도착 표시
            self._frame_count += 1
            elapsed = now - self._fps_reset_time
            if elapsed >= 1.0:
                self._cam_fps = self._frame_count / elapsed
                self._frame_count = 0
                self._fps_reset_time = now

    # ── IMU 콜백 ─────────────────────────────────────────────────────────────
    def _on_imu(self, msg: Imu):
        with self._lock:
            self._imu_data = {
                "ax": round(msg.linear_acceleration.x, 3),
                "ay": round(msg.linear_acceleration.y, 3),
                "az": round(msg.linear_acceleration.z, 3),
                "gx": round(msg.angular_velocity.x, 3),
                "gy": round(msg.angular_velocity.y, 3),
                "gz": round(msg.angular_velocity.z, 3),
            }

    # ── 키보드 상태 설정 (WebSocket에서 호출) ─────────────────────────────────
    def set_keys(self, key_state: dict):
        with self._lock:
            self._key_state = key_state

    # ── 20 Hz cmd_vel 발행 (속도 smoothing 적용) ─────────────────────────────
    def _publish_cmd(self):
        dt = 0.05  # 20 Hz 타이머 주기
        with self._lock:
            keys = dict(self._key_state)

        # 목표 속도 계산
        target_linear = 0.0
        target_angular = 0.0
        if keys.get("w") or keys.get("ArrowUp"):
            target_linear += MAX_LINEAR_VEL
        if keys.get("s") or keys.get("ArrowDown"):
            target_linear -= MAX_LINEAR_VEL
        if keys.get("a") or keys.get("ArrowLeft"):
            target_angular += MAX_ANGULAR_VEL
        if keys.get("d") or keys.get("ArrowRight"):
            target_angular -= MAX_ANGULAR_VEL

        # 가속/감속 기울기 제한 — 순간 반전 방지
        max_dl = self._LINEAR_ACCEL  * dt
        max_da = self._ANGULAR_ACCEL * dt
        dl = target_linear  - self._cur_linear
        da = target_angular - self._cur_angular
        self._cur_linear  += max(-max_dl, min(max_dl, dl))
        self._cur_angular += max(-max_da, min(max_da, da))

        # 정지 명령 시 즉시 0으로 스냅 (드리프트 방지)
        if target_linear == 0.0 and abs(self._cur_linear) < 0.05:
            self._cur_linear = 0.0
        if target_angular == 0.0 and abs(self._cur_angular) < 0.05:
            self._cur_angular = 0.0

        if _HAS_ROS:
            msg = Twist()
            msg.linear.x  = self._cur_linear
            msg.angular.z = self._cur_angular
            self.cmd_pub.publish(msg)
        with self._lock:
            self._speed   = self._cur_linear
            self._angular = self._cur_angular

    # ── 데이터 접근자 ─────────────────────────────────────────────────────────
    def get_jpeg(self) -> tuple[bytes | None, int]:
        with self._lock:
            return self._latest_jpeg, self._frame_id

    def get_status(self) -> dict:
        with self._lock:
            return {
                **self._imu_data,
                "speed": round(self._speed, 2),
                "angular": round(self._angular, 2),
                "cam_fps": round(self._cam_fps, 1),
            }


# ── ROS2 스레드 ───────────────────────────────────────────────────────────────

_node: RoverBridgeNode | None = None


def _ros_thread_fn():
    global _node
    if not _HAS_ROS:
        print("[warn] rclpy 없음 — 데모 모드로 실행")
        _node = RoverBridgeNode()
        return
    rclpy.init()
    _node = RoverBridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(_node)
    try:
        executor.spin()
    except Exception as e:
        print(f"[ros] executor 종료: {e}")
    finally:
        _node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


_ros_thread = threading.Thread(target=_ros_thread_fn, daemon=True)
_ros_thread.start()

# 노드가 생성될 때까지 최대 3초 대기
for _ in range(30):
    if _node is not None:
        break
    time.sleep(0.1)

# ── FastAPI 앱 ────────────────────────────────────────────────────────────────

app = FastAPI(title="Rover Web Controller")

STATIC_DIR = Path(__file__).resolve().parent / "static"
STATIC_DIR.mkdir(exist_ok=True)

# ── WebSocket: 카메라 스트림 ──────────────────────────────────────────────────

@app.websocket("/ws/camera")
async def ws_camera(ws: WebSocket):
    await ws.accept()
    interval = 1.0 / CAMERA_FPS
    last_sent_id = -1
    last_send_time = 0.0
    loop = asyncio.get_running_loop()
    debug_timer = loop.time()
    debug_sent = 0
    try:
        while True:
            t0 = loop.time()
            if _node is not None:
                jpeg, frame_id = _node.get_jpeg()
                now = loop.time()
                # 새 프레임이 있거나, 1초 이상 전송이 없으면 keepalive로 재전송
                if jpeg and (frame_id != last_sent_id or now - last_send_time > 1.0):
                    try:
                        await ws.send_bytes(jpeg)
                        last_sent_id = frame_id
                        last_send_time = now
                        debug_sent += 1
                    except WebSocketDisconnect:
                        return
                    except Exception:
                        return
            # 5초마다 수신 프레임 수 출력
            if loop.time() - debug_timer >= 5.0:
                fid = _node._frame_id if _node else -1
                print(f"[cam] ROS 수신 frame_id={fid}  WS 전송={debug_sent}회/5s")
                debug_sent = 0
                debug_timer = loop.time()
            elapsed = loop.time() - t0
            await asyncio.sleep(max(0.005, interval - elapsed))
    except WebSocketDisconnect:
        pass

# ── WebSocket: 키보드 컨트롤 ──────────────────────────────────────────────────

@app.websocket("/ws/control")
async def ws_control(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            raw = await ws.receive_text()
            if _node is not None:
                try:
                    _node.set_keys(json.loads(raw))
                except Exception:
                    pass
    except WebSocketDisconnect:
        if _node is not None:
            _node.set_keys({})   # 연결 끊기면 정지

# ── WebSocket: 상태 스트림 (10 Hz) ───────────────────────────────────────────

@app.websocket("/ws/status")
async def ws_status(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            await asyncio.sleep(0.1)
            if _node is not None:
                try:
                    await ws.send_text(json.dumps(_node.get_status()))
                except Exception:
                    break
    except WebSocketDisconnect:
        pass

# ── REST: 단순 이동 명령 (레거시 호환) ───────────────────────────────────────

@app.get("/move")
def move(linear: float = 0.0, angular: float = 0.0):
    if _node is not None and _HAS_ROS:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        _node.cmd_pub.publish(msg)
    return {"ok": True, "linear": linear, "angular": angular}

@app.get("/health")
def health():
    return {"status": "ok", "ros": _HAS_ROS, "node_ready": _node is not None}

@app.get("/debug")
def debug():
    if _node is None:
        return {"error": "node not ready"}
    with _node._lock:
        return {
            "frame_id": _node._frame_id,
            "cam_fps": round(_node._cam_fps, 1),
            "has_jpeg": _node._latest_jpeg is not None,
            "jpeg_bytes": len(_node._latest_jpeg) if _node._latest_jpeg else 0,
        }

# ── 정적 파일 서빙 ────────────────────────────────────────────────────────────

app.mount("/", StaticFiles(directory=str(STATIC_DIR), html=True))

# ── 진입점 ────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    if uvicorn is None:
        raise RuntimeError("pip install uvicorn")
    print(f"\n{'='*60}")
    print(f"  Rover Web Controller 시작")
    print(f"  http://localhost:{SERVER_PORT}")
    print(f"  ROS2 환경: {'✓' if _HAS_ROS else '✗ (데모 모드)'}")
    print(f"{'='*60}\n")
    uvicorn.run(app, host="0.0.0.0", port=SERVER_PORT, log_level="warning")
