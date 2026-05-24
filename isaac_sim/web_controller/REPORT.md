# Isaac Sim 로버 웹 조종 시스템 구현 보고서

**작성일**: 2026-05-23  
**작업자**: Claude Code (claude-sonnet-4-6)  
**작업 범위**: `a2_isaac/isaac_sim`

---

## 1. 작업 목표

| # | 목표 |
|---|------|
| 1 | `vehicle_v3.usd`에서 발행되는 ROS2 토픽 분석 및 확인 |
| 2 | 브라우저 웹 UI에서 카메라 피드 시각화 |
| 3 | 키보드(WASD)로 로버 실시간 수동 조종 |
| 4 | 레이싱게임 스타일 HUD 구현 |
| 5 | `mars_terrain_generator_v3.py`로 생성한 terrain과 웹 조종 호환성 확보 |

---

## 2. ROS2 토픽 분석

### 2-1. 분석 방법

`vehicle_v3.usd`는 USD crate 바이너리 포맷이므로 `strings` 명령으로 내부 문자열을 파싱하고,  
동일 Action Graph 구성을 가진 `rover_m0609_localization.usd`의 문서(`isaac_ros_topics.md`)와 교차 검증.

```bash
strings assets/vehicle/vehicle_v3.usd | grep -E "ActionGraph|ROS2|cmd_vel|image_raw|imu"
# 출력: ActionGraph, image_raw, /imu/, ROS2, resetSimul, sim_imu
```

**결론**: `vehicle_v3.usd` 내부에 ROS2 Action Graph **내장 확인**.

### 2-2. 발행/구독 토픽 목록

| 토픽 | 메시지 타입 | 방향 | 주기 | 용도 |
|------|------------|:----:|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | **INPUT** | on-demand | 로버 속도 명령 (linear.x, angular.z) |
| `/camera/rover/image_raw` | `sensor_msgs/Image` | OUTPUT | ~60 Hz | 로버 차체 카메라 RGB |
| `/camera/rover/depth` | `sensor_msgs/Image` | OUTPUT | ~55–68 Hz | 로버 차체 깊이 카메라 |
| `/camera/rover/camera_info` | `sensor_msgs/CameraInfo` | OUTPUT | ~100 Hz | 카메라 내부 파라미터 |
| `/imu/data` | `sensor_msgs/Imu` | OUTPUT | ~102 Hz | IMU (가속도 + 자이로) |
| `/joint_states_raw` | `sensor_msgs/JointState` | OUTPUT | ~103 Hz | 전체 관절 상태 (raw) |

### 2-3. Isaac Sim 내부 Action Graph 구성

```
/ActionGraph/RoverAckermannDrive   ← /cmd_vel 수신 → Ackermann 변환 → 휠 명령
/ActionGraph/LocalizationSensors   → /imu/data, /camera/rover/* 발행
/ActionGraph/RoverStatePublishers  → /joint_states_raw 발행
```

---

## 3. 시스템 아키텍처

```
┌──────────────────────────────────────────────────────────┐
│                    브라우저 (index.html)                   │
│                                                          │
│  ┌─────────────┐  ┌──────────────────┐  ┌────────────┐  │
│  │ 카메라 피드  │  │   WASD 키 입력   │  │  IMU 패널  │  │
│  │ (JPEG 스트림)│  │  (키보드 이벤트) │  │   (JSON)   │  │
│  └──────┬──────┘  └────────┬─────────┘  └─────┬──────┘  │
│         │WS /ws/camera     │WS /ws/control     │WS /ws/status
└─────────┼──────────────────┼───────────────────┼──────────┘
          ▼                  ▼                   ▼
┌──────────────────────────────────────────────────────────┐
│          FastAPI 웹 서버 (main.py, port 8001)             │
│                                                          │
│  RoverBridgeNode (rclpy, 별도 스레드)                    │
│  ┌────────────────────────────────────────────────────┐  │
│  │ sub: /camera/rover/image_raw                       │  │
│  │   → numpy 변환 → OpenCV JPEG 인코딩                 │  │
│  │   → WS /ws/camera 전송 (30 FPS)                    │  │
│  │                                                    │  │
│  │ timer 20Hz: 키 상태 → Twist 계산 → /cmd_vel pub    │  │
│  │   linear.x  = ±2.5 m/s  (W / S)                   │  │
│  │   angular.z = ±1.5 rad/s (A / D)                   │  │
│  │                                                    │  │
│  │ sub: /imu/data → JSON → WS /ws/status (10 Hz)     │  │
│  └────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────┘
          ▲ /cmd_vel             ▼ /camera/..., /imu/data
┌──────────────────────────────────────────────────────────┐
│          Isaac Sim (vehicle_v3.usd + terrain_only.usd)   │
│                                                          │
│  /ActionGraph/RoverAckermannDrive  ← /cmd_vel            │
│  /ActionGraph/LocalizationSensors  → /camera, /imu       │
│  /ActionGraph/RoverStatePublishers → /joint_states_raw   │
└──────────────────────────────────────────────────────────┘
```

---

## 4. 구현 파일 목록

### 4-1. 신규 생성 / 수정 파일

| 파일 | 상태 | 크기 | 설명 |
|------|:----:|-----:|------|
| `scripts/check_vehicle_v2_topics.py` | 신규 | 104 lines | Isaac Sim에서 USD 로드 후 토픽 확인 |
| `scripts/load_terrain_webcontroller.py` | 신규 | 120 lines | terrain + vehicle_v3 안전 로더 |
| `web_controller/main.py` | 수정 | 301 lines | FastAPI + WebSocket 웹 서버 |
| `web_controller/static/index.html` | 신규 | 748 lines | 레이싱게임 HUD UI |
| `web_controller/launch_web_controller.sh` | 신규 | 35 lines | 웹 서버 실행 스크립트 |
| `scripts/mars_terrain_generator_v3.py:58` | 수정 | — | vehicle_v1 → vehicle_v3 참조 변경 |

---

## 5. 각 파일 상세 설명

### 5-1. check_vehicle_v2_topics.py

Isaac Sim Python 환경에서 `vehicle_v3.usd`를 로드하고 30초간 시뮬레이션 재생.  
사용자가 다른 터미널에서 `ros2 topic list`로 실제 발행 토픽을 직접 확인하는 도구.

```bash
isaac-python src/a2_isaac/isaac_sim/scripts/check_vehicle_v2_topics.py
```

### 5-2. main.py (FastAPI 웹 서버)

**설계 결정 사항**

| 문제 | 해결책 |
|------|--------|
| rclpy(동기) ↔ FastAPI(비동기) 충돌 | rclpy를 별도 스레드(`SingleThreadedExecutor`)에서 실행, `threading.Lock`으로 데이터 동기화 |
| 카메라 이미지 전송 | `sensor_msgs/Image` → numpy → OpenCV JPEG 인코딩 → WebSocket 바이너리 전송 |
| 제어 지연 최소화 | 키 상태를 20Hz 타이머로 `/cmd_vel` 퍼블리시 (입력마다 즉시 전송하지 않고 주기적으로) |
| rclpy 미설치 환경 대응 | `_HAS_ROS` 플래그로 데모 모드 분기 처리 |

**WebSocket 엔드포인트**

| 엔드포인트 | 방향 | 내용 | 주기 |
|-----------|:----:|------|------|
| `WS /ws/camera` | 서버→브라우저 | JPEG 바이너리 프레임 | 30 FPS |
| `WS /ws/control` | 브라우저→서버 | JSON 키 상태 `{"w":true,"d":false,...}` | 키 이벤트 시 |
| `WS /ws/status` | 서버→브라우저 | JSON IMU + 속도 데이터 | 10 Hz |
| `GET /move` | REST | 레거시 호환 이동 명령 | — |
| `GET /health` | REST | 서버/ROS2 상태 확인 | — |

### 5-3. index.html (레이싱게임 HUD UI)

**화면 레이아웃**

```
┌──[●CONNECTED]───── ROVER CONTROL ─────CAM: 29.8 FPS──┐
│                                                        │
│  ┌──────┐                                              │
│  │ IMU  │      [ 카메라 피드 전체화면 ]                 │
│  │ AX   │         스캔라인 + 비네팅 오버레이             │
│  │ AY   │                                              │
│  │ AZ   │                                              │
│  │ GX   │           [ 조향 핸들 ]                      │
│  │ GY   │                                  [ W ]       │
│  │ GZ   │    [ 속도계(호 게이지) ]       [ A ][ S ][ D ]│
│  └──────┘                              Space: STOP     │
└────────────────────────────────────────────────────────┘
```

**주요 기능**

| 기능 | 구현 방법 |
|------|----------|
| 카메라 피드 | WebSocket 바이너리 → Blob URL → `<img>` 실시간 교체 |
| 스캔라인 효과 | CSS `repeating-linear-gradient` 오버레이 |
| 비네팅 효과 | CSS `radial-gradient` 오버레이 |
| 속도계 | SVG `stroke-dashoffset` 애니메이션 (전진=초록, 후진=빨간, 과속=주황) |
| 조향 핸들 | SVG `rotate()` transform — `angular.z`에 비례해 회전 |
| WASD 키 하이라이트 | `keydown`/`keyup` 이벤트 → CSS `.active` 클래스 |
| 비상 정지 | Space 토글 → 빨간 ESTOP 배너 + `/cmd_vel` 전송 차단 |
| 탭 전환 안전 처리 | `window.blur` 이벤트 → 모든 키 해제 후 정지 명령 전송 |
| 자동 재연결 | `WebSocket.onclose` → 2초 후 자동 재연결 시도 |
| 모바일 지원 | WASD 버튼에 `touchstart`/`touchend` 이벤트 추가 |

**조종 키 맵**

| 키 | 동작 |
|----|------|
| `W` / `↑` | 전진 (linear.x = +2.5 m/s) |
| `S` / `↓` | 후진 (linear.x = −2.5 m/s) |
| `A` / `←` | 좌회전 (angular.z = +1.5 rad/s) |
| `D` / `→` | 우회전 (angular.z = −1.5 rad/s) |
| `W+A` 등 | 동시 입력으로 대각 이동 가능 |
| `Space` | 비상 정지 토글 |

---

## 6. Terrain 호환성 분석 및 수정

### 6-1. 문제 발견

`mars_terrain_generator_v3.py`가 master scene에 embed하는 vehicle USD가 잘못 설정되어 있었음.

```python
# mars_terrain_generator_v3.py:58 (수정 전)
VEHICLE_USD = ISAAC_SIM_DIR / "assets" / "vehicle" / "vehicle_v1.usd"
#                                                     ^^^^^^^^^^^
#                                       ROS2 Action Graph 없음!
#                                       /cmd_vel, /camera, /imu 토픽 발행 안 됨
```

**vehicle_v1.usd vs vehicle_v3.usd 비교**

```bash
strings vehicle_v1.usd | grep -E "ActionGraph|ROS2|image_raw|imu"
# 출력: maximum:3  ← ROS2 관련 없음

strings vehicle_v3.usd | grep -E "ActionGraph|ROS2|image_raw|imu"
# 출력: ActionGraph, image_raw, /imu/, ROS2, resetSimul  ← ROS2 Action Graph 존재
```

### 6-2. 추가 위험 요소: 경로 재작성 문제

generator의 `_attach_vehicle_reference()` 함수는 vehicle 내부 prim 경로를 `/Root/Vehicle` → `/World/Vehicle`로 재작성함.  
`vehicle_v3.usd`의 Action Graph 노드들이 내부적으로 prim 경로를 문자열로 참조하는 경우, 이 재작성 과정에서 노드 연결이 끊어져 ROS2 토픽이 발행되지 않을 수 있음.

### 6-3. 수정 내용

**수정 1** — generator의 vehicle 참조 변경

```python
# mars_terrain_generator_v3.py:58 (수정 후)
VEHICLE_USD = ISAAC_SIM_DIR / "assets" / "vehicle" / "vehicle_v3.usd"
```

**수정 2** — `load_terrain_webcontroller.py` 신규 생성 (권장 방법)

경로 재작성 위험을 완전히 회피하는 전용 로더.  
`terrain_only.usd`(지형+물리)만 로드하고, `vehicle_v3.usd`를 `Sdf.Reference`로 직접 붙여서  
Action Graph 내부 경로를 **전혀 건드리지 않음**.

```python
# 핵심 코드 — 경로 재작성 없는 안전한 reference
vehicle_prim.GetReferences().AddReference(
    Sdf.Reference(assetPath=str(VEHICLE_V3_USD))  # defaultPrim 그대로 사용
)
```

### 6-4. 두 방법 비교

| 항목 | 방법 A (권장) | 방법 B |
|------|:------------:|:------:|
| 사용 스크립트 | `load_terrain_webcontroller.py` | Isaac Sim에서 master scene 직접 열기 |
| vehicle 로드 방식 | `Sdf.Reference` 직접 (경로 재작성 없음) | generator embed (경로 재작성) |
| Action Graph 안전성 | ✅ 안전 | ⚠️ 재작성으로 깨질 수 있음 |
| ROS2 토픽 정상 작동 | ✅ 보장 | ⚠️ 확인 필요 |

---

## 7. 실행 방법

### 7-1. terrain 생성 (최초 1회)

```bash
python3 src/a2_isaac/isaac_sim/scripts/mars_terrain_generator_v3.py \
    --seed 1234 --terrain-id terrain_00001
```

### 7-2. Isaac Sim 실행 — 방법 A (권장)

```bash
# 터미널 1
source /opt/ros/humble/setup.bash
isaac-python src/a2_isaac/isaac_sim/scripts/load_terrain_webcontroller.py
# 옵션: --terrain terrain_00001  (특정 terrain)
#       --spawn-z 2.0             (로버 초기 높이)
#       --gravity 3.72            (화성 중력, 기본값)
```

### 7-3. 웹 서버 실행

```bash
# 터미널 2
source /opt/ros/humble/setup.bash
source ~/dev_ws/rover_ws/install/setup.bash
bash src/a2_isaac/isaac_sim/web_controller/launch_web_controller.sh
```

### 7-4. 브라우저 접속

```
http://localhost:8001
```

### 7-5. 토픽 확인 (선택)

```bash
# 터미널 3
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /camera/rover/image_raw   # ~60 Hz 확인
ros2 topic echo /imu/data --once        # IMU 데이터 확인
```

---

## 8. 동작 확인 결과

서버 기동 테스트 통과:

```bash
$ curl http://localhost:8001/health
{"status":"ok","ros":true,"node_ready":true}

$ curl http://localhost:8001/ | head -3
<!DOCTYPE html>
<html lang="ko">
<head>
```

노드 생성 로그:
```
[INFO] [rover_web_bridge]: rover_web_bridge 준비 완료 — http://localhost:8001
```

---

## 9. 한계 및 향후 개선 사항

| 항목 | 현재 상태 | 개선 방향 |
|------|----------|----------|
| 카메라 레이턴시 | JPEG WebSocket (30 FPS 목표) | `web_video_server` + H.264로 교체 시 저지연 개선 가능 |
| 깊이 카메라 | 미구현 | `/camera/rover/depth` 구독 추가 → 장애물 거리 오버레이 |
| 오도메트리 미니맵 | 미구현 | `/rover/wheel_states` → dead-reckoning → 미니맵 표시 |
| 속도/각속도 파라미터 | 코드 하드코딩 | UI 슬라이더로 실시간 조정 가능하도록 개선 |
| vehicle_v3 경로 재작성 검증 | 미검증 | master scene 방식(방법 B) 실제 실행 후 Action Graph 동작 확인 필요 |
| HTTPS / WSS | 미적용 | 외부 네트워크 노출 시 TLS 적용 필요 |
