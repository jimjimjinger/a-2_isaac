# Isaac Sim 로버 웹 조종 시스템 — 세션 보고서

**작성일**: 2026-05-24  
**작업자**: Claude Code (claude-sonnet-4-6)  
**작업 범위**: `a2_isaac/isaac_sim/web_controller/`

---

## 0. 시스템 기술 스택

### 웹 서버 구성

| 라이브러리 | 역할 |
|-----------|------|
| **FastAPI** | REST API + WebSocket 서버 프레임워크 |
| **uvicorn** | ASGI 웹 서버 (FastAPI 실행 엔진) |
| **rclpy** | ROS2 Python 클라이언트 |
| **MultiThreadedExecutor** | ROS2 콜백 병렬 처리 (4스레드) |
| **OpenCV (cv2)** | ROS2 Image 메시지 → JPEG 인코딩 |
| **numpy** | 픽셀 배열 변환 |

### 서버 내부 구조

```
FastAPI (비동기, asyncio)  ─────────────────────────────
    ├── WS /ws/camera      ← JPEG 카메라 스트림 (30 FPS)
    ├── WS /ws/control     ← 브라우저 키 입력 수신
    ├── WS /ws/status      ← IMU / 속도 데이터 (10 Hz)
    ├── GET /health        ← 서버 상태 확인
    ├── GET /debug         ← 카메라 수신 프레임 확인
    └── Static /           ← index.html 정적 파일 서빙

별도 스레드 (threading.Thread)
    └── rclpy MultiThreadedExecutor (4스레드)
            ├── sub: /camera/rover/image_raw
            ├── sub: /imu/data
            ├── pub: /cmd_vel
            └── timer 20Hz: 속도 smoothing → /cmd_vel 발행
```

> FastAPI(비동기)와 rclpy(동기)의 충돌을 피하기 위해 rclpy를 별도 스레드에서 실행하고  
> `threading.Lock`으로 데이터를 공유하는 구조.

### 전체 통신 구조

```
Isaac Sim (PC 로컬)
  vehicle_v3.usd + terrain_only.usd
  ROS2 Action Graph
    /cmd_vel 수신 → 휠 구동
    /camera/rover/image_raw 발행 (60 Hz)
    /imu/data 발행 (102 Hz)
        │
        │ ROS2 토픽 (PC 내부)
        ▼
웹 서버 main.py  192.168.10.66:8001
  RoverBridgeNode (rclpy)
    이미지 → JPEG → WS /ws/camera
    키 상태 → 속도 smoothing → /cmd_vel
    IMU → JSON → WS /ws/status
        │
        │ Wi-Fi WebSocket (192.168.10.x 대역)
        ▼
브라우저 / 핸드폰
  http://192.168.10.66:8001
```

### 조종 신호 흐름

```
핸드폰 터치 / PC 키보드
    ↓ WS /ws/control  (키 이벤트 발생 시)
웹 서버 key_state 업데이트
    ↓ 20Hz 타이머 + 속도 smoothing (4 m/s² 기울기)
/cmd_vel (geometry_msgs/Twist) 발행
    ↓ ROS2 토픽
Isaac Sim 휠 구동 → 로버 이동
    ↓
카메라 새 프레임 생성
    ↓ /camera/rover/image_raw
웹 서버 JPEG 인코딩
    ↓ WS /ws/camera  (새 프레임 도착 시만 전송)
브라우저 화면 업데이트 (onload 후 blob URL 해제)
```

---

## 1. 분석 요약

### 1-1. `mars_terrain_generator_v2.py` ↔ `vehicle_v3.usd` 관계

`mars_terrain_generator_v2.py`는 **vehicle USD를 참조하지 않는다**.  
생성기는 terrain 전용 출력물만 만들며, vehicle은 별도 스크립트에서 로드된다.

| 파일 | 역할 |
|------|------|
| `mars_terrain_generator_v2.py` | terrain_only.usd / rocks_merged.usd / meta.json 생성 |
| `load_terrain_webcontroller.py` | **vehicle_v3.usd** 메인 스테이지 + terrain sublayer 로드 |

`load_terrain_webcontroller.py:30`에서 `vehicle_v3.usd` 경로 명시:
```python
VEHICLE_V3_USD = A2_ROOT / "isaac_sim" / "assets" / "vehicle" / "vehicle_v3.usd"
```

### 1-2. 시스템 토픽 구조

```
Isaac Sim (vehicle_v3.usd)
  발행: /camera/rover/image_raw  (~60 Hz, RGB)
  발행: /imu/data                (~102 Hz)
  발행: /joint_states_raw        (~103 Hz)
  구독: /cmd_vel                 (geometry_msgs/Twist)

웹 서버 (main.py, FastAPI)
  WS /ws/camera   ← /camera/rover/image_raw → JPEG → 브라우저
  WS /ws/control  ← 브라우저 WASD → /cmd_vel
  WS /ws/status   ← /imu/data → JSON 10Hz → 브라우저 HUD

브라우저 / 핸드폰
  http://192.168.1.5:8001
```

---

## 2. 발견된 버그 및 수정 내역

### 버그 1 — 카메라 화면 고정 현상 (blob URL race condition)

**증상**: 브라우저에서 카메라 첫 프레임만 표시되고 이후 화면이 고정됨  
**원인**: `index.html`에서 이전 blob URL을 브라우저가 JPEG 디코딩을 완료하기 전에 해제  
**위치**: `static/index.html` `camWs.onmessage` 핸들러

```javascript
// 수정 전 — 즉시 해제 (race condition)
const old = camImg.src;
camImg.src = url;
URL.revokeObjectURL(old);   // 브라우저가 디코딩 중에 URL 해제 → 렌더링 실패

// 수정 후 — onload 완료 후 해제
camImg.onload = () => {
  if (prev && prev.startsWith('blob:')) URL.revokeObjectURL(prev);
};
camImg.onerror = () => { URL.revokeObjectURL(url); };
camImg.src = url;
```

---

### 버그 2 — 동일 프레임 반복 전송

**증상**: Isaac Sim에서 새 프레임이 없어도 같은 JPEG를 30FPS로 계속 전송  
**수정**: `frame_id` 카운터 추가, 새 프레임 도착 시에만 전송 (keepalive 1초 fallback 포함)

```python
# main.py — RoverBridgeNode
self._frame_id: int = 0          # _on_image 호출마다 증가

# ws_camera WebSocket 핸들러
if jpeg and (frame_id != last_sent_id or now - last_send_time > 1.0):
    await ws.send_bytes(jpeg)
    last_sent_id = frame_id
    last_send_time = now
```

---

### 버그 3 — SingleThreadedExecutor 병목

**증상**: 카메라 콜백 처리 중 다른 콜백(IMU, cmd_vel 타이머)이 블록됨  
**수정**: `MultiThreadedExecutor(num_threads=4)` 으로 교체

```python
# 수정 전
executor = SingleThreadedExecutor()

# 수정 후
executor = MultiThreadedExecutor(num_threads=4)
```

---

### 버그 4 — `asyncio.get_event_loop()` deprecated

**위치**: `ws_camera` 핸들러  
**수정**: `asyncio.get_running_loop()` 으로 교체

---

## 3. 추가된 기능

### 디버그 엔드포인트 `/debug`

Isaac Sim 카메라 수신 상태를 실시간으로 확인할 수 있는 REST 엔드포인트 추가.

```
GET http://localhost:8001/debug
→ {"frame_id": 1842, "cam_fps": 58.3, "has_jpeg": true, "jpeg_bytes": 34521}
```

`frame_id`가 5초 후 새로고침 시 증가하면 정상 수신 중.

### 서버 측 디버그 로그

5초마다 터미널에 수신/전송 프레임 수 출력:
```
[cam] ROS 수신 frame_id=1842  WS 전송=150회/5s
```

---

## 4. 수정된 파일 목록

| 파일 | 수정 내용 |
|------|----------|
| `web_controller/main.py` | `frame_id` 추적, `MultiThreadedExecutor`, keepalive, `/debug` 엔드포인트, `get_running_loop` |
| `web_controller/static/index.html` | blob URL `onload` 후 해제 (race condition 수정) |

---

## 5. 모바일(핸드폰) 지원 확인

`index.html:721-740`에 터치 이벤트가 이미 구현되어 있음:
```javascript
el.addEventListener('touchstart', (e) => { keys[action] = true; sendKeys(); });
el.addEventListener('touchend',   (e) => { delete keys[action]; sendKeys(); });
```
서버가 `host="0.0.0.0"`으로 바인딩되므로 같은 Wi-Fi의 핸드폰에서 접속 가능.

---

## 6. 실행 방법

### 터미널 1 — Isaac Sim
```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/rover_ws/install/setup.bash

/mnt/data/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  ~/dev_ws/rover_ws/src/a2_isaac/isaac_sim/scripts/load_terrain_webcontroller.py
```

### 터미널 2 — 웹 서버
```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/rover_ws/install/setup.bash

cd ~/dev_ws/rover_ws/src/a2_isaac/isaac_sim/web_controller
python3 main.py
```

### 터미널 3 — 카메라 토픽 확인 (선택)
```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /camera/rover/image_raw   # 정상: ~60 Hz
```

### 접속 주소
| 기기 | 주소 |
|------|------|
| PC 브라우저 | `http://localhost:8001` |
| 핸드폰 / 외부 기기 | `http://192.168.1.5:8001` |
| 디버그 확인 | `http://localhost:8001/debug` |

---

## 7. Terrain 선택 및 신규 생성

### 7-1. 현재 사용 가능한 terrain 목록

`generated_terrains/` 디렉토리에 terrain_00001 ~ terrain_00023 총 23개 존재.  
각 terrain은 `heightmap.npy`, `obstacle_grid.npy`, `meta.json`, `terrain_only.usd`, `rocks_merged.usd`, `preview.png` 포함.

### 7-2. 특정 terrain 지정 실행

```bash
# terrain_00023 지정
/mnt/data/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  ~/dev_ws/rover_ws/src/a2_isaac/isaac_sim/scripts/load_terrain_webcontroller.py \
  --terrain terrain_00023

# 옵션 생략 → 가장 최근 생성본 자동 선택
/mnt/data/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  ~/dev_ws/rover_ws/src/a2_isaac/isaac_sim/scripts/load_terrain_webcontroller.py
```

### 7-3. 새 terrain 생성 후 즉시 적용

**1단계 — terrain 생성** (Isaac Sim 없이 일반 python3로 실행 가능)

```bash
cd ~/dev_ws/rover_ws

python3 src/a2_isaac/isaac_sim/scripts/mars_terrain_generator_v2.py \
  --seed 99999 \
  --terrain-id terrain_00024
```

`--seed` 값을 바꾸면 지형 모양이 완전히 달라짐. 원하는 만큼 생성 가능.

**2단계 — Isaac Sim 실행** (생성 직후 바로 적용)

```bash
# 특정 terrain 지정
... load_terrain_webcontroller.py --terrain terrain_00024

# 또는 옵션 생략 (가장 최근 생성본 = terrain_00024 자동 선택)
... load_terrain_webcontroller.py
```

### 7-4. terrain 전환 규칙 요약

| 옵션 | 동작 |
|------|------|
| `--terrain terrain_NNNNN` | 지정한 terrain 로드 |
| 옵션 없음 | `generated_terrains/` 에서 수정 시간 최신 terrain 자동 선택 |
| `--no-terrain` | vehicle_v3.usd 만 로드 (terrain 없음) |

---

## 8. 버그 수정 — Isaac Sim 멈춤 (과도한 키 입력 시)

**증상**: W→S 등 빠른 키 전환 시 Isaac Sim이 멈춤  
**원인**: `/cmd_vel` 속도가 `+2.5 → -2.5 m/s`로 순간 반전되어 PhysX 물리엔진 불안정  
**수정**: `main.py` `_publish_cmd`에 속도 smoothing(가속 기울기 제한) 추가

```python
# 가속/감속 기울기 파라미터
self._LINEAR_ACCEL  = 4.0   # m/s²
self._ANGULAR_ACCEL = 3.0   # rad/s²

# 목표 속도로 dt당 최대 기울기만큼만 이동
max_dl = self._LINEAR_ACCEL * dt          # 0.05 s × 4.0 = 0.2 m/s per tick
self._cur_linear += max(-max_dl, min(max_dl, target_linear - self._cur_linear))
```

속도 변화 비교:

| | 수정 전 | 수정 후 |
|---|---|---|
| W→S 전환 | 즉시 +2.5→-2.5 m/s | 약 0.25초에 걸쳐 부드럽게 전환 |
| 정지 | 즉시 0 | 0.05 m/s 미만 시 즉시 스냅 |
| Isaac Sim 안정성 | PhysX 불안정 가능 | 안정적 |

---

## 9. 로버 복구 모드 (강화학습 + M0609)

### 9-1. 개요

로버가 화성 지형에서 넘어졌을 때 M0609 로봇 팔이 자동으로 일으켜 세우는 복구 시스템.  
Isaac Lab PPO 강화학습으로 복구 정책을 훈련하고, 웹 UI의 RECOVERY 버튼으로 수동 트리거 가능.

### 9-2. 시스템 구조

```
Isaac Lab 학습 (오프라인)
  recovery_env_cfg.py + recovery_mdp.py
  train_recovery.py → policies/recovery_policy.pt

                    ↓

실시간 Isaac Sim
  vehicle_v3.usd + m0609_isaac_sim.usd
  RecoveryNode (recovery_node.py)
    /imu/data → 넘어짐 감지 (|roll|>45° 또는 |pitch|>45°, 2초 지속)
    /recovery/start 서비스 수신
    recovery_policy.pt inference (22dim obs → 6dim action)
    /m0609/joint_command 발행
    /recovery/status 발행

                    ↓

웹 브라우저 / 핸드폰
  RECOVERY 버튼 → POST /recovery/start
  상태 표시: IDLE / FALLEN / RECOVERING / SUCCESS / TIMEOUT
```

### 9-3. RL 환경 설계

| 항목 | 내용 |
|------|------|
| 알고리즘 | PPO (rsl_rl) |
| 환경 수 | 256개 병렬 |
| 물리 주기 | 200 Hz (화성 중력 3.72 m/s²) |
| 정책 주기 | 50 Hz (decimation=4) |
| 에피소드 길이 | 최대 10초 |

**Observation (22차원)**

| 항목 | 차원 |
|------|:----:|
| rover roll / pitch / yaw | 3 |
| rover position z | 1 |
| rover linear velocity (x,y,z) | 3 |
| rover angular velocity (x,y,z) | 3 |
| M0609 joint position (6축) | 6 |
| M0609 joint velocity (6축) | 6 |

**Action (6차원)**: M0609 joint_1~6 위치 목표 (normalized −1~+1)

**Reward 설계**

| 항목 | 가중치 | 내용 |
|------|:------:|------|
| upright_reward | +2.0 | exp(−3×(|roll|+|pitch|)) — 기립에 가까울수록 |
| success_bonus | +100.0 | roll/pitch < 15° 달성 시 |
| joint_vel_penalty | −0.01 | 관절 속도² 합 (부드러운 동작 유도) |
| joint_limit_penalty | −1.0 | 관절 한계 초과 시 |

**초기 상태**: rover roll 60~120° (옆으로 넘어진 상태), M0609 홈 자세

### 9-4. 생성된 파일

| 파일 | 설명 |
|------|------|
| `isaac_rl/isaac_rl/recovery/recovery_env_cfg.py` | Isaac Lab 환경 설정 (Scene, Obs, Action, Reward, Termination) |
| `isaac_rl/isaac_rl/recovery/recovery_mdp.py` | Obs/Reward/Termination/Event 함수 구현 |
| `isaac_rl/isaac_rl/recovery/train_recovery.py` | PPO 학습 진입점 |
| `isaac_rl/isaac_rl/recovery/recovery_node.py` | ROS2 배포 노드 (낙하 감지 + 정책 inference) |
| `web_controller/main.py` | `/recovery/start`, `/recovery/stop`, `/recovery/status` 엔드포인트 추가 |
| `web_controller/static/index.html` | RECOVERY 버튼 + 상태 표시 추가 |

### 9-5. 실행 방법

**① RL 학습 (최초 1회, GPU 필요)**

```bash
/mnt/data/isaac_sim/IsaacLab/isaaclab.sh -p \
  ~/dev_ws/rover_ws/src/a2_isaac/isaac_rl/isaac_rl/recovery/train_recovery.py \
  --num_envs 256 --headless --max_iterations 3000

# TensorBoard로 학습 모니터링
tensorboard --logdir ~/dev_ws/rover_ws/src/a2_isaac/isaac_rl/logs/recovery
```

**② 복구 노드 실행 (Isaac Sim 실행 후)**

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/rover_ws/install/setup.bash
ros2 run isaac_rl recovery_node
```

**③ 웹 버튼 사용**

- 브라우저 오른쪽 상단 **RECOVERY** 버튼 클릭
- 상태: `IDLE` → `RECOVERING` → `SUCCESS`
- 로버가 넘어지면 2초 후 **자동 트리거**

---

## 10. 미해결 사항 / 향후 과제

| 항목 | 내용 |
|------|------|
| 카메라 프레임 수신 검증 | Isaac Sim 실행 중 `/debug`로 `frame_id` 증가 여부 최종 확인 필요 |
| 모바일 UI 개선 | 현재 WASD 버튼이 PC 기준 크기 — 핸드폰용 큰 조이스틱 버튼 추가 검토 |
| 카메라 지연 개선 | 현재 JPEG WebSocket 30FPS — H.264 스트림으로 교체 시 저지연 가능 |
| HTTPS/WSS | 외부 네트워크 노출 시 TLS 적용 필요 |
