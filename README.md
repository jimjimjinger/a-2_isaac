# Isaac Sim 기반 화성 탐사 로버 자원 채취 시스템

> Mars Mineral Collection Rover Simulation — ROS2 + Isaac Sim + PPO

본 ROS2 workspace는 절차생성된 화성 지형에서 6륜 로버가 광물을 인식·접근·수집·복귀하는 미션을 자동 시뮬레이션합니다.

## 트랙 ↔ 담당자

| 트랙 | 담당자 | 영역 | GPU |
|:---:|:----:|------|:---:|
| **T1** | **김현중** | Environment (절차생성 지형, basecamp, Mars physics) | 5060 |
| **T2** | **최진우** | Perception + M0609 (vision, manipulation) | 5080 |
| **T3** | **이찬휘** | Driving (mission FSM, A*, coverage, PPO wrapper) — **Critical Path** | 5080 |
| **T4** | **성선규** | Integration + PM (ROS2 wiring, UI, demo) | 5070 Ti |
| **T5** | **이지민** | Localization + Infra (TRN, EKF, sensor fusion) | 5080 |

각 트랙의 onboarding: [docs/tracks/T*_BRIEF.md](docs/tracks/)

## 현재 상태

✅ **구현 완료**:
- I1 지형 에셋 생성기 v2 + 샘플 terrain (`terrain_00001/`)
- `isaac_interfaces` — 4 msg / 3 srv / 3 action 정의
- `isaac_drive` — drive_manager, mobile_base_executor, navigation 전체 (FSM, BCD planner, coverage, A*, fog map)
- `isaac_supervisor` — mission_manager_node (683줄), battery_monitor_node
- `isaac_rl` — driving_policy_node, PPO wrapper, recovery_node, recovery_env_cfg v4
- **웹 컨트롤러** — WebRTC 영상 + WebSocket 조종 + HUD (아래 섹션 참조)

⏳ **stub / 진행 중**:
- `isaac_localization` — localization_node, EKF, TRN (이지민 T5)
- `isaac_perception` — perception_node 골격만 (최진우 T2)
- `isaac_manipulation` — arm_executor_node 골격만 (최진우 T2)

## 웹 컨트롤러

Isaac Sim 로버를 브라우저에서 실시간으로 조종하고 모니터링하는 웹 기반 조종 인터페이스.

### 시스템 구조

```
┌──────────────────────────────────────────┐
│              ISAAC SIM                   │
│  vehicle_v3.usd ──▶ ROS2 Bridge          │
│                                          │
│  /camera/rover/image_raw  (~60 Hz)       │
│  /imu/data                (~102 Hz)      │
│  /cmd_vel                 (구독, 20 Hz)  │
└────────────┬─────────────────▲───────────┘
             │ ROS2 DDS        │
             ▼                 │
┌──────────────────────────────────────────┐
│     RoverBridgeNode  (main.py)           │
│     FastAPI + uvicorn  :8001             │
│                                          │
│  ┌─────────────┐ ┌──────────┐ ┌───────┐ │
│  │CameraVideo  │ │/ws/status│ │/ws/   │ │
│  │Track(WebRTC)│ │10Hz JSON │ │control│ │
│  └──────┬──────┘ └────┬─────┘ └───┬───┘ │
│         │             │           │     │
│       POST /rtc/offer │    REST API     │
│       SDP 시그널링    │  /recovery/*   │
└─────────┬─────────────┬───────────┬─────┘
          │ WebRTC(VP8) │ WebSocket │ WebSocket
          ▼             ▼           ▼
┌──────────────────────────────────────────┐
│          브라우저  index.html            │
│                                          │
│  카메라 피드 + HUD 오버레이              │
│  IMU 패널 · 속도계 SVG · 조향 SVG       │
│  WASD / 방향키 / 터치 조종              │
│  SPACE: 비상 정지(E-STOP)               │
│  RECOVERY 버튼 (복구 시작/중지)         │
└──────────────────────────────────────────┘
```

### 통신 채널

| 채널 | 프로토콜 | 방향 | 내용 | 주기 |
|---|---|---|---|---|
| 카메라 영상 | WebRTC (VP8) | 서버 → 브라우저 | JPEG→YUV 스트림 | 30 fps |
| 상태 스트림 | WebSocket `/ws/status` | 서버 → 브라우저 | IMU, 속도, FPS (JSON) | 10 Hz |
| 조종 입력 | WebSocket `/ws/control` | 브라우저 → 서버 | WASD 키 상태 (JSON) | 이벤트 기반 |
| 복구 명령 | REST POST | 브라우저 → 서버 | start / stop | 버튼 클릭 |
| 복구 상태 | REST GET | 서버 → 브라우저 | IDLE/RECOVERING/SUCCESS | 2초 폴링 |

### 실행

```bash
# 터미널 1 — Isaac Sim 시뮬레이션 시작
/mnt/data/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  src/a2_isaac/isaac_sim/scripts/load_terrain_webcontroller.py

# 터미널 2 — 웹 서버 시작
source /opt/ros/humble/setup.bash
source ~/dev_ws/rover_ws/install/setup.bash
bash src/a2_isaac/isaac_sim/web_controller/launch_web_controller.sh

# 브라우저에서 열기
# http://localhost:8001
```

### 조종 키

| 키 | 동작 |
|---|---|
| W / ↑ | 전진 (최대 2.5 m/s) |
| S / ↓ | 후진 |
| A / ← | 좌회전 (최대 1.5 rad/s) |
| D / → | 우회전 |
| Space | 비상 정지 토글 (E-STOP) |
| RECOVERY 버튼 | 로버 자세 복구 시작/중지 |

## 빠른 시작

### 1. 워크스페이스 생성 + clone

```bash
mkdir -p ~/dev_ws/rover_ws/src
cd ~/dev_ws/rover_ws/src
git clone https://github.com/sungyu-sung/a-2_isaac.git a2_isaac
cd a2_isaac
```

> ℹ️ 위치는 `~/dev_ws/rover_ws/` 권장. 다른 위치도 기능적으로 무관 — [docs/SETUP_BASHRC.md](docs/SETUP_BASHRC.md) 참조.

### 2. 의존성

```bash
# 공통
pip install --user noise usd-core jsonschema scipy

# 웹 컨트롤러 (venv 권장)
cd src/a2_isaac/isaac_sim/web_controller
python3 -m venv venv
source venv/bin/activate
pip install fastapi uvicorn aiortc opencv-python-headless numpy

# T3 이찬휘 — A* 구현
pip install --user pyastar2d
```

### 3. ROS2 빌드 + source

```bash
cd ~/dev_ws/rover_ws
colcon build --symlink-install
source install/setup.bash

# 패키지 확인
ros2 pkg list | grep isaac_
```

### 4. bashrc 설정

→ **[docs/SETUP_BASHRC.md](docs/SETUP_BASHRC.md)** — 권장 alias + 환경 변수 + ROS_DOMAIN_ID

### 5. ROS2 실행

```bash
# 전체 통합
ros2 launch isaac_bringup full_system.launch.py

# 트랙별 부분 실행
ros2 launch isaac_bringup sim.launch.py            # T1 환경
ros2 launch isaac_bringup perception.launch.py     # T2 vision
ros2 launch isaac_bringup drive.launch.py          # T3 주행
ros2 launch isaac_bringup localization.launch.py   # T5 TRN
ros2 launch isaac_bringup manipulation.launch.py   # T2 M0609
ros2 launch isaac_bringup supervisor.launch.py     # T4 mission
ros2 launch isaac_bringup rl.launch.py             # T3 RL inference
```

## Workspace 구조

```text
a2_isaac/
├─ README.md
├─ docs/
│  ├─ STUDY_AND_PLAN.md
│  ├─ SETUP_BASHRC.md
│  ├─ flowcharts/
│  ├─ interfaces/
│  │  ├─ INTERFACE_CONTRACTS.md
│  │  ├─ I1_TERRAIN_ASSETS.md
│  │  ├─ META_JSON_FIELDS.md
│  │  ├─ terrain_meta_schema.json
│  │  └─ msg/
│  ├─ tracks/
│  │  ├─ T{1-5}_BRIEF.md / T{1-5}_CLAUDE.md
│  └─ pm_tools/
│
├─ isaac_bringup/                             # ① 진입점 (T4 성선규)
│  └─ launch/                                 # 8개 launch
│
├─ isaac_sim/                                 # ② Isaac Sim 환경 (T1 김현중)
│  ├─ isaac_sim/sim_bridge_node.py
│  ├─ worlds/mars_exploration_world.usd       ✅
│  ├─ assets/
│  │  ├─ generated_terrains/terrain_00001/    ✅ I1 1샘플
│  │  └─ markers/                             ✅ 광물 USD
│  ├─ scripts/
│  │  ├─ mars_terrain_generator_v2.py         ✅
│  │  └─ load_terrain_webcontroller.py        ✅ Isaac Sim 로더
│  └─ web_controller/                         ✅ 웹 조종 인터페이스
│     ├─ main.py                              # FastAPI + WebRTC + WebSocket
│     ├─ static/index.html                    # HUD UI
│     └─ launch_web_controller.sh
│
├─ isaac_perception/                          # ③ 인지 (T2 최진우)
│  └─ isaac_perception/
│     ├─ perception_node.py                   ⏳ 골격
│     └─ vision/{mineral_detector, obstacle_detector, terrain_analyzer}.py  ⏳
│
├─ isaac_drive/                               # ④ 주행 (T3 이찬휘) ⭐ Critical Path
│  └─ isaac_drive/
│     ├─ drive_manager_node.py                ✅ 234줄
│     ├─ mobile_base_executor_node.py         ✅ 98줄
│     ├─ navigation/
│     │  ├─ mission_fsm.py                    ✅ 174줄
│     │  ├─ bcd_planner.py                    ✅ 229줄
│     │  ├─ coverage_planner.py               ✅ 109줄
│     │  ├─ path_planner.py                   ✅ 106줄
│     │  ├─ fog_map.py                        ✅ 208줄
│     │  └─ obstacle_grid.py                  ✅ 73줄
│     └─ primitives/{drive_to_target, avoid_obstacle, stop_rover}.py  ✅
│
├─ isaac_rl/                                  # ⑤ PPO 정책 (T3 이찬휘)
│  └─ isaac_rl/
│     ├─ driving_policy_node.py               ✅ 84줄
│     ├─ ppo_wrapper.py                       ✅
│     └─ recovery/
│        ├─ recovery_node.py                  ✅ 293줄
│        ├─ recovery_env_cfg_v4.py            ✅ 382줄
│        └─ train_recovery_v4.py              ✅
│
├─ isaac_interfaces/                          # ⑥ 통신 규격 (T4 성선규)
│  ├─ msg/                                    ✅ 4개
│  ├─ srv/                                    ✅ 3개
│  └─ action/                                 ✅ 3개
│
├─ isaac_supervisor/                          # ⑦ 미션 감독 (T4 성선규)
│  └─ isaac_supervisor/
│     ├─ mission_manager_node.py              ✅ 683줄
│     └─ battery_monitor_node.py              ✅
│
├─ isaac_manipulation/                        # ⑧ M0609 (T2 최진우)
│  └─ isaac_manipulation/
│     ├─ arm_executor_node.py                 ⏳ 96줄 (골격)
│     └─ primitives/{pick_mineral, place_to_cargo, unload_to_base, deploy_solar_panel}.py  ⏳
│
└─ isaac_localization/                        # ⑨ GPS-less 위치 (T5 이지민)
   └─ isaac_localization/
      ├─ localization_node.py                 ⏳ stub
      ├─ ekf_fusion.py                        ⏳ stub
      ├─ trn.py                               ⏳ stub
      └─ sensors/{wheel_odom, imu_integrator, sun_yaw}.py  ⏳
```

> ✅ = 구현 완료, ⏳ = stub / 진행 중

## 패키지 상태

| 패키지 | 트랙 | 담당자 | 주요 노드 | 상태 |
|--------|:----:|:-----:|----------|:----:|
| `isaac_bringup` | T4 | 성선규 | 8 launch | ✅ |
| `isaac_sim` | T1 | 김현중 | sim_bridge, generator, **web_controller** | ✅ |
| `isaac_perception` | T2 | 최진우 | perception_node (vision/depth/lidar) | ⏳ |
| `isaac_rl` | T3 | 이찬휘 | driving_policy_node, recovery_node | ✅ |
| `isaac_drive` | T3 | 이찬휘 | drive_manager, mobile_base, navigation | ✅ |
| `isaac_supervisor` | T4 | 성선규 | mission_manager, battery_monitor | ✅ |
| `isaac_manipulation` | T2 | 최진우 | arm_executor + 4 primitives | ⏳ |
| `isaac_localization` | T5 | 이지민 | localization, TRN, EKF | ⏳ |
| `isaac_interfaces` | T4 | 성선규 | msg/srv/action 정의 | ✅ |

## 핵심 문서 인덱스

| 누가 보나 | 무엇 | 위치 |
|----------|------|------|
| 신규 합류자 (5명) | 본인 트랙 onboarding | `docs/tracks/T{1-5}_BRIEF.md` |
| 모든 개발자 (Day 1) | bashrc 셋업 | [docs/SETUP_BASHRC.md](docs/SETUP_BASHRC.md) |
| Claude Code 사용 시 | 자동 로드 context | `docs/tracks/T{1-5}_CLAUDE.md` |
| 모든 개발자 | 5개 인터페이스 계약 | [docs/interfaces/INTERFACE_CONTRACTS.md](docs/interfaces/INTERFACE_CONTRACTS.md) |
| 김현중, 이지민 | I1 풀 가이드 | [docs/interfaces/I1_TERRAIN_ASSETS.md](docs/interfaces/I1_TERRAIN_ASSETS.md) |
| 성선규 (PM) | 매일/주간 운영 | `docs/pm_tools/` |
| 모든 개발자 | 전체 설계 의도 | [docs/STUDY_AND_PLAN.md](docs/STUDY_AND_PLAN.md) |

## Architecture

- 전체 시스템 아키텍처: [docs/flowcharts/system_architecture_full.svg](docs/flowcharts/system_architecture_full.svg)
- 미션 시나리오: [docs/flowcharts/project_overview_flowchart.svg](docs/flowcharts/project_overview_flowchart.svg)
- 개발 일정 timeline: [docs/flowcharts/development_timeline.svg](docs/flowcharts/development_timeline.svg)
