# Isaac Sim 기반 화성 탐사 로버 자원 채취 시스템

> Mars Mineral Collection Rover Simulation — ROS2 + Isaac Sim + PPO

본 ROS2 workspace는 절차생성된 화성 지형에서 6륜 로버가 광물을 인식·접근·수집·복귀하는 미션을 자동 시뮬레이션합니다.

---

## 🚀 팀원이 main pull 후 해야 할 것 (Quick Start)

### 1. 환경 셋업 (1회만)

**Isaac Sim 5.1 PyPI binary** — [docs/SETUP_ISAAC_PYPI.md](docs/SETUP_ISAAC_PYPI.md) 참조:
- 권장 venv 경로: `~/dev_ws/isaac_sim_pypi/venv`
- ROS2 humble 워크스페이스: `~/dev_ws/isaac_sim/IsaacSim-ros_workspaces/humble_ws`
- 다른 경로 사용 시 env var override: `ISAAC_PYPI_VENV`, `ISAAC_ROS2_WS`

**.bashrc** — [docs/SETUP_BASHRC.md](docs/SETUP_BASHRC.md) 참조 (ROS humble + 워크스페이스 source + ROS_DOMAIN_ID).

**ROS_DOMAIN_ID** (팀 합의 — 같은 LAN 충돌 방지):

| 트랙 | 담당자 | ID |
|:---:|:---:|:---:|
| T1 | 김현중 | **111** |
| T2 | 최진우 | **114** |
| T3 | 이찬휘 | **112** |
| T4 | 성선규 | **113** |
| T5 | 이지민 | **115** |

```bash
export ROS_DOMAIN_ID=<본인 값>   # ~/.bashrc 에 추가
```

### 2. 빌드 (main pull 후 또는 코드 변경 시)

```bash
cd ~/dev_ws/rover_ws
colcon build --symlink-install
source install/setup.bash
```

> ✅ **별도 build 불필요한 자산** (모두 main 에 baked):
> - `vehicle_v3.usd` (액션그래프 내장 로버) — `build_vehicle_v3.py` 재실행 불필요
> - `terrain_00001~00022` (heightmap + obstacle + meta) — `mars_terrain_generator_v2.py` 재실행 불필요
> - YOLO 모델 `mineral_yolo_best.pt` (5.3MB) — git tracked
> - Doosan M0609 + RG2 gripper USDs
> 빌드 스크립트는 USD/terrain 자체를 수정·재생성할 때만 실행.

### 3. 시연 실행 (3 터미널)

```bash
# T1 — Isaac Sim (source 없이! tools/isaac-pypi wrapper 가 자체 처리)
cd ~/dev_ws/rover_ws/src/a2_isaac
tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00004

# T2 — ROS2 노드 묶음 (perception + coverage + supervisor + arm + GT cheat)
source /opt/ros/humble/setup.bash && source ~/dev_ws/rover_ws/install/setup.bash
ros2 launch isaac_bringup mvp.launch.py

# T3 — 카메라 view 2개 (body + wrist)
source /opt/ros/humble/setup.bash && source ~/dev_ws/rover_ws/install/setup.bash
ros2 launch isaac_bringup rqt_views.launch.py
```

→ rover 가 자율적으로 EXPLORE → APPROACH → PICK 반복 후, `collection_goal` (기본 5개) 도달
또는 배터리 critical 시 **RETURN_TO_BASE** → 베이스캠프 (0,0) 도착 → **MISSION_COMPLETE**.

### 4. Mission Control UI (Web HUD)

`mvp.launch.py` 가 띄우는 **mission_web_node** (Flask+SocketIO) + **web_video_server** 가
SC2 풍 HUD 를 제공합니다.

```bash
# 1회만: 의존성 설치
pip3 install --user flask-socketio eventlet
sudo apt install -y ros-humble-web-video-server

# 브라우저 접속
xdg-open http://localhost:8088     # 또는 LAN 내 다른 PC: http://<host-ip>:8088
```

- **좌하단**: coverage minimap (canvas, rover 위치/베이스캠프 표시)
- **중앙하단**: portrait + phase 배지 + battery/collected 바 + POS/YAW/SPEED/CMD/TASK/ERR
- **우상단**: 광물 종류별 카운터 + collected/goal
- **우하단**: 3×3 액션 그리드 (디자인 단계 — placeholder, 버튼 비활성)
- **중앙 메인**: Isaac Sim overview (현재 sun cam placeholder) / Body YOLO / Wrist YOLO

원격조종 (teleop_twist_keyboard) 과 AUTO/MANUAL/ESTOP 버튼 연동은 다음 단계.

### 5. T5 정공법 졸업 (선택, 정확도 검증 완료 시점에)

`mvp.launch.py` 의 `odom_to_estimated_pose` 노드를 빼고 별도 터미널에서:
```bash
ros2 launch isaac_bringup localization.launch.py
```
+ `arm_executor` / `mission_manager_node` 의 `odom_topic` 파라미터를 `/rover/estimated_odom` 으로 swap.

---

> ℹ️ 명칭 명료성·책임 분리를 위한 **9개 패키지 구조**가 `main`에 정착 완료 (6개 → 9개 재편성). 설계 의도: [docs/STUDY_AND_PLAN.md](docs/STUDY_AND_PLAN.md) Part XI.

## 트랙 ↔ 담당자

| 트랙 | 담당자 | 영역 | GPU |
|:---:|:----:|------|:---:|
| **T1** | **김현중** | Environment (절차생성 지형, basecamp, Mars physics) | 5060 |
| **T2** | **최진우** | Perception + M0609 (vision, manipulation) | 5080 |
| **T3** | **이찬휘** | Driving (mission FSM, A*, coverage, PPO wrapper) — **Critical Path** | 5080 |
| **T4** | **성선규** | Integration + PM (ROS2 wiring, UI, demo) — 사용자 본인 | 5070 Ti |
| **T5** | **이지민** | Localization + Infra (TRN, EKF, sensor fusion) | 5080 |

각 트랙의 onboarding: [docs/tracks/T*_BRIEF.md](docs/tracks/)

## 현재 상태 (2026-05-26 발표 직전)

🎉 **MVP 시연 모델 완성** — 화성 rover 자율 mineral 수집 mission 의 end-to-end loop:
`EXPLORE → 발견 → APPROACH → PICK (FixedJoint snap) → CARGO swing → RELEASE → EXPLORE` 반복 동작 검증 완료.

✅ **구현 완료 (시연 PASS)**:
- **I1 지형 자산** — v2 생성기로 `terrain_00001`~`terrain_00022` 생성 (장애물·22 mineral·basecamp baked)
- **vehicle_v3.usd** — 액션그래프 내장 자립 로버 (T1+T2+T3+T5 통합): ROS2 sensor 발행 + `/cmd_vel→Ackermann→휠` 구동 + `/arm/joint_command→m0609+finger` 제어 + `/grasp/command` FixedJoint snap + GT odom (dev cheat)
- **isaac_drive** — `coverage_node` (BCD anchor sweep + A* obstacle 회피 + replan), `odom_to_estimated_pose` (GT cheat 어댑터), `minimap_publisher`
- **isaac_perception** — `yolo_perception_node` (nav + wrist YOLO + depth backproject → mineral world XYZ)
- **isaac_supervisor** — `mission_manager_node` (EXPLORE/APPROACH/PICK phase + cmd_vel mux + arm action client + lock-on)
- **isaac_manipulation** — `arm_executor_node` (T2 DLS-IK 상태머신: HOME→APPROACH→DESCEND→pickup→HOME_MID→CARGO→release→HOME)
- **isaac_localization (T5 이지민)** — `ekf_fusion` (wheel+imu+sun+TRN EKF), 8개 sensor/fusion 노드 구현. **정확도 추가 튠 필요** (졸업 작업)
- **isaac_bringup** — `mvp.launch.py` (시연용 GT cheat 모드), `rqt_views.launch.py` (카메라 view 2개), `localization.launch.py` (T5 stack + EDL pose prior)
- **isaac_interfaces** — msg + action 계약 정착

⚠️ **시연용 cheat (졸업 시 청산)**:
- `/ground_truth/odom` — `odom_to_estimated_pose` 가 `/rover/estimated_pose` 로 forwarding. T5 EKF stack 정확도 검증 완료되면 `localization.launch.py` 가 대체.
- `ik_descend_dz=-0.40` — perception z bias (+47cm 평균) 보정. T2 depth 정확도 올리면 default 복귀.
- FixedJoint snap grip — 30cm 광물 vs 4cm finger gap 비대칭 (hardware 한계). 시연용 hack 박제.

⏳ **추가 작업 (시연 후)**:
- T5 EKF: sun_yaw + TRN 정확도 튠
- T2 perception: depth z bias 해결
- T3 RL inference: PPO 정책 통합 (`driving_policy_node`)
- 정리 대기 목록: [list_to_fix.md](list_to_fix.md)

## 빠른 시작 (Day 1 셋업, ~5분)

### 1. 워크스페이스 생성 + clone

```bash
# ROS2 워크스페이스 (~/dev_ws/rover_ws/) 만들고 src/ 안에 우리 repo clone
mkdir -p ~/dev_ws/rover_ws/src
cd ~/dev_ws/rover_ws/src

# 폴더명을 a2_isaac으로 명시 (repo는 a-2_isaac이지만 우리 환경은 a2_isaac로 통일)
git clone https://github.com/sungyu-sung/a-2_isaac.git a2_isaac

cd a2_isaac
```

> ℹ️ 위치는 `~/dev_ws/rover_ws/` 권장 (문서 명령어 그대로 copy-paste 가능). 다른 위치도 기능적으로 무관 — [docs/SETUP_BASHRC.md](docs/SETUP_BASHRC.md) §위치 무관성 참조.

### 2. 의존성 (한 번만)

```bash
# Python 의존성 (지형 생성기 + meta.json schema 검증 + PNG 오버뷰)
pip install --user usd-core jsonschema numpy matplotlib

# 트랙 owner별 추가 (필요한 사람만)
# T3 이찬휘 — A* 빠른 구현
pip install --user pyastar2d
```

### 3. ROS2 빌드 + source

```bash
cd ~/dev_ws/rover_ws

# 첫 빌드 (~30초)
colcon build --symlink-install

# source (매 터미널 또는 .bashrc에 등록)
source install/setup.bash

# 검증 — 9개 패키지 등록 확인
ros2 pkg list | grep isaac_
# 예상 출력: isaac_bringup, isaac_drive, isaac_interfaces, isaac_localization,
#           isaac_manipulation, isaac_perception, isaac_rl, isaac_sim, isaac_supervisor
```

### 4. (선택) bashrc 설정

매 터미널마다 source / cd / build 명령 안 치려면 **`~/.bashrc`에 한 번 등록**해두면 편함. 권장 alias + 환경 변수 + ROS_DOMAIN_ID 충돌 방지:

→ **[docs/SETUP_BASHRC.md](docs/SETUP_BASHRC.md)** — 필수/권장/선택 단계별 + 트랙별 추가 alias

### 5. Isaac Sim 시각 확인

```bash
# I1 지형은 이미 생성되어 있음 (terrain_00001~00003)
# Isaac Sim에서 master scene 열기:
isaac ~/dev_ws/rover_ws/src/a2_isaac/isaac_sim/worlds/mars_exploration_world.usd

# 또는 다른 seed로 새 terrain 생성:
cd ~/dev_ws/rover_ws/src/a2_isaac
python3 isaac_sim/scripts/mars_terrain_generator_v2.py --seed 777 --terrain-id terrain_00004
```

→ I1 풀 가이드: [docs/interfaces/I1_TERRAIN_ASSETS.md](docs/interfaces/I1_TERRAIN_ASSETS.md)

### 6. 시연 실행 (3 터미널, MVP 통합)

> 📌 **사전 셋업 필수** — Isaac Sim 5.1 PyPI binary 환경: [docs/SETUP_ISAAC_PYPI.md](docs/SETUP_ISAAC_PYPI.md)
> .bashrc / ROS_DOMAIN_ID: [docs/SETUP_BASHRC.md](docs/SETUP_BASHRC.md)

```bash
# ───── T1: Isaac Sim vehicle_v3 (source 없이, tools/isaac-pypi wrapper) ─────
cd ~/dev_ws/rover_ws/src/a2_isaac
tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00004

# ───── T2: ROS2 노드 묶음 (perception + coverage + supervisor + arm + GT cheat) ─────
# 새 터미널, source 필요
source /opt/ros/humble/setup.bash && source ~/dev_ws/rover_ws/install/setup.bash
ros2 launch isaac_bringup mvp.launch.py

# ───── T3: 카메라 view 2개 (body + wrist) ─────
# 새 터미널, source 필요
source /opt/ros/humble/setup.bash && source ~/dev_ws/rover_ws/install/setup.bash
ros2 launch isaac_bringup rqt_views.launch.py
```

→ rover 가 자율적으로 EXPLORE → mineral 발견 → APPROACH → PICK → CARGO RELEASE 반복.

### 트랙별 부분 launch (개발용)

```bash
ros2 launch isaac_bringup sim.launch.py            # 환경 검증 (T1)
ros2 launch isaac_bringup perception.launch.py     # vision (T2)
ros2 launch isaac_bringup drive.launch.py          # 주행 (T3)
ros2 launch isaac_bringup localization.launch.py   # T5 EKF stack (정공법, 졸업 대기)
ros2 launch isaac_bringup manipulation.launch.py   # M0609 (T2)
ros2 launch isaac_bringup supervisor.launch.py     # mission (T4)
ros2 launch isaac_bringup full_system.launch.py    # 전체 (모든 트랙 통합)
```

### 시연용 → 정공법 졸업 경로

| Cheat | mvp.launch.py | 졸업 launch |
|---|---|---|
| GT odom (`odom_to_estimated_pose`) | ✓ default | `localization.launch.py` 실행 (T5 EKF) |
| `ik_descend_dz=-0.40` (perception z bias 보정) | ✓ default | T2 depth 정확도 향상 시 launch param 제거 |
| FixedJoint snap grip | ✓ default | (hardware 한계 — 영구) |

## Workspace 구조 (2026-05-26 시연 시점)

⭐ = 시연용 (mvp.launch.py / T1~T3 명령) 핵심 파일.

```text
a2_isaac/
├─ README.md                              # ⭐ 이 문서 (Quick Start 포함)
├─ list_to_fix.md                         # 시연 후 작업 목록 + 졸업 경로
├─ .gitignore                             # build/cache/temp + .pt 예외 등록
│
├─ docs/                                  # 설계 문서 + 가이드 + 트랙 onboarding
│  ├─ STUDY_AND_PLAN.md                   # 전체 설계 의도
│  ├─ SETUP_BASHRC.md                     # ⭐ ~/.bashrc + ROS_DOMAIN_ID 가이드
│  ├─ SETUP_ISAAC_PYPI.md                 # ⭐ Isaac Sim PyPI venv 셋업
│  ├─ TREE_CLEANUP_PLAN.md                # 정리 작업 기록 (참조용 archive)
│  ├─ interfaces/                         # I1~I5 인터페이스 계약 (msg/action 규격)
│  ├─ tracks/                             # 트랙별 onboarding (T1~T5_BRIEF.md)
│  ├─ pm_tools/                           # PM 운영 (KICKOFF/DAILY/DECISIONS)
│  ├─ system_design/                      # 아키텍처 평가 (ARCHITECTURE_EVAL_*)
│  └─ troubleshooting/                    # 진단 기록 (camera regression 등)
│
├─ tools/
│  ├─ isaac-pypi                          # ⭐ Isaac Sim PyPI 환경 wrapper (T1 entry)
│  └─ README.md                           # tools/ 가이드
│
├─ isaac_bringup/                         # ① 진입점 — launch 파일 모음 (T4)
│  └─ launch/
│     ├─ mvp.launch.py                    # ⭐ 시연용 5노드 통합 (T2 entry, GT cheat 모드)
│     ├─ rqt_views.launch.py              # ⭐ body+wrist 카메라 view 2개 (T3 entry)
│     ├─ localization.launch.py           # T5 EKF stack + EDL prior 자동 (졸업용)
│     ├─ full_system.launch.py            # mvp + use_localization/use_rqt_views 옵션
│     ├─ supervisor.launch.py             # mission_manager + battery_monitor (단독 검증용)
│     ├─ perception.launch.py             # yolo_perception_node 단독
│     ├─ drive.launch.py                  # coverage_node + cmd_vel remap 단독
│     ├─ manipulation.launch.py           # arm_executor_node + ik_descend_dz=-0.40 단독
│     └─ sim.launch.py                    # sim_bridge_node (mock lifecycle service, 단위 테스트용)
│
├─ isaac_sim/                             # ② Isaac Sim 환경 + 자산 (T1)
│  ├─ assets/
│  │  ├─ vehicle/
│  │  │  ├─ vehicle_v3.usd                # ⭐ 액션그래프 내장 자립 로버 (시연 사용)
│  │  │  ├─ vehicle_v2.usd                # v3 빌드 입력 (외형·물리·관절)
│  │  │  ├─ vehicle_v2_scene.usd          # v2 시각 검증용
│  │  │  ├─ vehicle_v1.usd                # T3 coverage 검증 끝난 구버전
│  │  │  └─ vehicle_origin_T2.usd         # T2 원본 (vehicle_v1 base, build_integrated_vehicle 의존)
│  │  ├─ markers/tier2_mineral/           # ⭐ 광물 mesh + texture (T2 YOLO 학습 대상)
│  │  │  ├─ blue_mineral.usd              # 30cm 청색 광물 (RigidBody + convexHull)
│  │  │  ├─ yellow_mineral.usd            # 30cm 황색 광물
│  │  │  └─ green_gas.usd                 # 20cm 가스 cube
│  │  ├─ markers/                         # mineral 변형 + basecamp/command_center USD
│  │  ├─ generated_terrains/
│  │  │  └─ terrain_00001~00022/          # 22개 terrain (heightmap + obstacle + USD + meta)
│  │  ├─ doosan-robot2/                   # Doosan M0609 arm 자산
│  │  ├─ onrobot_rg2/                     # RG2-FT 그리퍼 자산
│  │  └─ rover/                           # AAU Mars rover 자산
│  ├─ worlds/
│  │  └─ mars_exploration_world.usd       # master scene (최신 alias)
│  └─ scripts/
│     ├─ run_vehicle_v3.py                # ⭐ vehicle_v3 + terrain 런처 (T1 entry script)
│     ├─ build_vehicle_v3.py              # vehicle_v3.usd 빌더 (USD 수정 시만)
│     ├─ mars_terrain_generator_v2.py     # terrain 생성기 (새 terrain 필요 시만)
│     └─ test_grip_unit.py                # grip 격리 단위 테스트
│
├─ isaac_drive/                           # ③ 주행 (T3 — Critical Path)
│  └─ isaac_drive/
│     ├─ coverage_node.py                 # ⭐ BCD anchor sweep + A* obstacle 회피 + replan
│     ├─ odom_to_estimated_pose.py        # GT cheat 어댑터 (T5 졸업 시 폐기)
│     ├─ minimap_publisher.py             # /mission/minimap 시각화
│     └─ navigation/                      # A*, BCD, fog_map, terrain_loader, navigator 모듈
│
├─ isaac_perception/                      # ④ 인지 (T2)
│  ├─ isaac_perception/
│  │  ├─ yolo_perception_node.py          # ⭐ YOLO + depth backproject → mineral world XYZ
│  │  ├─ vision/                          # mineral_detector, value_estimator 등
│  │  └─ depth/                           # depth_estimator
│  └─ models/
│     └─ mineral_yolo_best.pt             # ⭐ 학습된 YOLO 모델 (5.3MB, 시연 필수)
│
├─ isaac_supervisor/                      # ⑤ mission FSM (T4)
│  └─ isaac_supervisor/
│     ├─ mission_manager_node.py          # ⭐ EXPLORE/APPROACH/PICK phase + cmd_vel mux
│     └─ battery_monitor_node.py          # 배터리 모니터 (시연 미사용)
│
├─ isaac_manipulation/                    # ⑥ M0609 arm (T2)
│  ├─ isaac_manipulation/
│  │  ├─ arm_executor_node.py             # ⭐ DLS-IK 상태머신 + /grasp/command FixedJoint
│  │  └─ kinematics.py                    # M0609 DH 파라미터 + DLS-IK
│  └─ scripts/                            # T2 standalone 데모 (참고 archive)
│
├─ isaac_localization/                    # ⑦ localization (T5, 정확도 졸업 대기)
│  └─ isaac_localization/
│     ├─ ekf_fusion.py                    # ⭐ EKF (wheel+imu+sun+TRN) + EDL initial prior
│     ├─ trn.py                           # Terrain Relative Navigation
│     ├─ localization_node.py             # 통합 wrapper
│     ├─ terrain_map_publisher.py         # terrain heightmap publish
│     └─ sensors/                         # wheel_odom + imu_integrator + sun_yaw + splitter
│
├─ isaac_interfaces/                      # ⑧ msg/srv/action 계약 (T4)
│  ├─ msg/                                # Detection, DetectionArray 등
│  ├─ srv/
│  └─ action/                             # ExecuteArmTask 등
│
├─ isaac_rl/                              # ⑨ 강화학습 (T3, 시연 미사용 stub)
│  ├─ isaac_rl/driving_policy_node.py     # PPO inference 골격 (stub)
│  └─ policies/driving_policy.pt          # 학습 정책 placeholder
│
└─ temp/                                  # (gitignored) 개인 임시 작업 (팀원별 자유)
```

> ✅ = 동작, ⏳ = stub (트랙 owner가 채울 자리), 📦 = binary asset

## Patch from previous structure

| 이전 | 변경 후 | 사유 |
|------|--------|------|
| `isaac_ai` | `isaac_perception` + `isaac_rl` | vision과 RL은 다른 책임, 다른 owner |
| `isaac_navigation` | `isaac_drive` | 실제 콘텐츠 = "주행 실행" (navigation은 내부 서브) |
| `isaac_nodes` | `isaac_supervisor` + `isaac_manipulation` | "nodes" 모호, 책임별 분리 |
| (없음) | `isaac_localization` 신규 | GPS-less Mars TRN 영역 |

## 패키지 매핑

| 패키지 | 트랙 | 담당자 | 주요 노드 | 상태 |
|--------|:----:|:-----:|----------|:----:|
| `isaac_bringup` | T4 | 성선규 | 8 launch | ⏳ launch 골격 |
| `isaac_sim` | T1 | 김현중 | `sim_bridge_node`, 지형 생성기 v2 | ✅ I1 3샘플 |
| `isaac_perception` | T2 | 최진우 | `perception_node` (vision/depth/lidar) | ✅ 노드 / ⏳ 세부 |
| `isaac_rl` | T3 | 이찬휘 | `driving_policy_node`, PPO wrapper | ✅ 노드 / ⏳ 내부 |
| `isaac_drive` | T3 | 이찬휘 | `drive_manager_node` + navigation 일습 | ✅ 노드+navigation |
| `isaac_supervisor` | T4 | 성선규 | `mission_manager_node`, `battery_monitor_node` | ✅ |
| `isaac_manipulation` | T2 | 최진우 | `arm_executor_node` + 4 primitives | ✅ 노드 / ⏳ primitives |
| `isaac_localization` | T5 | 이지민 | `localization_node`, TRN, EKF | ⏳ 미착수 |
| `isaac_interfaces` | T4 | 성선규 | msg/srv/action 정의 | ✅ 정의됨 |

## 핵심 문서 인덱스

| 누가 보나 | 무엇 | 위치 |
|----------|------|------|
| 신규 합류자 (5명) | 본인 트랙 onboarding | `docs/tracks/T{1-5}_BRIEF.md` |
| 모든 개발자 (Day 1) | **bashrc 셋업 가이드** | [docs/SETUP_BASHRC.md](docs/SETUP_BASHRC.md) |
| Claude Code 사용 시 | 자동 로드 context | `docs/tracks/T{1-5}_CLAUDE.md` |
| 모든 개발자 | 5개 인터페이스 계약 | [docs/interfaces/INTERFACE_CONTRACTS.md](docs/interfaces/INTERFACE_CONTRACTS.md) |
| 김현중, 이지민 (좌표계 합의) | I1 풀 가이드 | [docs/interfaces/I1_TERRAIN_ASSETS.md](docs/interfaces/I1_TERRAIN_ASSETS.md) |
| meta.json 필드 참고 | 라인별 주석 | [docs/interfaces/META_JSON_FIELDS.md](docs/interfaces/META_JSON_FIELDS.md) |
| 성선규 (PM) | 매일/주간 운영 | `docs/pm_tools/` |
| 모든 개발자 | 전체 설계 의도 | [docs/STUDY_AND_PLAN.md](docs/STUDY_AND_PLAN.md) |

## Architecture

- 전체 시스템 아키텍처: [system_architecture_full.svg](docs/flowcharts/system_architecture_full.svg)
- 미션 시나리오: [project_overview_flowchart.svg](docs/flowcharts/project_overview_flowchart.svg) · [PNG](docs/flowcharts/project_overview_flowchart_landscape.png)
- 개발 일정 timeline: [development_timeline.svg](docs/flowcharts/development_timeline.svg) · [PNG](docs/flowcharts/development_timeline.png)
