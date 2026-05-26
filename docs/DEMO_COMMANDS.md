# 시연 명령어 모음 (2026-05-26 통합 후)

> 모든 명령은 **새로 연 터미널 (zero state)** 기준. 각 터미널 시작 시 source / cd / export 까지 self-contained.
>
> 사용자(T4) ROS_DOMAIN_ID=113 가정. 다른 트랙은 본인 값으로 교체.

---

## 0. 사전 준비 (1회만)

### 빌드
```bash
source /opt/ros/humble/setup.bash
cd ~/dev_ws/rover_ws
colcon build --symlink-install
source install/setup.bash
```

### Web HUD 의존성 (이미 설치돼있을 가능성 큼)
```bash
pip3 install --user flask-socketio eventlet
sudo apt install -y ros-humble-web-video-server
```

### `~/.bashrc` 권장 등록 ([docs/SETUP_BASHRC.md](SETUP_BASHRC.md))
```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=113
```

---

## 1. 단일 rover 시연 — GT cheat baseline

> **언제**: 검증된 안전한 시연. mission FSM 풀 흐름 (EXPLORE→APPROACH→PICK→RTB→COMPLETE).

### T1 — Isaac Sim
```bash
export ROS_DOMAIN_ID=113
cd ~/dev_ws/rover_ws/src/a2_isaac
tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00023 --no-overview
```

### T2 — MVP stack (Web HUD 자동 포함)
```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=113
ros2 launch isaac_bringup mvp.launch.py collection_goal:=5
```

### T3 (선택) — rqt 카메라
```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=113
ros2 launch isaac_bringup rqt_views.launch.py
```

### T4 — 브라우저
```bash
xdg-open http://localhost:8088
```

### Quick 시연 (1개만 채집 후 종료)
T2 명령에 `collection_goal:=1` 으로 변경. RTB → MISSION_COMPLETE 까지 ~2-3분.

### terrain 교체 — 다른 terrain 으로 robustness 검증
T1, T2 양쪽 모두 동일 terrain_id 명시:

```bash
# T1
tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00007 --no-overview

# T2 — mvp.launch.py 의 terrain_id default = terrain_00023 라 변경 시 명시
ros2 launch isaac_bringup mvp.launch.py terrain_id:=terrain_00007 collection_goal:=1
```

추천 terrain 셋:
- `terrain_00004` — 옛 baseline
- `terrain_00007`, `terrain_00012`, `terrain_00018` — mid-difficulty
- `terrain_00023` — 발표용 (epic obstacle 4종이 시각 임팩트)

---

## 2. 단일 rover 시연 — T5 localization 정공법

> **언제**: GT cheat 없이 EKF stack (sun_yaw + TRN + wheel_odom + IMU + EDL prior) 으로 좌표 추정. 발표 talking point "GT 없이 동작" 검증.

### T1
```bash
export ROS_DOMAIN_ID=113
cd ~/dev_ws/rover_ws/src/a2_isaac
tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00023 --no-overview
```

### T2 — integrated_localization (`odom_to_estimated_pose` cheat 어댑터 제거)
```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=113
ros2 launch isaac_bringup integrated_localization.launch.py collection_goal:=1
```

### T3 — Web HUD 백엔드 (integrated 에 mission_web_node 미포함)
```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=113
WEB_VIDEO_PORT=8090 ros2 run isaac_supervisor mission_web_node &
ros2 run web_video_server web_video_server --ros-args -p port:=8090
```

### T4 — 브라우저
```bash
xdg-open http://localhost:8088
```

### 검증 명령 — 정공법 확정 (별도 터미널)
```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=113

# (1) cheat 어댑터 부재 확인
ros2 node list | grep -E "odom_to_estimated_pose|ekf_fusion|sun_yaw|trn"
# 기대: ekf_fusion_node / sun_yaw_node / trn_node 있음, odom_to_estimated_pose 없음

# (2) estimated_odom publisher = ekf_fusion
ros2 topic info /rover/estimated_odom --verbose | grep "Node name"
# 기대: ekf_fusion_node

# (3) GT vs EKF 비교 (다르면 정공법 확정)
ros2 topic echo /ground_truth/odom --once | grep -A3 position
ros2 topic echo /rover/estimated_odom --once | grep -A3 position
```

---

## 3. 멀티 rover 시연 (2대)

> **언제**: 발표 robustness 임팩트. 두 rover 동시 자율 운용. mineral 중복 회피 + 정면 충돌 회피 협조 활성.

### T1 — Isaac Sim (2대 spawn)
```bash
export ROS_DOMAIN_ID=113
cd ~/dev_ws/rover_ws/src/a2_isaac
tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00023 --rovers rover_1 rover_2
```

T1 콘솔에서 다음 라인 확인:
```
[run_v3]   patched N topicName attrs, M ScriptNode(s) → namespace /rover_1
[run_v3]   patched N topicName attrs, M ScriptNode(s) → namespace /rover_2
[run_v3] ready — 2대 vehicle namespaces: /rover_1, /rover_2
```

### T2 — mvp_multi (mineral_claims + rover_positions 협조 자동 활성, Web HUD 자동 포함)
```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=113
ros2 launch isaac_bringup mvp_multi.launch.py collection_goal:=1
```

T2 콘솔 확인:
```
[rover_1.mission_manager_node]: rover avoidance 활성 — id=rover_1 ...
[rover_1.mission_manager_node]: mineral claim 협조 활성 — id=rover_1 ...
[rover_2.mission_manager_node]: ...
```

### T3 — 브라우저 (멀티 추적 UI)
```bash
xdg-open http://localhost:8088
```

UI 기능:
- 상단 **`rover_1` / `rover_2` chip** 토글 — 선택된 rover 의 STATUS/카메라 표시
- 좌하단 **COVERAGE MAP** — 두 rover 동시 표시 (rover_1=cyan, rover_2=yellow), path/target 색 분리

### terrain 교체 멀티
```bash
# T1
tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00007 --rovers rover_1 rover_2

# T2
ros2 launch isaac_bringup mvp_multi.launch.py terrain_id:=terrain_00007 collection_goal:=1
```

### 충돌 회피 검증 — 강제 가까운 spawn
> **언제**: 두 rover 가 좁은 공간에서 정면 충돌 가능성. A* 의 dynamic obstacle (다른 rover 위치 inflate) 동작 검증.

```bash
# T1 — spawn 사이 간격 5m 강제 (default 는 meta.json 의 spawn_locations 사용)
tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00023 --rovers rover_1 rover_2 --spawn-spacing 5

# T2 (동일)
ros2 launch isaac_bringup mvp_multi.launch.py collection_goal:=1
```

기대: A* 가 두 rover 위치를 1.2m inflate → 우회 path 생성. 마주칠 때 path 휘어짐 시각 확인.

### Spawn 즉시 RTB (시연 시간 최소화)
> `collection_goal=0` 으로 미네랄 0개 캐도 spawn 직후 RTB → MISSION_COMPLETE.

```bash
# T1
tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00023 --rovers rover_1 rover_2

# T2
ros2 launch isaac_bringup mvp_multi.launch.py collection_goal:=0
```

---

## 4. 검증 명령 모음 (별도 터미널)

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=113

# 노드 list
ros2 node list

# 토픽 namespace 격리 확인 (멀티 시연 시)
ros2 topic list | grep "rover_1/" | head -10
ros2 topic list | grep "rover_2/" | head -10

# action 격리 확인
ros2 action list | grep execute_arm_task
# 기대: /rover_1/execute_arm_task, /rover_2/execute_arm_task

# cmd_vel 발행 빈도 (둘 다 움직이는지)
ros2 topic hz /rover_1/cmd_vel
ros2 topic hz /rover_2/cmd_vel

# phase 실시간 모니터
ros2 topic echo /rover_1/mission/phase &
ros2 topic echo /rover_2/mission/phase

# 협조 토픽
ros2 topic hz /mineral_claims      # APPROACH/PICK_READY 시 ~2 Hz
ros2 topic hz /rover_positions     # ~10 Hz (양쪽 모두 publish)

# Web HUD 의 GT cheat 누수 확인 (subscriber count)
ros2 topic info /ground_truth/odom --verbose | grep Subscription
```

---

## 5. 종료

| 모드 | 절차 |
|---|---|
| 단일 (mvp.launch.py) | T2 `Ctrl+C` → 모든 mission stack + Web HUD 정리 |
| 단일 (integrated_localization) | T2/T3 각각 `Ctrl+C` (mission_web_node + web_video_server) |
| 멀티 (mvp_multi) | T2 `Ctrl+C` → 모든 노드 정리 |
| T1 Isaac Sim | 윈도우 닫기 또는 콘솔 `Ctrl+C` |

---

## 6. 자주 쓰는 launch 인자 표

### mvp.launch.py (단일 rover GT cheat)
| 인자 | default | 의미 |
|---|---|---|
| `terrain_id` | `terrain_00023` | T1 의 `--terrain` 과 일치 필수 |
| `collection_goal` | `5` | N개 채집 시 RTB. 빠른 시연 `:=1` |

### integrated_localization.launch.py (단일 rover T5 정공법)
| 인자 | default | 의미 |
|---|---|---|
| `terrain_id` | `terrain_00023` | EKF prior + TRN heightmap 매칭에 동일 |
| `collection_goal` | `5` | 빠른 시연 `:=1` |

### mvp_multi.launch.py (멀티 rover)
| 인자 | default | 의미 |
|---|---|---|
| `rovers` | `"rover_1 rover_2"` | 공백 구분 namespace 리스트 |
| `terrain_id` | `terrain_00023` | T1 의 `--terrain` 과 일치 |
| `collection_goal` | `5` | 각 rover 의 채집 목표 (0 = spawn 직후 RTB) |
| `enable_dashboard` | `true` | mission_web_node Web HUD 자동 활성 |
| `enable_web_video` | `true` | web_video_server (port 8090) 자동 활성 |

### run_vehicle_v3.py (Isaac Sim)
| 인자 | default | 의미 |
|---|---|---|
| `--terrain` | `terrain_00004` | terrain 폴더 이름 |
| `--rovers ns1 ns2 ...` | (없음) | 멀티 모드. 비우면 단일 rover (`/World/Rover`) |
| `--spawn-spacing N` | `0.0` | 멀티 시 강제 가까운 spawn (A* 회피 검증) |
| `--no-chase` | (false) | chase cam 비활성 (GPU 부담 측정용) |
| `--no-overview` | (false) | overview cam 비활성 |
| `--headless` | (false) | UI 없이 실행 |

---

## 7. 트러블슈팅

| 증상 | 원인 / 조치 |
|---|---|
| 빌드 실패 `rosidl_typesupport_c not found` | ROS humble source 안 됨. `source /opt/ros/humble/setup.bash` 먼저 |
| T2 토픽 없음 | T1 Isaac Sim 안 떴거나 spawn 안 됐음. T1 콘솔 `[run_v3] ready` 확인 후 T2 |
| Web HUD 빈 화면 | (1) `ros-humble-web-video-server` 미설치 (2) port 8088/8090 충돌 (lsof 확인) |
| 멀티 시 카메라 viewport 검정 | T1 의 `--rovers` 인자 빠짐 → multi-rover spawn 실패. 또는 T2 의 rover_namespaces 미적용 |
| matplotlib viewer 자꾸 뜸 | mvp_multi 는 `enable_minimap=False` default — 매번 fresh launch 필요 |
| rover_2 가 베이스 멀리서 MISSION_COMPLETE | RTB A* fail fallback 가능성. T2 콘솔의 `RTB A* failed at ...` 로그 확인. 협조 inflate (1.2m) 가 너무 크면 `enable_rover_avoid:=false` 로 임시 우회 |
| terrain mismatch 로그 (`terrain_00004 ↔ terrain_00023`) | T1 의 `--terrain` 과 T2 의 `terrain_id:=` 가 다름. 일치 확인 |
| sun_yaw `innovation=±π` reject | yaw frame 180° flip — list_to_fix 의 졸업 과제. 시연에선 GT cheat 모드로 우회 |

---

## 8. 빠른 reference — 시연 시나리오별 한 줄

| 목표 | 명령 (T1 → T2) |
|---|---|
| 단일 GT, 5개 채집 풀 demo | `--terrain terrain_00023 --no-overview` → `mvp.launch.py` |
| 단일 GT, 1개 quick | `--terrain terrain_00023 --no-overview` → `mvp.launch.py collection_goal:=1` |
| 단일 T5 정공법 | `--terrain terrain_00023 --no-overview` → `integrated_localization.launch.py collection_goal:=1` |
| 멀티 2대 협조 | `--terrain terrain_00023 --rovers rover_1 rover_2` → `mvp_multi.launch.py collection_goal:=1` |
| 멀티 충돌 회피 검증 | `--rovers rover_1 rover_2 --spawn-spacing 5` → `mvp_multi.launch.py` |
| 멀티 spawn 즉시 RTB | `--rovers rover_1 rover_2` → `mvp_multi.launch.py collection_goal:=0` |
| 다른 terrain 시도 | `--terrain terrain_NNNNN ...` 와 `terrain_id:=terrain_NNNNN` 양쪽 일치 |

---

## 9. 발표 talking point

### 정공법 검증된 부분
- ✅ MVP 미션 FSM (EXPLORE → APPROACH → PICK_READY → RETURN_TO_BASE → MISSION_COMPLETE)
- ✅ T5 EKF stack (wheel_odom + IMU + EDL prior) 기반 좌표 추정 (GT cheat 어댑터 제거됨)
- ✅ 22 terrain robustness (v3 generator + epic obstacle 4종 통일)
- ✅ 멀티 rover 동시 운용 + 협조 (`/mineral_claims`, `/rover_positions`)

### 졸업 과제 (list_to_fix.md)
- ⏳ sun_yaw / TRN 절대 위치 보정 (현재 ±π flip reject + heightmap clip)
- ⏳ TF tree 발행 (vehicle_v3 GT_SCRIPT 에 /tf 추가)
- ⏳ nav2 도입 (TF + localization 완료 후)
- ⏳ Perception z bias (+47cm 평균 — `ik_descend_dz=-0.40` 임시 cheat)
