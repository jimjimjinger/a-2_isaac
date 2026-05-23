# isaac_manipulation

> **트랙 owner**: 최진우 (T2 — Perception + M0609 매니퓰레이션)
> **책임**: M0609 로봇팔 제어 + scripted/IK pick·place·unload·deploy primitives
> **분석 기준**: 2026-05-23 시점 코드 (총 ~5,200 lines)

---

## 1. 모듈 역할 한 줄

Doosan **M0609 6축 + OnRobot RG2** 매니퓰레이터를 Isaac Sim 안에서 구동.
Mars 환경 + AAU Mars Rover 위에 얹어 **cyan 광물 cube pick → cargo bin place**
파이프라인을 시연. ROS2 action(`/execute_arm_task`)으로도 호출 가능 (mock 단계).

크게 두 트랙:
1. **ROS2 패키지 트랙** (`isaac_manipulation/`) — `arm_executor_node` action
   server + 4개 primitive stub. 현재는 mock loop.
2. **Isaac Sim 직접 실행 트랙** (`scripts/`) — 단독 `isaac-python` 실행 스크립트.
   씬 빌더, scripted/IK pick&place demo, wrist 카메라 viewer, vision detector,
   RMPFlow controller wrapper, 진단 도구 등.

---

## 2. 폴더 구조

```text
isaac_manipulation/
├─ isaac_manipulation/                        # ROS2 ament_python 패키지 본체
│  ├─ __init__.py
│  ├─ arm_executor_node.py                    ✅ ExecuteArmTask action server (96 L, mock 단계)
│  └─ primitives/                             ⏳ TODO 주석만 있는 stub들
│     ├─ __init__.py
│     ├─ pick_mineral.py
│     ├─ place_to_cargo.py
│     ├─ unload_to_base.py
│     └─ deploy_solar_panel.py
├─ scripts/                                   # Isaac Sim 직접 실행 (ROS2 패키지 외부)
│  ├─ build_rover_m0609_scene.py              ✅ Day 1 spike — 통합 씬 빌더 (557 L)
│  ├─ pickup_demo.py                          ✅ Scripted cyan cube pickup (656 L)
│  ├─ pickplace_visual_rover.py               ✅ DLS-IK state-machine pick&place (1044 L)
│  ├─ view_wrist_cam.py                       🛠 joint-space Lissajous wrist scan (616 L)
│  ├─ view_wrist_cam_posx.py                  🛠 Cartesian (posx) IK Lissajous scan (898 L)
│  ├─ find_home_pose.py                       🛠 FK home-pose 탐색 (403 L)
│  ├─ m0609_pick_place_controller.py          📦 Isaac Sim PickPlaceController wrapper
│  ├─ m0609_rmpflow_controller.py             📦 Isaac Sim RMPFlow controller wrapper
│  ├─ wrist_camera.py                         📦 Isaac Sim Camera + RPY mount helper
│  ├─ camera_viewer.py                        📦 OpenCV/subprocess/imwrite 3-모드 viewer
│  ├─ viewer_process.py                       📦 system-Python3 외부 viewer 프로세스
│  ├─ realsense_mount.py                      📦 RealSense D455 USD attach helper
│  ├─ vision_tracker_cyan.py                  📦 HSV cyan cube detector
│  ├─ visual_servo_controller.py              📦 P-control pixel→world EE delta
│  ├─ m0609_rg2_description.yaml              📦 RMPFlow robot description (c-space + 충돌 sphere)
│  └─ m0609_rmpflow_common.yaml               📦 RMPFlow tuning weights
├─ package.xml                                # ROS2 패키지 매니페스트
├─ setup.py
├─ setup.cfg
└─ resource/isaac_manipulation                # ament resource marker
```

범례: ✅ 동작 검증 · 🛠 진단/탐색 도구 · 📦 helper/라이브러리 · ⏳ TODO stub

---

## 3. ROS2 패키지 트랙

### 3-1. `package.xml` / `setup.py`

- **빌드 타입**: `ament_python`
- **exec_depend**: `rclpy`, `action_msgs`, `isaac_interfaces`
- **entry_point**: `arm_executor_node = isaac_manipulation.arm_executor_node:main`
- 빌드 결과 실행: `ros2 run isaac_manipulation arm_executor_node`

### 3-2. `arm_executor_node.py` — `ExecuteArmTask` action server

`/execute_arm_task` action 노출. drive_manager(T3)에서 호출 → primitive 실행.
현재는 **mock 구현** — 실제 trajectory/IK 없이 일정 시간 동안 feedback 만 발행
하다가 `success=True` 로 종료. T3-T2 통합 테스트 + action contract 검증용.

**지원 command** (`SUPPORTED_COMMANDS` 집합):
- `pick_mineral`
- `place_to_cargo`
- `unload_to_base`
- `deploy_solar_panel`

위 외 command 는 `_goal_callback` 에서 `GoalResponse.REJECT`.

**파라미터** (`declare_parameter`):
- `mock_duration_sec` (default 2.0) — mock loop 의 전체 길이
- `feedback_hz` (default 5.0) — feedback publish rate

**Feedback 메시지 구조** (각 iteration):
- `state="manipulating"`
- `progress=(step+1)/steps`
- `message=f"executing {goal.command}"`

`Cancel` 응답 ACCEPT. `MultiThreadedExecutor` 로 spin (action server 안정성).

> ⚠️ TODO 표시된 위치(`_execute_callback` 내부)에 추후 `manipulation_primitives/`
> 호출 또는 MoveIt/Isaac Sim articulation control 을 끼워넣어 실 동작 전환.

### 3-3. `primitives/` 폴더

4개 파일 모두 **주석 한 단락짜리 stub**. ROS2 패키지가 빌드만 깨지지 않도록
유지된 자리표시자.

| 파일 | 호출 시점 | 책임 |
|------|----------|------|
| `pick_mineral.py` | `ExecuteArmTask command=pick_mineral` | approach/grasp/lift 시퀀스 |
| `place_to_cargo.py` | `command=place_to_cargo` | grasp pose → rover cargo bay drop |
| `unload_to_base.py` | `command=unload_to_base` | basecamp 에서 cargo 전체 unload |
| `deploy_solar_panel.py` | `command=deploy_solar_panel` | 저배터리 복귀 시 패널 전개 |

**Tier 1.5 전략**: 진짜 IK + force feedback 안 함. scripted joint trajectory +
광물 텔레포트(FixedJoint attach) 로 시각 시연. 자세한 결정: docs DECISIONS #005.

---

## 4. Isaac Sim 직접 실행 스크립트 (`scripts/`)

### 4-1. `build_rover_m0609_scene.py` — Day 1 spike 통합 씬 빌더 ✅

**산출물**: rover + M0609 + RG2 가 결합된 단일 articulation 씬 (mars 환경 포함).

**Pipeline** (`build_scene()` 6단계, [1/5]~[5/5] + assembly):

1. `mars_exploration_world.usd` reference 로드 (T1 환경) + `PhysicsScene`(중력 3.72 m/s²) 추가 + terrain mesh `CollisionAPI`/`MeshCollisionAPI`(meshSimplification) 보강
2. `Mars_Rover.usd` reference, world pos `(5.0, 0.0, 1.0)` 에 spawn, drive freeze
3. `m0609_isaac_sim.urdf` URDF import → `TransformPrimSRTCommand` 로 `(5.15274, 0, 1.21232)` 로 이동 (USD xformOp 만으로는 PhysX sync 안 됨)
4. `onrobot_rg2.urdf` URDF import → `RobotAssembler` 로 M0609 `link_6` ↔ RG2 `angle_bracket` 결합
5. rover `Body` 자식에 `M0609_Mount` Xform 생성 (offset `(0.15274, 0, 0.21232)`) → `RobotAssembler` 로 M0609 `base_link` 가 그 mount 위치로 정렬 (단일 articulation)
6. `_paint_subtree_dark()` 로 M0609 + RG2 서브트리에 dark `UsdPreviewSurface` 강제 재바인딩 (URDF in-memory stage 한계로 본체 머터리얼만 흰색 유지)

**핵심 hack**:
- URDF importer 가 cwd 에 임시 mesh USD 를 만듦 → 시작 시 `tempfile.mkdtemp()` 로 chdir (read-only 위치에서도 실행 가능하게)
- `RobotAssembler.begin_assembly()` 가 `EditTarget` 을 sublayer 로 바꿔두는데 `finish_assemble()` 가 원복 안 함 → 호출 직후 `stage.SetEditTarget(root_layer)` 로 명시적 복구
- `Articulation.set_world_pose()` 호출 X — FixedJoint 가 두 articulation 의 상대 위치를 잡고 있어 한쪽만 강제 이동시키면 분리됨

**실행**:
```bash
isaac-python ~/dev_ws/rover_ws/src/a2_isaac/isaac_manipulation/scripts/build_rover_m0609_scene.py
isaac-python … build_rover_m0609_scene.py --auto-play
# 결합 stage 를 USD 로 저장 (reference 유지, 가벼움):
isaac-python … --export ../isaac_sim/assets/rover_m0609_assembled.usd
# 자가포함(flatten, 다른 PC 배포용, 큰 파일):
isaac-python … --export …/rover_m0609_assembled.usd --export-flatten
```

**튜닝 상수** (스크립트 상단):
```python
SPAWN_X, SPAWN_Y, SPAWN_Z = 5.0, 0.0, 1.0
M0609_MOUNT_OFFSET_X = 0.15274
M0609_MOUNT_OFFSET_Y = 0.0
M0609_MOUNT_OFFSET_Z = 0.21232   # GUI 시각 검증값. ↓ 박힘, ↑ 떠 있음
```

### 4-2. `pickup_demo.py` — Scripted cube pickup ✅

build 패턴은 4-1 과 동일하나 **ground level**(`SPAWN_Z=0.30`)에서 spawn 후
`/World/Joints/RoverAnchor` `FixedJoint` 로 rover.Body 를 world 에 anchor →
자유낙하 없음. 도달 가능 거리에 5cm cyan cube + `CubeAnchor` FixedJoint 로
spawn 위치 고정.

**시퀀스**(`_run_demo()`, 시간 기반 phase):

```
home_settle (3 s, drive 적용 대기)
   → detect (cyan 보일 때까지 ≤10 s)
   → open (RG2 finger drive target=0.0, 1.5 s)
   → descend (simplified — 실제 IK 없음, 2 s)
   → close (finger target=0.8 → CubeAnchor 제거 → cube↔gripper_body FixedJoint attach → LIFT joint pose 적용)
   → lift (3 s)
   → done
```

**HOME / LIFT joint pose** (deg): `(0, 90, 90, 0, 0, 0)` → `(0, 60, 90, 0, 0, 0)`
(joint_2 90°→60° 로 tool0 z ≈ 0.16 → 0.43, +27 cm 상승)

**Articulation 제어**:
- `_init_m0609_articulation()` — `SingleArticulation(prim_path=robot_root)` 를
  `world.scene.add()` (world.reset 전에 등록).
- `_set_m0609_pose()` — `ArticulationController.apply_action(ArticulationAction(joint_positions=…, joint_indices=…))`. `teleport=True` 면 `set_joint_positions()` 로 즉시 도달.
- `_drive_finger()` — RG2 `finger_joint` / `right_inner_knuckle_joint` 의
  `UsdPhysics.DriveAPI("angular")` target 직접 set.

**시각화**: wrist `Camera` 프림(angle_bracket 자식, RPY (0,-90,90)°) + `cv2`
윈도우. `isaac_perception.cyan_detector.CyanDetector` import 해서 HSV 검출
bbox/crosshair/mask side-by-side overlay. cv2 build 가 headless 면 imshow skip.

**파라미터 노트**:
- 카메라 해상도 640×480, local translate `(0, 0.045, 0.05)` from `angle_bracket`
- HOME 자세는 `find_home_pose.py` 로 검증된 값 (dist 0.07 m, 큐브 바로 위)
- `--auto-play`, `--max-sec 180` 옵션

### 4-3. `pickplace_visual_rover.py` — DLS IK 기반 pick & place ✅

**전체 pipeline 의 시연용 endpoint**. ground-truth 좌표 + (옵션) vision-driven
deprojection 두 모드로 cube 위치 추정 → DLS Jacobian IK 로 EE 추종 → 10-state
state machine 으로 pick & place 전 과정.

**Scene 차이점**:
- Mars 환경 + **pre-assembled `Vehicle.usd`** (`/home/rokey/dev_ws/rover_ws/src/Vehicle.usd`)
  reference. URDF import 안 함 → 시작 빠르고 안정.
- `SETTLE_FRAMES=120` 자유낙하 후 rover Body world pose 캡쳐 → `RoverAnchor` FixedJoint
- cube 는 `DynamicCuboid` 로 `(spawn + 0.7, 0, 0.5)` 에 drop → `static_friction=1.2` PhysicsMaterial
- finger 4개 link 에 `static_friction=4.0` PhysicsMaterial 추가 (grip 안정)
- wrist 카메라: `realsense_mount.attach_realsense_d455()` 로 D455 USD reference, 내장 `Camera_OmniVision_OV9782_Color` 프림 사용 (extra Z-yaw +90°)

**State machine** (`PickPlaceStateMachine`):

```
MOVE_TO_HOME → SEARCH → APPROACH → DESCEND → GRASP_CLOSE → ATTACH_LIFT
            → MOVE_TO_GOAL → PLACE_DESCEND → RELEASE → RETREAT → DONE
```

- HOME 자세: `(0, 0, 90, 0, 90, 0)°` (j5=90 → 카메라 ↓)
- SEARCH 는 vision detection 결과(`vision_cube_xyz`, depth `distance_to_image_plane` deprojection) 우선, 120 step 초과 시 ground-truth fallback. vs GT 오차 mm 로깅
- APPROACH 높이: cube top + 0.30 m
- GRASP 높이: cube top + 0.20 m (reach 한계 vs joint limit 트레이드오프)
- LIFT 높이: cube top + 0.45 m
- `IK_POS_TOL=0.04` m (4 cm), `IK_MAX_STEPS_PER_PHASE=400` (~7 s @ 60 Hz)
- `IK_GRASP_REACH_THRESHOLD=0.10` m — DESCEND 가 timeout 인데 pos err > 10 cm 면 grasp 거부 → `DONE` (ABORT, 안전)
- RELEASE 는 IK 실패해도 항상 release (cube 떨굼)

**DLS Jacobian IK** (`_ik_dls_step`):
- 자코비안: `Articulation.get_jacobians()` 또는 `articulation_view.get_jacobians()`, mobile base 면 col_offset=6 자동 보정
- 오차벡터: `[pos_err(3); IK_ORIENTATION_WEIGHT * rot_err(3)]`, rot_err 는 quat-to-axis-angle
- `dq = J⁺ err + N · per_joint_null_gain · (q_home − q_cur)`
  - `IK_NULL_GAIN_PER_JOINT = [0, 0.1, 0.1, 1.5, 1.5, 1.5]` — wrist 3개(joint_4~6)는 HOME 강제 유지, j1 자유, j2/j3 약한 bias (limit hit 방지)
- joint limit clamping (`IK_JOINT_LIMITS_DEG`)
- `α=0.4`, `damping=0.10`, `IK_NULLSPACE_GAIN=0.6`, `IK_ORIENTATION_WEIGHT=1.0`

**Grasp attach**: `_attach_cube_to_link()` 는 grasp 시점의 cube↔link 상대 pose 를
캡쳐해 그대로 FixedJoint 생성 → cube 가 자석에 끌리듯 순간이동하지 않음.

**실행**:
```bash
isaac-python … pickplace_visual_rover.py                 # default spawn (4.5,-1.0)
isaac-python … pickplace_visual_rover.py --spawn 5.0,0.0 --goal-offset 0.0,-0.3,0.0
```

**의존 sibling 모듈** (script dir 가 sys.path 에 자동 추가):
`wrist_camera.WristCamera`, `vision_tracker_cyan.CyanCubeTracker`,
`camera_viewer.CameraViewer`, `realsense_mount.attach_realsense_d455`

### 4-4. Wrist-cam Lissajous 진단 — `view_wrist_cam.py` / `view_wrist_cam_posx.py` 🛠

cube 가 보일 때까지 EE 를 **8-figure (Lissajous) 궤적**으로 sweep 하는 cyan 탐색
진단 도구. 둘은 제어공간만 다름:

| 항목 | `view_wrist_cam.py` | `view_wrist_cam_posx.py` |
|------|---------------------|--------------------------|
| 제어 | joint-space (j1, j2 정현파 가산, posj-style) | Cartesian (link_6 world XY, DLS IK) |
| 진폭/주파수 | `±30° / 0.12 Hz` (j1), 별도 (j2) | `--xy-amp`, `--xy-freq` |
| 씬 로드 | URDF import + RobotAssembler | 사전 통합 `Vehicle.usd` reference |
| 정착 | 즉시 FixedJoint anchor | 120 frame 자유낙하 → anchor |
| IK | 없음 | `_ik_dls_step` α=0.5, damping=0.1, nullspace_gain=0.6 |

공통: `scan`(탐색) → `found`(검출 후 4 s hold) → `resume` state machine.
검출 기준 `DETECTION_MIN_AREA_PX2=200`. `isaac_perception.cyan_detector` 사용.

옵션:
```bash
view_wrist_cam.py:        --pose, --cam-xyz, --cam-rpy
view_wrist_cam_posx.py:   --xy-center, --z, --xy-amp, --xy-freq
```

### 4-5. `find_home_pose.py` — FK 기반 HOME 자세 탐색 🛠

13개 candidate joint pose 를 순차로 `set_joint_positions()` 텔레포트 →
`tool0` world pose FK 계산 → TARGET(cube + 0.1 m 위) 과의 거리로 ranking.
출력: best pose 와 거리(mm). `HOME_JOINT_DEG=(0, 90, 90, 0, 0, 0)` 의 근거가 된
도구.

### 4-6. RMPFlow controller wrapper — `m0609_*` 📦

- **`m0609_rmpflow_controller.py`** — `mg.MotionPolicyController` 상속.
  내부 `lula.RmpFlow(robot_description_path, rmpflow_config_path, urdf_path, end_effector_frame_name="link_6")` 생성 → `ArticulationMotionPolicy` 로 articulation 에
  bind. `reset()` 마다 `set_robot_base_pose()` 재호출.
  - 기본 yaml/urdf 경로: 스크립트 폴더의 `m0609_isaac_sim.urdf`, `m0609_description.yaml`, `m0609_rmpflow_common.yaml` (생성자 인자로 override 가능)
- **`m0609_pick_place_controller.py`** — `manipulators_controllers.PickPlaceController` 상속. 위 RMPFlow controller 를 cspace controller 로 주입,
  RG2 `ParallelGripper` 와 결합. `events_dt=[0.008, 0.005, 0.1, 0.1, 0.0025, 0.001, 0.0025, 1, 0.008, 0.08]` 10-event 시퀀스 (Isaac Sim 표준 pick&place state DT).

> 위 두 모듈은 라이브러리. 현재 demo 스크립트(`pickup_demo`, `pickplace_visual_rover`)
> 는 더 단순한 직접 driving 으로 작성돼 있어 이 wrapper 를 직접 import 안 함.
> RMPFlow 정식 통합 시 활용 예정.

### 4-7. Wrist camera + viewer infrastructure 📦

- **`wrist_camera.py`** — `WristCamera` 클래스. `isaacsim.sensors.camera.Camera`
  (없으면 `omni.isaac.sensor` fallback) 래핑. 부모 prim 자식으로 Xform+Camera prim 생성, RPY(deg) → quat 변환(scipy 있으면 그것, 없으면 수동), `get_rgb()` 로 (H,W,3) uint8 반환. `from_existing_prim` 클래스 메서드로 이미 만들어진 카메라 prim 도 수용.
- **`camera_viewer.py`** — `CameraViewer`. 3-tier fallback:
  1) Isaac Sim cv2 의 GTK 빌드 → 직접 `cv2.imshow`
  2) 실패 시 `viewer_process.py` 를 system Python3 subprocess 로 띄워 stdin 으로 frame stream
  3) 그것도 실패 시 `/tmp/wrist_cam_*.png` `imwrite` fallback
  - `update(rgb, det, state_str, extra_lines)` → bbox/crosshair/state-overlay 그려서 표시, key code 반환
- **`viewer_process.py`** — 별도 process. stdin protocol: `0x01 size_LE BGR_bytes`(frame), `0x02`(mask), `0xFF`(shutdown). stdout 에 1-byte key.
- **`realsense_mount.py`** — `attach_realsense_d455(parent, child_name, translation, rpy_deg)`. RealSense D455 USD 를 부모 prim 자식 Xform 으로 reference, RigidBody/Collision 비활성화(부모가 rigid 라 중복 방지). 자산 경로는 Isaac Sim 의 `isaacsim.storage` / `nucleus` 로 자동 해석.

### 4-8. Vision / servo

- **`vision_tracker_cyan.py`** — `CyanCubeTracker`. BGR→HSV → `H ∈ [70°,110°]`, `S, V ≥ 40` (Mars 조명 desaturation 고려) → open/close morphology → 가장 큰 contour → `Detection(found, cx, cy, area, bbox, mask)` dataclass 반환.
- **`visual_servo_controller.py`** — `VisualServoController`. 픽셀 오차(cx, cy) →
  `kp` gain + pixel-to-world 변환 → EE world XY delta. `tolerance_px=8`, 연속 `lock_frames=15` frame 안정 시 `is_locked=True`.

`pickplace_visual_rover.py` 는 `vision_tracker_cyan` 만 사용. `visual_servo_controller`
는 현재 demo flow 에서 미사용 — 후속 closed-loop servo 전환 시 활용 후보.

### 4-9. RMPFlow YAML 설정 📦

- **`m0609_rg2_description.yaml`** — 로봇 기구학/기하 baseline.
  - c-space 6 joint (joint_1…joint_6), default `[0, -π/2, π/2, 0, 0, 0]`
  - per-joint 가속/저크 한계
  - base→tool0 chain 따라 약 16개 collision sphere (반지름 0.045–0.10 m) — self-collision 검사
- **`m0609_rmpflow_common.yaml`** — RMPFlow tuning weights.
  - 8개 behavior module: cspace target/trajectory/affine, joint-limit avoidance(buffer 0.02–0.05 rad, exploder 1e-3), velocity cap(3.927 rad/s), target reach (accel_p_gain=30), task-space axis target(p_gain=210), collision repulsion(gain=800, metric radius 0.5 m), damping
  - canonical resolver `max_acceleration_norm=20 m/s²`
  - body cylinder obstacle 정의 (base/shoulder 본체)
  - body collision controllers 가 link_3…tool0 에 damping sphere 부착

> ⚠️ `m0609_rmpflow_controller.py` 는 기본 description yaml 경로로 같은 폴더의
> `m0609_description.yaml` 을 찾지만, 현재 repo 에 있는 파일명은
> `m0609_rg2_description.yaml`. 사용 시 명시적으로 `robot_description_path=` 인자로
> 넘기거나 심볼릭 링크/rename 필요.

---

## 5. 실행 가이드 (Quick Start)

전제: `isaac-python` alias 가 `~/.bashrc:128` 에 등록돼 있음(memory: isaac-python alias).
Isaac Sim 5.1 (Python 3.11), `~/dev_ws` 워크스페이스, RTX 5080 Laptop 80W PRIME on-demand.

### 5-1. 통합 씬만 띄우기 (Day 1 baseline)

```bash
isaac-python ~/dev_ws/rover_ws/src/a2_isaac/isaac_manipulation/scripts/build_rover_m0609_scene.py
# GUI 창 뜨면 Spacebar → rover/M0609 동반 자유낙하 확인
```

### 5-2. Cyan cube scripted pickup

```bash
isaac-python ~/dev_ws/rover_ws/src/a2_isaac/isaac_manipulation/scripts/pickup_demo.py --auto-play
# OpenCV 창에 wrist 시점 + HSV mask + bbox 표시. phase 로그 콘솔.
```

### 5-3. DLS-IK 기반 pick&place (full pipeline)

```bash
isaac-python ~/dev_ws/rover_ws/src/a2_isaac/isaac_manipulation/scripts/pickplace_visual_rover.py
# state 진행 로그 + CameraViewer 윈도우 (RGB+mask+overlay)
```

### 5-4. ROS2 action server (mock)

```bash
cd ~/dev_ws/rover_ws && colcon build --packages-select isaac_manipulation
source install/setup.bash
ros2 run isaac_manipulation arm_executor_node
# 다른 터미널에서:
ros2 action send_goal /execute_arm_task isaac_interfaces/action/ExecuteArmTask \
    "{command: 'pick_mineral', target_id: 'mineral_01'}" --feedback
```

---

## 6. drive_manager (이찬휘 T3)와의 인터페이스

```
drive_manager_node (T3 이찬휘)
   │  PICK phase 진입
   │  ↓  action call: ExecuteArmTask
   │       command ∈ {pick_mineral, place_to_cargo, unload_to_base, deploy_solar_panel}
   │       target_id / target_x,y,z
arm_executor_node (T2)
   │  1. (real) primitives/* 호출
   │     (mock) feedback_hz Hz 로 progress feedback
   │  ↓  action result
   │       success: bool
   │       message: "{command} completed"
```

I3/I4 인터페이스 상세: `docs/interfaces/INTERFACE_CONTRACTS.md` I3 섹션,
action 정의: `isaac_interfaces/action/ExecuteArmTask.action`.

---

## 7. 의존성 요약

| 외부 모듈 | 사용처 |
|----------|--------|
| `rclpy`, `isaac_interfaces` | ROS2 action server |
| `isaacsim.SimulationApp`, `isaacsim.core.api.World` | 모든 scripts/ 진입점 |
| `isaacsim.asset.importer.urdf` | URDF import (build_rover_m0609_scene, pickup_demo, view_wrist_cam) |
| `isaacsim.robot_setup.assembler.RobotAssembler` | rover ↔ M0609 ↔ RG2 결합 |
| `isaacsim.core.prims.SingleArticulation` | articulation 등록, jacobian, joint pos |
| `isaacsim.core.utils.types.ArticulationAction` | joint position drive |
| `isaacsim.sensors.camera.Camera` | wrist 카메라 frame 획득 |
| `isaacsim.robot_motion.motion_generation.lula.RmpFlow` | (예약) RMPFlow controller |
| `isaac_perception.cyan_detector.CyanDetector` | pickup_demo, view_wrist_cam* — sibling package |
| `cv2`, `numpy` | 비전 + 시각화 |

**Asset 경로** (모두 in-repo, self-contained — 다른 PC 에서도 그대로 작동):
```
~/dev_ws/rover_ws/src/a2_isaac/isaac_sim/worlds/mars_exploration_world.usd
~/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/rover/Mars_Rover.usd
~/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/doosan-robot2/urdf/m0609_isaac_sim.urdf
~/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/onrobot_rg2/urdf/onrobot_rg2.urdf
~/dev_ws/rover_ws/src/Vehicle.usd                              # pre-assembled (pickplace_visual_rover, view_wrist_cam_posx)
```

---

## 8. 진행 상태 / 알려진 한계 / 다음 단계

### 검증된 동작 ✅
- rover + M0609 + RG2 결합 단일 articulation 자유낙하 안정
- HSV cyan cube detector + wrist 카메라 + bbox/mask overlay
- Scripted joint trajectory + cube↔gripper FixedJoint attach 로 lift 시연 (`pickup_demo`)
- DLS Jacobian IK + nullspace bias + joint limit clamping + 10-state pick&place pipeline (`pickplace_visual_rover`)
- Vision-driven cube XYZ 추정 (depth deprojection) + GT 와의 mm 단위 오차 로깅
- ROS2 `/execute_arm_task` action server (mock)

### 알려진 한계 ⚠️
- M0609 본체 머터리얼이 흰색 (URDF in-memory stage 의 한계, RG2 만 dark 적용됨)
- DESCEND 가 reach 한계 근처에서 pos_err > 10 cm 면 grasp 거부 (`IK_GRASP_REACH_THRESHOLD`)
- Visual servo (`visual_servo_controller.py`) 는 작성만 됨, demo flow 에서 미사용
- `primitives/*` 가 stub → action server 는 mock loop 로 동작
- `m0609_rmpflow_controller.py` 의 default description yaml 이름 mismatch (`m0609_description.yaml` vs `m0609_rg2_description.yaml`)

### 다음 단계 (T2 후속) 📋
1. `arm_executor_node._execute_callback` → `primitives/pick_mineral.py` 등에 실 동작 (`pickplace_visual_rover` state machine 의 재사용 가능 부분 분리)
2. RMPFlow controller wrapper 실 통합 — yaml 경로 수정 + `pickup_demo` 에 옵션으로 wire
3. T5(추정 pose) + T3(pick_request) 받아 첫 통합 (Day 4 마일스톤)
4. Visual servo closed loop 로 grasp 미세조정 (현재는 ground-truth/vision-deproject 의 1-shot 추정)

---

## 9. 한 줄 요약

> **최진우의 M0609 매니퓰레이션.** ROS2 action server 는 mock, scripts/ 는 진짜
> 동작. scripted trajectory + cube 텔레포트 (Tier 1.5) 부터 DLS IK 기반 10-state
> pick&place 까지 단계별 산출물 갖춤. 다음은 primitive 실 구현 + T5/T3 통합.
