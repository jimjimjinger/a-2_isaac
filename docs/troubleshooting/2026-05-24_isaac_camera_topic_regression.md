# Isaac Sim 카메라 ROS2 토픽 회귀 진단 보고서

| | |
|---|---|
| **작성일** | 2026-05-24 |
| **작성자** | T4 성선규 |
| **대상 환경** | Ubuntu 22.04 / RTX 5070 Ti Laptop / driver 580.159.03 / Isaac Sim 5.1 / ROS2 humble |
| **영향 범위** | vehicle_v3.usd 의 6개 카메라 토픽 + ReadGtPose ScriptNode 동시 불능 |
| **상태** | ✅ 근본 원인 확정, 우회 검증 완료 |

---

## 한 줄 요약

**사용자가 직접 빌드한 source build Isaac Sim 5.1 의 `omni.syntheticdata` ↔ `omni.replicator.core` ↔ `isaacsim.ros2.bridge` 통합이 incomplete 한 상태였고**, 그 결과 카메라 토픽 발행이 dtype 에러로 영구 fail. **PyPI binary release (`pip install isaacsim==5.1.0`) 로 같은 USD 를 띄우면 정상 동작**함이 확인되어, 환경 측 회귀로 진단 종료.

---

## 1. 증상

`vehicle_v3.usd` 를 `run_vehicle_v3.py` 로 띄웠을 때:

- 6개 카메라 토픽 (`/camera/rover/image_raw`, `/camera/rover/depth`, `/camera/rover/camera_info`, `/camera/wrist/image_raw`, `/camera/wrist/depth`, `/camera/wrist/camera_info`) 이 모두 `ros2 topic list` 에 안 나타남
- Isaac Sim stdout 에 다음 에러가 stage open 직후 **단 한 번** 발생:
  ```
  [Error] [isaacsim.core.nodes.impl.base_writer_node]
  Could not process writer attach request
  (..., '/Render/OmniverseKit/HydraTextures/Replicator'),
  Unable to write from unknown dtype, kind=i, size=0
  ```
- `kind=i` (integer, RGB image), `kind=f` (float, depth/ScriptNode) 두 패턴 모두 발현
- 한 번 attach fail 후 retry 없음 → 토픽 자체가 영원히 생성 안 됨

---

## 2. 진단 타임라인

| # | 가설 | 검증 방법 | 결과 |
|---|---|---|---|
| 1 | `.bashrc` 의 `rover_ws` auto-source 가 PYTHONPATH 오염 → Isaac Python 3.11 ↔ ROS humble Python 3.10 ABI 충돌 | rover_ws auto-source 끄고 wrapper (`temp/ros-isaac-python`) 로 ROS env 명시 주입 | 부분 해결 — rclpy load 는 성공, 카메라는 여전히 안 나옴 |
| 2 | run 스크립트에 `isaacsim.ros2.bridge` extension 활성화 누락 → `ROS2Context` 노드 type 등록 안 됨 | `from isaacsim.core.utils.extensions import enable_extension` 추가 | `Could not find node type interface` 경고 해결. 그러나 토픽 미발행 지속 |
| 3 | DLSS minimum resolution (300px) 미만으로 RTX render output 이 빈 dtype 으로 들어옴 | 해상도 640×480 → 1280×720 으로 증가 | DLSS 경고는 사라졌으나 dtype 에러 그대로 |
| 4 | Shader cache corruption | `~/.cache/ov/shaders` + `~/.cache/warp` + `~/.cache/nvidia` 삭제 후 재시도 | 변화 없음 |
| 5 | driver 580.159 회귀 | 동료 PC (RTX 5080 Laptop, **동일 driver 580.159**) 비교 | 동료 환경에선 정상 → driver 단독 원인 아님 |
| 6 | Python `Camera` API 로 USD-baked CameraHelper 우회 | `isaacsim.sensors.camera.Camera` + `rclpy` publisher 패턴 | 같은 `omni.syntheticdata` backend 거치므로 segfault (`Py_FinalizeEx + omni.syntheticdata`) |
| 7 | source build Isaac Sim 5.1 자체가 incomplete | PyPI binary (`pip install isaacsim==5.1.0`) 별도 venv 설치 후 동일 USD 비교 | **PyPI 환경에선 6개 토픽 모두 정상 발행 (64 Hz). 원인 확정** |

---

## 3. 진짜 원인 (확정)

### "통합 incomplete" 의 정확한 의미

Isaac Sim 의 카메라 토픽 발행은 다음 sub-package 체인을 거칩니다:

```
isaacsim.ros2.bridge.ROS2CameraHelper   (image 발행)
       ↓ 호출
omni.replicator.core.NodeWriter         (image 데이터 형식 변환)
       ↓ 호출
omni.syntheticdata                      (RTX render → numpy array 변환)
       ↓ 호출
omni.graph.core                         (노드 그래프 평가 엔진)
```

각 sub-package 는 서로 **data 형식 (numpy dtype, struct layout 등) 의 약속** 을 지켜야 합니다. 사용자 source build 의 어느 한 layer 에서 그 약속이 깨져 있어, writer attach 시점에 dtype 정보가 비어 (`kind=i/f, size=0`) 들어오고, 그 결과 attach fail → 토픽 생성 실패.

PyPI binary 는 NVIDIA 가 한 set 로 정합성 보장된 채 묶어 배포하므로 이 incomplete 가 없음.

### 발현 범위

source build 환경에서 같은 패턴으로 동시 불능:

| 노드 | 토픽/입력 | dtype 에러 |
|---|---|---|
| ROS2CameraHelper (Rover RGB) | `/camera/rover/image_raw` | `kind=i, size=0` |
| ROS2CameraHelper (Rover Depth) | `/camera/rover/depth` | `kind=f, size=0` |
| ROS2CameraInfoHelper (Rover) | `/camera/rover/camera_info` | `kind=i, size=0` |
| ROS2CameraHelper (Wrist RGB) | `/camera/wrist/image_raw` | `kind=i, size=0` |
| ROS2CameraHelper (Wrist Depth) | `/camera/wrist/depth` | `kind=f, size=0` |
| ROS2CameraInfoHelper (Wrist) | `/camera/wrist/camera_info` | `kind=i, size=0` |
| ScriptNode (ReadGtPose) | (GT odometry) | `kind=f, size=0` |

---

## 4. driver 580.159 업그레이드와의 연관성

**부분 연관** — driver 자체가 직접 원인은 아니지만, **업그레이드가 trigger 였을 가능성**이 큼:

1. **동료 PC (동일 driver, 다른 GPU) 정상** → driver 가 직접 원인이면 동료도 깨졌어야 함.
2. 메모리 [reference_driver_580_159_scriptnode](../../../../.claude/projects/-home-sungyu-dev-ws-rover-ws-src-a2-isaac/memory/reference_driver_580_159_scriptnode.md) 에 따르면 driver 580.159 는 **별도의** ScriptNode numpy dtype 명시 fix 를 이미 요구했고 (ACK_SCRIPT/GT_SCRIPT), 그건 다른 layer 의 호환 이슈.
3. **유력 시나리오**: 사용자의 source build 는 driver 580.159 업그레이드 **이전 시점**에 빌드되어, 빌드 산출물의 일부 sub-package (특히 syntheticdata/replicator) 가 새 driver 의 RTX behavior 와 mismatch. 동료는 PyPI binary 라 NVIDIA 의 최신 정합성 보장 환경에서 빌드된 산출물 사용 → 영향 없음.

확정하려면 `_build/` 디렉터리의 modification time 과 driver 580.159 install 시점을 비교하면 됨.

---

## 5. 우회 / 해결 방법

### 5.1 즉시 해결책 (검증 완료)

PyPI binary release 별도 venv 설치 후 동일 USD 사용:

```bash
# 별도 venv (소스 빌드와 분리)
python3.11 -m venv ~/dev_ws/isaac_sim_pypi/venv
source ~/dev_ws/isaac_sim_pypi/venv/bin/activate
pip install --upgrade pip
pip install isaacsim[all,extscache]==5.1.0 --extra-index-url https://pypi.nvidia.com

# wrapper (temp/ros-isaac-python-pypi) 로 ROS env 주입 + venv python 호출
temp/ros-isaac-python-pypi temp/run_min_camera_usd.py isaac_sim/assets/vehicle/vehicle_v3.usd

# → /camera/rover/image_raw 등 6개 토픽 64 Hz 발행 확인
```

### 5.2 시도했으나 효과 없었던 우회들 (재시도 비용 회피용 기록)

- Shader cache 삭제 (1.6 GB) — 효과 없음
- 해상도 증가 (640×480 → 1280×720) — DLSS 경고는 해소되나 dtype 에러 무관
- Python `Camera` API 우회 — 같은 backend 거쳐 segfault 까지 감

### 5.3 시도하지 않은 옵션

- Source build clean rebuild — 시간 비용 큼 (수 시간), incomplete 원인이 빌드 자체일지 환경일지 불확실
- Isaac Sim 5.0 다운그레이드 — 다른 호환성 영향 미지수

---

## 6. 환경 의존성 매트릭스 (산출물 재사용 가능성)

| 산출물 | 환경 독립? | 다른 팀원이 그대로 써도 OK? |
|---|---|---|
| `*.usd` (Camera + 그래프 박힌 USD) | ✅ 표준 USD 포맷 | ✅ 어디서 열든 같은 stage 구성 |
| 빌드/run Python 스크립트 (`temp/*.py`) | ✅ 표준 isaacsim API | ✅ Isaac 환경 깔려있으면 작동 |
| wrapper (`temp/ros-isaac-python*`) | ❌ 사용자 hardcoded path | ❌ 팀원 path 가 다르면 수정 필요 |
| **카메라 토픽 발행 (실제 동작)** | ❌ **Isaac Sim 빌드 정합성 의존** | ⚠️ 팀원 환경에 따라 달라짐 |

---

## 7. 팀에 대한 권장

1. **팀 표준을 PyPI binary release 로 통일** — 모든 팀원이 같은 정합성 보장된 환경 사용. 설치 한 줄로 끝. 본 incomplete 이슈 재발 방지.
2. **각자 환경 점검** — 이미 source build 쓰는 팀원이 있다면 같은 dtype 에러 발현 여부 확인 (단순 RGB 카메라 토픽 한 개로 5분이면 가능)
3. **본 보고서의 진단 절차** (가설 → PyPI 비교 검증) 를 동일 카테고리 증상 발생 시 사용

---

## 8. 부록 — 사용된 진단 도구

작업 기간 중 `temp/` 에 만들어둔 진단 도구들 (커밋 안 함, 일회성):

| 파일 | 역할 |
|---|---|
| `temp/build_min_camera_usd.py` | 최소 minimum USD (Camera + 광원 + 큐브, 그래프 0개) |
| `temp/build_step1_tick_context.py` | Step 1: OnPlaybackTick + ROS2Context 만 |
| `temp/build_step2_render_product.py` | Step 2: + IsaacCreateRenderProduct |
| `temp/build_step3_camera_rgb.py` | Step 3: + ROS2CameraHelper (RGB). 회귀 위치 확정 |
| `temp/run_min_camera_usd.py` | 공통 run 스크립트 (USD path 인자 받음) |
| `temp/run_camera_python_pub.py` | Python rclpy publisher 우회 (실패 — 같은 backend) |
| `temp/ros-isaac-python` | source build 용 wrapper (humble_ws source + LD_LIBRARY_PATH) |
| `temp/ros-isaac-python-pypi` | PyPI binary 용 wrapper |
| `temp/_dump_vehicle_v3_graph.py` | vehicle_v3 의 ActionGraph 구조 비교용 dump |
| `temp/_dump_camera_test.py` | 빌드된 USD 의 노드/attr 검사 |

점진적 빌드 패턴 (step1 → step2 → step3) 은 향후 다른 그래프 회귀 진단 시 재사용 권장.

---

## 9. 관련 메모리 / 문서

- 메모리 `reference_driver_580_159_scriptnode.md` — driver 580.159 관련 다른 호환 이슈 (별개 layer)
- 메모리 `reference_isaac_sim_python.md` — Isaac Sim source build python 경로
- 메모리 `project_rover_v2.md` — vehicle_v3 라이브 경로 정의
- `docs/interfaces/INTERFACE_CONTRACTS.md` — 6개 카메라 토픽의 인터페이스 계약

---

## 10. 추가 검증 (라이브 경로 완성)

본 보고서 작성 후 동일 세션 내에서 vehicle_v3 의 전체 ROS2 인터페이스 + 라이브 경로까지 검증 완료:

### 10.1 vehicle_v3 11개 인터페이스 (PyPI 환경, 2026-05-24)

발행 9개 모두 ~32 Hz 안정적:

| 토픽 | 타입 | Hz |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | 32.26 |
| `/joint_states_raw` | `sensor_msgs/JointState` (27 DOF) | 32.00 |
| `/ground_truth/odom` | `nav_msgs/Odometry` | 32.45 (ScriptNode 정상) |
| `/camera/rover/{image_raw,depth,camera_info}` | Image/CameraInfo | 64 (이전 step3 검증) |
| `/camera/wrist/{image_raw,depth,camera_info}` | Image/CameraInfo | 64 |

구독 2개 Subscription count = 1 정상:
- `/cmd_vel` (geometry_msgs/Twist)
- `/arm/joint_command` (sensor_msgs/JointState)

### 10.2 라이브 경로 (3 프로세스) 완성

```
vehicle_v3 ─/ground_truth/odom─▶ odom_to_estimated_pose ─/rover/estimated_pose─▶ coverage_node ─/cmd_vel─▶ vehicle_v3
```

3 터미널 구성:
- **A**: `temp/ros-isaac-python-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00004`
- **B**: `ros2 run isaac_drive odom_to_estimated_pose --ros-args -p odom_topic:=/ground_truth/odom` (parameter override 필요했음 — 11.1 참조)
- **C**: `ros2 run isaac_drive coverage_node`

### 10.3 잠시 헤맸던 부분 (학습 가치)

- 처음에 `temp/run_min_camera_usd.py vehicle_v3.usd` 로 띄웠더니 **terrain reference 안 됨** → vehicle 이 빈 공간에 spawn → 중력으로 떨어짐. 해결: 정식 런처 `run_vehicle_v3.py` 사용 (terrain 로드 포함).
- coverage_node 의 minimap viewer 가 `waiting…` 상태로 멈춤 → 진짜 원인은 11.1 의 odom_topic mismatch.

---

## 11. 동시 발견된 별개 이슈

### 11.1 `odom_to_estimated_pose` 의 default odom_topic mismatch (Fix 완료)

**증상**: `odom_to_estimated_pose` 의 default `odom_topic` 이 `/odom` 으로 되어 있었으나 vehicle_v3 표준은 `/ground_truth/odom`. → 어댑터가 데이터 못 받음 → `/rover/estimated_pose` 안 흐름 → coverage_node 가 pose 못 받음.

**Fix** (2026-05-24, 동일 세션):
- `isaac_drive/isaac_drive/odom_to_estimated_pose.py:25` 의 default 를 `/ground_truth/odom` 으로 변경
- 호환 위해 parameter override 로 옛 `/odom` 도 받게 유지 (`-p odom_topic:=/odom`)
- docstring 도 vehicle_v3 표준 명시로 갱신

```bash
# 영구 fix 적용 후 build (symlink-install 이면 source 만 수정으로 즉시 적용됨)
cd ~/dev_ws/rover_ws
colcon build --symlink-install --packages-select isaac_drive
```

---

## 12. 다음 세션 결정 사항

- [ ] 메인 환경을 PyPI binary 로 전환할지 (개인 alias 추가 + 팀 공지)
- [ ] source build clean rebuild 도 병행 시도할지
- [ ] 메모리 `reference_driver_580_159_scriptnode.md` 에 본 진단 결과 cross-reference 추가
- [ ] `temp/` 의 진단 도구 중 일부 (특히 `ros-isaac-python-pypi` wrapper) 를 정식 경로로 승격할지 (예: `scripts/` 또는 `tools/`)
- [ ] launch 파일 도입 검토: `ros2 launch isaac_drive live.launch.py` 한 줄로 odom_to_estimated_pose + coverage_node 동시 띄움 (parameter override 불필요)
