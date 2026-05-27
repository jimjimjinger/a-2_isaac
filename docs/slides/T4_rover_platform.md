# T4 — 로버 플랫폼 상세 설계

> 발표자: 성선규 (T4)
> 주제: A2 ISAAC 미션의 로버 플랫폼 — 발전 과정과 최종 통합 (`vehicle_v3.usd`)

---

## 1. 한 줄 요약

**“팀별로 제각각이던 rover 정의를, 액션그래프까지 USD 안에 구워넣은 단일 표준 플랫폼으로 통합했다.”**

terrain 에 reference + play 만 하면 그 자체로 ROS2 토픽을 발행·구독하는 **자립 (standalone) 로봇**.

---

## 2. 발전 과정 — 5단계 진화

```
[1] AAU Mars Rover                     ← 외부 베이스 (단순 차체)
        │  build_integrated_vehicle.py
        │  + Doosan M0609 6-DOF 팔
        │  + OnRobot RG2-FT 그리퍼
        ▼
[2] vehicle_origin_T2.usd              ← T2(최진우) 원본 통합본
        │  T3(이찬휘) coverage 검증
        │  + 후방 바스켓 (visual-only)
        ▼
[3] vehicle_v1.usd                     ← 첫 통합 vehicle
        │  build_vehicle_v2.py
        │  + 밸러스트 추가 + 외형 정리
        ▼
[4] vehicle_v2.usd                     ← 정적 USD (그래프 없음)
        │  build_vehicle_v3.py
        │  + 센서/주행/팔/GT odom 액션그래프 bake
        │  + flatten (v2 미참조)
        ▼
[5] vehicle_v3.usd       ★ 현재 시연 ★  ← 액션그래프 내장 자립 USD
```

### 단계별 핵심 결정

| 단계 | 자산 | 핵심 변화 | 이유 |
|---|---|---|---|
| [1]→[2] | origin_T2 | AAU 차체 + M0609 팔 + RG2 그리퍼 결합 | manipulation 통합 |
| [2]→[3] | v1 | T3 coverage 검증 + 후방 바스켓 | 채집물 적재 공간 |
| [3]→[4] | v2 | 밸러스트 추가 | 6-DOF 팔 swing 시 무게 중심 안정 |
| [4]→[5] | **v3** | 액션그래프 USD 안에 bake + flatten | **런타임 그래프 빌드 코드 제거 — 실물 하드웨어처럼 “플러그 앤 플레이”** |

### 폐기된 코드 (회귀 진단용으로만 보존)

`isaac_sim/assets/vehicle/legacy/` 에 v1/v2/v2_scene/origin_T2 보존. 시연 main path 에는 사용 안 함.

---

## 3. 최종 플랫폼 — `vehicle_v3.usd` 구성도

```
/Root  (defaultPrim, terrain 에 reference 시 /World/Rover 로 remap)
└── Vehicle
    ├── rover                                  ── AAU Mars Rover 베이스
    │   └── Body
    │       ├── Imu_Sensor                     ── /imu/data
    │       ├── Camera                         ── /camera/rover/* (Body cam)
    │       └── SunCamera                      ── /camera/sun/*   (T5 sun_yaw 용, invisible)
    │
    ├── m0609                                  ── Doosan M0609 6-DOF 팔
    │   └── base_link                          ── (articulation root)
    │
    └── onrobot_rg2ft                          ── OnRobot RG2-FT 그리퍼 (T2 의도 존중)
        └── angle_bracket
            └── realsense_d455
                └── RSD455                     ── Intel RealSense D455 (wrist mount)
                    ├── Camera_OmniVision_OV9782_Color    ── /camera/wrist/image_raw
                    └── Camera_Pseudo_Depth               ── /camera/wrist/depth

ActionGraph  (이 그래프 자체가 USD 안에 baked — 런타임 코드 X)
  · 센서 발행: IMU / Joint / Body cam / Wrist RGB+Depth / Sun cam
  · 주행: /cmd_vel → ScriptNode (6-wheel Ackermann) → Steer/Drive Articulation
  · 팔: /arm/joint_command → m0609 Articulation
  · GT cheat: ScriptNode → /ground_truth/odom (졸업 시 이 두 노드만 제거)
  · Grasp: /grasp/command (Twist hijack) → FixedJoint snap + invisible
```

---

## 4. 부분별 USD 자산 일람

| 부위 | USD / 폴더 | 위치 | 역할 |
|---|---|---|---|
| **메인 통합 차량** | `vehicle_v3.usd` | `isaac_sim/assets/vehicle/` | 모든 센서·그래프 내장한 standalone 로봇 |
| **차체 (베이스)** | `Mars_Rover.usd` | `isaac_sim/assets/rover/` | AAU Mars Rover (6륜 + 본체) |
| 차체 머티리얼 | `SubUSDs/materials/*.mdl` | `isaac_sim/assets/rover/SubUSDs/` | Rubber_Textured / Cast_Metal_Silver_Vein / Plastic_ABS |
| **RGBD 카메라** | `rsd455.usd` | `isaac_sim/assets/d455/` | Intel RealSense D455 (wrist mount) |
| D455 머티리얼 | `materials/*.mdl` | `isaac_sim/assets/d455/materials/` | Aluminum_Anodized / Aluminum_Cast / Plastic_ABS |
| **그리퍼** | `onrobot_rg2/{urdf,meshes}` | `isaac_sim/assets/onrobot_rg2/` | OnRobot RG2-FT (F/T 센서 활용 여지) |
| **팔** | Doosan M0609 | (v2 에 inline 된 후 v3 로 flatten) | 6-DOF manipulator |
| **후방 바스켓** | (vehicle 본체에 내장) | — | 채집물 적재 (visual-only) |

> ⚠️ **머티리얼 path 락**: `vehicle_v3.usd` 가 `rover/SubUSDs/materials/*.mdl` 과 `d455/materials/*.mdl` 을 **절대경로로 참조** (build_vehicle_v3.py 가 v2 를 flatten 할 때 빌드 환경의 절대경로가 그대로 baked). 위 폴더들 이동 시 머티리얼 로드 실패 → rover 가 unshaded 빨간색. 이동 금지. (2026-05-27 사후 patch 시도 → 회귀 확인. 정공법은 build_vehicle_v3.py 의 입력 reference 를 상대경로로 만들어 재빌드.)

---

## 5. ROS2 인터페이스 총망라

### 5.1 발행 (Publish) — 11 토픽

| 토픽 | 타입 | 주기 | 발행 노드 (그래프 내) | 용도 |
|---|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | tick | `PubImu` | EKF (T5 localization) |
| `/joint_states_raw` | `sensor_msgs/JointState` | tick | `PubJoint` | 휠·팔 관절 측정값 |
| `/camera/rover/image_raw` | `sensor_msgs/Image` (RGB 640×480) | tick | `CamRoverRgb` | 본체 카메라 (YOLO 입력) |
| `/camera/rover/depth` | `sensor_msgs/Image` (depth) | tick | `CamRoverDepth` | 본체 깊이 |
| `/camera/rover/camera_info` | `sensor_msgs/CameraInfo` | tick | `CamRoverInfo` | 본체 intrinsic |
| `/camera/wrist/image_raw` | `sensor_msgs/Image` (RGB 640×480) | tick | `CamWristRgb` | 손목 RGB (D455, manipulation 정밀) |
| `/camera/wrist/depth` | `sensor_msgs/Image` (depth) | tick | `CamWristDepth` | 손목 깊이 |
| `/camera/wrist/camera_info` | `sensor_msgs/CameraInfo` | tick | `CamWristInfo` | 손목 intrinsic |
| `/camera/sun/image_raw` | `sensor_msgs/Image` (RGB 320×240) | tick | `CamSunRgb` | 태양 위치 (T5 sun_yaw 입력) |
| `/camera/sun/camera_info` | `sensor_msgs/CameraInfo` | tick | `CamSunInfo` | sun cam intrinsic |
| `/ground_truth/odom` | `nav_msgs/Odometry` | tick | `PubGtOdom` | **GT cheat** — 졸업 시 제거 |

### 5.2 구독 (Subscribe) — 3 토픽

| 토픽 | 타입 | 발행자 | 그래프 내 처리 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | T5 navigation / coverage | `SubTwist` → ScriptNode (6-wheel Ackermann) → Steer/Drive Articulation |
| `/arm/joint_command` | `sensor_msgs/JointState` | `arm_executor_node` (T2/T4) | `SubJointCmd` → m0609 Articulation (6 관절 위치 지령) |
| `/grasp/command` | `geometry_msgs/Twist` (hijack) | `arm_executor_node` | `SubGrasp` → ScriptNode → FixedJoint snap (pickup) / MakeInvisible (release) |

### 5.3 인터페이스 계약 (INTERFACE_CONTRACTS.md 등재)

| ID | 토픽/자산 | 비고 |
|---|---|---|
| I1 | terrain asset (T1 → 전체) | 시뮬 시작 전 파일 |
| I2 | `/perception/detections` (T2 → T3/T4) | YOLO 결과 |
| I3 | `/mission/pick_request` (T3 → T2) | event |
| I4 | `/mission/pick_response` (T2 → T3) | event |
| I5 | `/rover/estimated_pose` (T5 → T3/T4) | 30 Hz |
| **I11** | **`/arm/joint_command`** | **vehicle_v3 도입에 따른 신규 — 저수준 팔 제어** |

> v3 의 그래프-내장 아키텍처가 I11(저수준 팔 제어)을 표준 인터페이스로 끌어올렸음. 기존 “arm 관절을 직접 Articulation API 로 명령” → “ROS2 토픽 publish 만으로 제어”.

---

## 6. 차별점 — 왜 v3 아키텍처인가

| 비교 | 기존 (v1/v2) | v3 |
|---|---|---|
| 그래프 빌드 | 런타임 Python 스크립트 (`run_vehicle_v3.py` 초기 버전) | **USD 안에 baked** |
| 센서 발행 | 런타임 wiring | **그래프 안에 baked** |
| 주행 제어 | 외부 ScriptNode 동적 생성 | **그래프 안 ScriptNode + Ackermann inline** |
| 팔 제어 | Articulation API 직접 호출 | **ROS2 `/arm/joint_command` 표준** |
| 멀티 rover 지원 | 토픽 namespace 수동 prep | run script 가 ScriptNode `topicName` 만 patch (`/rover_1/...`) |
| 새 terrain 적용 | terrain 별 별도 wiring | **terrain 에 reference 만 하면 끝** |

---

## 7. 핵심 talking point (1분 요약용)

1. **단일 표준 플랫폼** — 4개 팀(T1~T5)이 같은 `vehicle_v3.usd` 위에서 작업
2. **자립 standalone USD** — terrain 에 reference + play 만 하면 ROS2 인터페이스 자동
3. **그래프-내장 아키텍처** — 런타임 코드 의존성 제거, 실물 하드웨어 대체 가능
4. **멀티 rover ready** — namespace patching 으로 2대 동시 운용 검증 완료
5. **GT cheat 분리** — `/ground_truth/odom` 발행 노드 2개만 제거하면 졸업

---

## 8. 부록 — 빌드 & 실행

### 빌드 (USD 수정 시만)
```bash
tools/isaac-pypi isaac_sim/scripts/build_vehicle_v3.py
```
v2 위에 액션그래프 + 센서 + 팔 그래프 bake + flatten → v3 standalone USD 생성.

### 시연 (단일 rover)
```bash
tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00026 --no-overview
```

### 시연 (멀티 rover, 2대)
```bash
tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00026 --rovers rover_1 rover_2
```

전체 시연 명령어 모음: [`docs/DEMO_COMMANDS.md`](../DEMO_COMMANDS.md)

---

## 9. 참고 자료

- 자산 README: [`isaac_sim/assets/vehicle/README.md`](../../isaac_sim/assets/vehicle/README.md)
- 인터페이스 계약: [`docs/interfaces/INTERFACE_CONTRACTS.md`](../interfaces/INTERFACE_CONTRACTS.md)
- 빌드 스크립트: [`isaac_sim/scripts/build_vehicle_v3.py`](../../isaac_sim/scripts/build_vehicle_v3.py)
- 런타임 launcher: [`isaac_sim/scripts/run_vehicle_v3.py`](../../isaac_sim/scripts/run_vehicle_v3.py)
