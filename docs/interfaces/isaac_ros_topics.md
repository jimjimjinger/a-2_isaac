# vehicle_v3 ROS2 인터페이스

> 갱신: 2026-05-22 — **vehicle_v3** (액션그래프 내장 로봇) 기준.
> 이전 문서는 T5 의 `rover_m0609_localization.usd` 씬 기준이었으나, 현재 팀
> 표준 로봇은 `vehicle_v3.usd` 로 통일됨.

## vehicle_v3 = 표준 로봇

ROS2 인터페이스(센서·주행·팔)가 USD 에 내장된 "고정 로봇". terrain 에
reference·play 하면 **런타임 그래프 코드 없이** 토픽이 살아난다 — 실물
하드웨어처럼.

- `vehicle_v3.usd` 는 **자립(standalone) 파일** — 외형·물리·관절·Action Graph 를
  모두 자체 보유. 다른 USD 를 reference 하지 않는다 (flatten 으로 inline).
- `vehicle_v2.usd` 는 v3 빌드의 *입력 소스*일 뿐. v2 수정 시 `build_vehicle_v3.py`
  재실행 = v3 재bake.

| 항목 | 경로 (repo 루트 기준) |
|---|---|
| 로봇 (자립 USD, 그래프 내장) | `isaac_sim/assets/vehicle/vehicle_v3.usd` |
| 그래프 빌더 (bake 파이프라인) | `isaac_sim/scripts/build_vehicle_v3.py` |
| 단독 실행 런처 | `isaac_sim/scripts/run_vehicle_v3.py` |

실행:

```bash
<isaac-python> isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00004
```

## 토픽 표

### 발행 (로봇 → 외부 · 센서)

| 데이터 | 토픽 | 메시지 타입 | 소스 | 용도 |
|---|---|---|---|---|
| IMU | `/imu/data` | `sensor_msgs/Imu` | Body/Imu_Sensor | Localization / EKF |
| 전체 관절 상태 | `/joint_states_raw` | `sensor_msgs/JointState` | articulation (27 DOF) | 원시 로봇 상태 (splitter 입력) |
| 로버 카메라 RGB | `/camera/rover/image_raw` | `sensor_msgs/Image` | Body 카메라 | Vision / 미네랄 탐지 |
| 로버 카메라 depth | `/camera/rover/depth` | `sensor_msgs/Image` | Body 카메라 | 장애물 대응 |
| 로버 카메라 info | `/camera/rover/camera_info` | `sensor_msgs/CameraInfo` | Body 카메라 | deprojection / 카메라 모델 |
| 손목 카메라 RGB | `/camera/wrist/image_raw` | `sensor_msgs/Image` | D455 Color cam | manipulation vision (T2) |
| 손목 카메라 depth | `/camera/wrist/depth` | `sensor_msgs/Image` | D455 Pseudo_Depth | manipulation depth |
| 손목 카메라 info | `/camera/wrist/camera_info` | `sensor_msgs/CameraInfo` | D455 | 손목 depth deprojection |
| Sun 카메라 RGB | `/camera/sun/image_raw` | `sensor_msgs/Image` | Body 상단 SunCamera (320×240, +z 향함) | T5 sun_yaw 노드 입력 — 절대 방위 추정 |
| Sun 카메라 info | `/camera/sun/camera_info` | `sensor_msgs/CameraInfo` | SunCamera | sun_yaw deprojection / 카메라 모델 |
| **GT odometry (dev cheat)** | `/ground_truth/odom` | `nav_msgs/Odometry` | ScriptNode `ReadGtPose` → `ROS2PublishOdometry` | T5 localization 완성 전까지 perfect pose 대용 — frame_id=`world`, child=`base_link`. **졸업 시 PubGtOdom + ReadGtPose 두 노드 제거 → 재bake 로 cheat 제거** |

### 구독 (외부 → 로봇 · 제어)

| 데이터 | 토픽 | 메시지 타입 | 발행자 | 동작 |
|---|---|---|---|---|
| 주행 명령 | `/cmd_vel` | `geometry_msgs/Twist` | `coverage_node` 등 | 내장 Ackermann → 6 구동휠 + 4 조향휠 |
| 팔 명령 | `/arm/joint_command` | `sensor_msgs/JointState` | `arm_executor_node` | m0609 6축 + 그리퍼 관절 위치 (계약 **I11**) |

## 의도적 제외 (현재)

| 토픽 | 사유 |
|---|---|
| `/odom` | Isaac 생성 odom 은 실세계 인터페이스로 부적합 (실세계는 wheel encoder 적분이 정직한 path). 현재 coverage 는 v3 가 발행하는 `/ground_truth/odom` (dev cheat) 을 `odom_to_estimated_pose` 어댑터로 받아 사용. 추후 정직한 wheel odometry 는 `/rover/wheel_states` 에서 유도 예정. |
| `/tf`, `/tf_static` | 현 마일스톤 범위 밖. pick-place 단계에서 추가 예정(남음). |
| `/clock` | 현 워크플로에 불필요. |

## 내장 Action Graph

v3 의 그래프는 `/Root/ActionGraph` 하나 (terrain 에 reference 되면
`/World/Rover/ActionGraph` 로 자동 remap). 센서·주행·팔 노드가 한 그래프에:

| 그룹 | 노드 구성 | 토픽 |
|---|---|---|
| 센서 (몸체) | `IsaacReadIMU`→`ROS2PublishImu` / `ROS2PublishJointState` / `IsaacCreateRenderProduct`×4 + `ROS2Camera(Info)Helper`×8 | `/imu/data` `/joint_states_raw` `/camera/rover/*` `/camera/wrist/*` `/camera/sun/*` 발행 |
| 주행 | `ROS2SubscribeTwist` → `ScriptNode`(6륜 Ackermann) → `IsaacArticulationController`×2 | `/cmd_vel` 구독 |
| 팔 | `ROS2SubscribeJointState` → `IsaacArticulationController` | `/arm/joint_command` 구독 |
| GT pose (dev cheat) | `ScriptNode`(`ReadGtPose`, stage traverse 로 GT 추출) → `ROS2PublishOdometry` | `/ground_truth/odom` 발행 — T5 졸업 시 두 노드 제거 |

그래프 정의의 단일 소스 = `build_vehicle_v3.py` (코드). USD 는 그 산출물(bake).

## joint_state_splitter (별도 ROS2 노드)

v3 가 발행하는 `/joint_states_raw` 는 전체 관절 원시 상태. `isaac_localization`
패키지의 `joint_state_splitter_node` 가 이를 둘로 분리:

```
/joint_states_raw  ──splitter──▶  /rover/wheel_states   (휠 10)
                                  /joint_states         (팔/그리퍼 12)
```

실행:

```bash
ros2 run isaac_localization joint_state_splitter_node
```

- `/rover/wheel_states`: `FL/FR/CL/CR/RL/RR_Drive_Continuous` + `FL/FR/RL/RR_Steer_Revolute` — wheel odometry / localization 용
- `/joint_states`: `joint_1`~`joint_6` + `finger_joint` + knuckle/finger mimic 관절 — M0609 + 그리퍼 상태

## 데이터 흐름

```
주행:  coverage_node ─/cmd_vel─▶ vehicle_v3 (내장 Ackermann) ─▶ 휠 관절
팔:    arm_executor  ─/arm/joint_command─▶ vehicle_v3 ─▶ m0609 관절
센서:  vehicle_v3 ─▶ /imu/data · /camera/* · /joint_states_raw
                                          └▶ joint_state_splitter ─▶ /rover/wheel_states · /joint_states
```

## 관련 문서

- [`INTERFACE_CONTRACTS.md`](INTERFACE_CONTRACTS.md) — I11 `/arm/joint_command` 정식 계약, 미션 인터페이스 I1~I5
