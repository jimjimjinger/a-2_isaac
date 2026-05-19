# Isaac Sim 기반 화성 탐사 로버 자원 채취 시스템

Isaac Sim 기반 화성 탐사 로버 자원 채취 시스템의 ROS2 workspace 모듈 구조입니다.

> ⚠️ **이 브랜치는 패키지 재구성 제안**입니다 (`restructure/package-clarity`).
> 명칭 명료성과 책임 분리를 위해 6개 → 9개 패키지로 재편성. 자세한 설계 의도는 [docs/STUDY_AND_PLAN.md](docs/STUDY_AND_PLAN.md) Part XI 참조.

## Workspace

```text
~/dev_ws/isaac_sim/src/
├─ isaac_bringup/        # 시스템 launch 모음 (진입점)
├─ isaac_sim/            # Isaac Sim 환경 + 월드 + bridge
├─ isaac_perception/     # 인지 (vision/depth/lidar)
├─ isaac_rl/             # 강화학습 정책 + 학습 환경
├─ isaac_drive/          # 주행 전체 (manager + executor + navigation + primitives)
├─ isaac_supervisor/     # 미션 감독 (mission_manager + battery_monitor)
├─ isaac_manipulation/   # M0609 매니퓰레이터 + primitives
├─ isaac_localization/   # 위치 추정 (TRN + EKF + sensor fusion)
└─ isaac_interfaces/     # ROS2 통신 규격 (msg/srv/action)
```

## Modules

| 패키지 | 책임 | 주요 노드 |
|--------|------|----------|
| `isaac_bringup` | 시스템 launch 통합 관리 | 8개 launch 파일 |
| `isaac_sim` | Isaac Sim 환경 + ROS2 bridge | `sim_bridge_node` |
| `isaac_perception` | 광석 인식, 장애물·지형 판단 (Vision + Depth + LiDAR 통합 가능) | `perception_node` |
| `isaac_rl` | 강화학습 기반 주행 행동 선택 (PPO 정책) | `driving_policy_node`, `rl_trainer` |
| `isaac_drive` | 자율/수동 주행 흐름 + 경로 계획 + 휠 명령 실행 | `drive_manager_node`, `mobile_base_executor_node` |
| `isaac_supervisor` | 전체 미션 흐름 감독 + 배터리 모니터링 | `mission_manager_node`, `battery_monitor_node` |
| `isaac_manipulation` | M0609 로봇팔 + pick/place/unload/deploy_solar | `arm_executor_node` |
| `isaac_localization` | GPS-less 위치 추정 (TRN + EKF) | `localization_node` |
| `isaac_interfaces` | ROS2 노드 간 통신 규격 | msg/srv/action |

## Patch from previous structure

| 이전 | 변경 후 | 사유 |
|------|--------|------|
| `isaac_ai` | `isaac_perception` + `isaac_rl` | vision과 RL은 다른 책임, 다른 owner |
| `isaac_navigation` | `isaac_drive` | 실제 콘텐츠 = "주행 실행" (navigation은 내부 서브) |
| `isaac_nodes` | `isaac_supervisor` + `isaac_manipulation` | "nodes" 모호, 책임별 분리 |
| (없음) | `isaac_localization` 신규 | GPS-less Mars TRN 영역 |

## Build

```bash
cd ~/dev_ws/isaac_sim
colcon build --symlink-install
```

## Run

```bash
ros2 launch isaac_bringup full_system.launch.py
```

또는 부분 실행:

```bash
ros2 launch isaac_bringup perception.launch.py     # 인지만
ros2 launch isaac_bringup drive.launch.py          # 주행만
ros2 launch isaac_bringup localization.launch.py   # 위치 추정만
ros2 launch isaac_bringup manipulation.launch.py   # M0609만
ros2 launch isaac_bringup supervisor.launch.py     # 미션 감독만
ros2 launch isaac_bringup rl.launch.py             # RL 정책만
ros2 launch isaac_bringup sim.launch.py            # Isaac Sim 브릿지만
```

## Architecture

전체 시스템 아키텍처는 [docs/flowcharts/system_architecture_full.svg](docs/flowcharts/system_architecture_full.svg) 참조.

미션 시나리오는 [docs/flowcharts/project_overview_flowchart.svg](docs/flowcharts/project_overview_flowchart.svg) 참조.
