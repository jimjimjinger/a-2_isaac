# Isaac Sim 기반 화성 탐사 로버 자원 채취 시스템

Isaac Sim 기반 화성 탐사 로버 자원 채취 시스템의 ROS2 workspace 모듈 구조입니다.

## Workspace

```text
~/dev_ws/isaac_sim/src/
├─ isaac_bringup/
├─ isaac_sim/
├─ isaac_ai/
├─ isaac_navigation/
├─ isaac_nodes/
└─ isaac_interfaces/
```

## Modules

- `isaac_bringup`: 전체 시스템 launch 실행 관리
- `isaac_sim`: Isaac Sim 화성 탐사 환경 구성
- `isaac_ai`: 광석 인식, 지형/장애물 판단, 강화학습 기반 주행 행동 선택
- `isaac_navigation`: 자율주행 흐름 관리, 경로 판단, 장애물 회피, 수동 조종, 맵 관리, 주행 실행
- `isaac_nodes`: 전체 미션 관리, 배터리 감시, 로봇팔 실행
- `isaac_interfaces`: ROS2 노드 간 통신 규격 정의 예정 폴더

## Build

```bash
cd ~/dev_ws/isaac_sim
colcon build --symlink-install
```

## Run

```bash
ros2 launch isaac_bringup full_system.launch.py
```
