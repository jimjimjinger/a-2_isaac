# Isaac Sim 기반 화성 탐사 로버 자원 채취 시스템

Isaac Sim 기반 화성 탐사 로버 프로젝트의 ROS2 workspace skeleton입니다.

## Workspace

```text
src/
├─ isaac_bringup/
├─ isaac_sim/
├─ isaac_ai/
├─ isaac_navigation/
├─ isaac_nodes/
└─ isaac_interfaces/
```

## Modules

- `isaac_bringup`: 전체 시스템 launch 실행 관리
- `isaac_sim`: 화성 지형, 로버, 로봇팔, cargo, 광석, 기지, 센서 환경 구성
- `isaac_ai`: 광석 인식, 지형 인식, 강화학습 기반 주행 판단
- `isaac_navigation`: 자율주행, 경로 계획, 장애물 회피, 맵 관리, 수동 조종, 주행 실행
- `isaac_nodes`: 전체 미션 관리, 배터리 감시, 로봇팔 실행
- `isaac_interfaces`: ROS2 노드 간 msg/srv/action 통신 규격

## Build

```bash
cd ~/dev_ws/isaac_ws
colcon build --symlink-install
```

## Run

```bash
ros2 launch isaac_bringup full_system.launch.py
```
