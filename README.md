# a-2_isaac

Isaac Sim 기반 객체 인식, 3D 위치 추정, 강화학습 정책 선택, 작업 관리, 로봇 실행 흐름을 구성하는 ROS2 패키지 모음입니다.

## Packages

- `isaac_launch`: 전체 파이프라인 launch/config
- `isaac_sim`: Isaac Sim 환경, 월드, USD, 시뮬레이션 보조 스크립트
- `isaac_ai`: `vision_ai_node`, `object_pose_node`, `rl_policy_node`
- `isaac_nodes`: `state_collector_node`, `task_manager_node`, `robot_executor_node`, logger, motion primitives
- `isaac_interfaces`: 커스텀 msg/srv/action 통신 규격

## Build

```bash
cd ~/dev_ws/isaac_sim
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch isaac_launch pipeline.launch.py
```
