# a-2_isaac

Isaac Sim 기반 화성 탐사 로버 프로젝트의 ROS2 workspace skeleton입니다.

## Workspace

```text
isaac_ws/
└─ src/
   ├─ isaac_bringup/
   ├─ isaac_sim/
   ├─ isaac_ai/
   ├─ isaac_nodes/
   └─ isaac_interfaces/
```

## Build

```bash
cd isaac_ws
colcon build --symlink-install
```

## Run

```bash
ros2 launch isaac_bringup full_system.launch.py
```
