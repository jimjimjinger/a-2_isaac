# tools/

프로젝트 전반 환경 launcher / 도구 모음. ROS2 패키지 (colcon build 대상) 아님.

## isaac-pypi

Isaac Sim 5.1 **PyPI binary** + ROS2 humble 환경 launcher.

```bash
tools/isaac-pypi <python-script> [args...]
```

예:
```bash
tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00004
```

### 왜 PyPI binary 인가
source build Isaac Sim 의 syntheticdata/replicator 통합이 incomplete 한 케이스가 있어 (사용자 T4 환경 검증, 2026-05-24), 카메라 토픽/GT odom 이 dtype 에러로 발행 안 되는 회귀가 발생했습니다. PyPI binary 는 NVIDIA 가 한 set 로 정합성 보장된 채 배포되므로 같은 USD 가 정상 동작합니다.

상세 진단: [`docs/troubleshooting/2026-05-24_isaac_camera_topic_regression.md`](../docs/troubleshooting/2026-05-24_isaac_camera_topic_regression.md)

### 셋업
처음 사용 전 PyPI Isaac Sim 5.1 설치 + 환경변수 설정 필요:

→ **[`docs/SETUP_ISAAC_PYPI.md`](../docs/SETUP_ISAAC_PYPI.md)** 참조

### 환경변수 (override)
| 변수 | default | 의미 |
|---|---|---|
| `ISAAC_PYPI_VENV` | `~/dev_ws/isaac_sim_pypi/venv` | PyPI venv 경로 |
| `ISAAC_ROS2_WS` | `~/dev_ws/isaac_sim/IsaacSim-ros_workspaces/humble_ws` | humble_ws 경로 (이미 `.bashrc` export 되어 있을 가능성) |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | RMW 구현체 |

다른 경로에 설치한 팀원은 `.bashrc` 에 `export ISAAC_PYPI_VENV=...` 추가하면 wrapper 가 자동으로 그 경로 사용.
