# 🚀 Isaac Sim PyPI binary 환경 셋업

> 팀 표준 결정 (2026-05-24, T4): Isaac Sim 5.1 은 **PyPI binary release** 사용.
>
> 이유: 사용자(T4) 환경의 source build 가 syntheticdata/replicator incomplete 상태로 카메라/GT odom 토픽 회귀 발생. PyPI binary 에선 같은 USD 가 정상 동작 확정. 상세 진단: [`troubleshooting/2026-05-24_isaac_camera_topic_regression.md`](troubleshooting/2026-05-24_isaac_camera_topic_regression.md)

main 을 pull 한 팀원이 따라 해야 할 1회성 셋업입니다. 이미 source build 만 쓰던 분들도 **이걸 깔고 같이 사용**하는 게 안전합니다 (둘이 같은 PC 에 공존 가능, 디스크 ~15GB).

---

## 0. 사전 요구

- Ubuntu 22.04
- Python 3.11 (Ubuntu 22.04 의 기본은 3.10 이라 별도 설치)
- ROS2 humble 시스템 설치 (`/opt/ros/humble`) — 기존대로

## 1. Python 3.11 설치

```bash
sudo apt install -y python3.11 python3.11-venv
python3.11 --version    # Python 3.11.x 떠야 OK
```

`apt update` 가 실패하면 (`librealsense GPG key 등 무관한 repo 에러`) 그대로 `apt install` 만 시도 — Ubuntu universe 의 cache 에 이미 metadata 있어 잘 깔립니다.

## 2. PyPI Isaac Sim 5.1 설치

별도 venv 에 격리 설치 (system Python 안 건드림):

```bash
mkdir -p ~/dev_ws/isaac_sim_pypi
python3.11 -m venv ~/dev_ws/isaac_sim_pypi/venv
source ~/dev_ws/isaac_sim_pypi/venv/bin/activate
pip install --upgrade pip
pip install isaacsim[all,extscache]==5.1.0 --extra-index-url https://pypi.nvidia.com
```

수 GB 다운로드 — 수 분 ~ 수십 분 소요. 첫 실행 시 EULA 동의 프롬프트 뜨면 `Yes`.

### 다른 경로에 두고 싶다면
경로를 바꿔서 설치한 뒤 `.bashrc` 에 한 줄 추가:
```bash
export ISAAC_PYPI_VENV=/your/custom/path/venv
```
이 변수가 set 되면 [`tools/isaac-pypi`](../tools/isaac-pypi) 가 자동으로 그 경로 사용.

## 3. 설치 검증

```bash
source ~/dev_ws/isaac_sim_pypi/venv/bin/activate
pip list | grep isaacsim
python -c "import isaacsim; print('OK', isaacsim.__file__)"
```

`isaacsim 5.1.0.0` 등 25개 패키지 + `OK ...` 출력되면 정상.

## 4. 사용 방법

`tools/isaac-pypi` 가 자동으로 환경 setup + python 실행:

```bash
cd ~/dev_ws/rover_ws/src/a2_isaac
tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00004
```

내부적으로:
1. `humble_ws/local_setup.bash` source → `AMENT_PREFIX_PATH` 설정
2. PyPI 내장 humble libs 를 `LD_LIBRARY_PATH` 에 추가 → RMW 로딩
3. PyPI venv 의 python 으로 스크립트 실행

## 5. 라이브 경로 (전체 system 띄우기)

```bash
# 터미널 A — Isaac Sim (vehicle_v3 + terrain)
cd ~/dev_ws/rover_ws/src/a2_isaac
tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00004

# 터미널 B — odom 어댑터 (default 가 /ground_truth/odom 이라 추가 인자 불필요)
rover    # ~/.bashrc 의 alias (humble + rover_ws source)
ros2 run isaac_drive odom_to_estimated_pose

# 터미널 C — coverage
rover
ros2 run isaac_drive coverage_node
```

## 6. 기존 source build 어떻게 할까

- 그대로 둬도 OK. 디스크 공간 여유 있으면 공존.
- 팀 공유 alias `isaac-python` (source build python.sh) 는 그대로 유지 — 단순 stage 조작, URDF import 같은 카메라 무관 작업엔 source build 도 작동.
- **카메라 / replicator / syntheticdata 가 관여하는 모든 작업** → 반드시 `tools/isaac-pypi` 로.

## 7. FAQ

### Q1. `apt install python3.11` 가 "Unable to locate package" 로 실패
A. apt cache 가 비어있는 케이스. `sudo apt update` (librealsense 등 무관한 repo 에러는 무시 가능) 후 재시도.

### Q2. `tools/isaac-pypi` 실행 시 `✗ PyPI venv python not found`
A. ISAAC_PYPI_VENV 환경변수 또는 default 경로 (`~/dev_ws/isaac_sim_pypi/venv`) 에 venv 가 없음. 2단계 다시 진행.

### Q3. `tools/isaac-pypi` 실행 시 `✗ humble_ws setup.bash not found`
A. `ISAAC_ROS2_WS` 환경변수가 본인 PC 의 humble_ws 위치를 가리켜야 함. `.bashrc` 에:
```bash
export ISAAC_ROS2_WS=$HOME/dev_ws/isaac_sim/IsaacSim-ros_workspaces/humble_ws
```
(기존 [SETUP_BASHRC.md](SETUP_BASHRC.md) 에 이미 있을 가능성 큼.)

### Q4. 카메라 토픽이 안 떠요
A. 첫 번째로 확인:
1. `tools/isaac-pypi` 로 띄웠는지 (`isaac-python` alias 가 아니라)
2. Isaac Sim 로그에 `Unable to write from unknown dtype, kind=i, size=0` 가 있는지 — 있으면 source build 환경에서 띄운 것. PyPI 로 다시.
3. 토픽 list 출력 시점이 `=== timeline.play ===` 후 충분히 지난 뒤인지 (vehicle init ~10초 걸림)

### Q5. 우리 팀 코드 (isaac_drive, rover_ws) 도 PyPI venv 안에 깔아야 하나?
A. **아니요.** isaac_drive 같은 ROS2 패키지는 `colcon build --symlink-install` 로 system humble 에서 빌드/실행. PyPI venv 는 Isaac Sim (USD 띄우는 측) 만 담당. 두 환경은 ROS2 토픽으로만 연결되니 분리되어도 잘 작동.

---

## 관련 문서

- [`troubleshooting/2026-05-24_isaac_camera_topic_regression.md`](troubleshooting/2026-05-24_isaac_camera_topic_regression.md) — 카메라 회귀 진단 (왜 PyPI 가 필요한가)
- [`SETUP_BASHRC.md`](SETUP_BASHRC.md) — `.bashrc` 기본 셋업 (이 가이드의 전제)
- [`interfaces/isaac_ros_topics.md`](interfaces/isaac_ros_topics.md) — vehicle_v3 의 ROS2 토픽 인터페이스
