# isaac_sim

## 1. 모듈 역할

`isaac_sim`은 Isaac Sim 기반 화성 탐사 환경을 관리하는 모듈입니다.

이 모듈은 Isaac Sim 프로그램 자체를 저장하는 폴더가 아닙니다.  
프로젝트에서 사용할 **화성 지형, 로버, 로봇팔, 광석, 기지, 카메라, Depth 센서, LiDAR, Cargo 등 시뮬레이션 환경 구성 요소**를 관리하는 패키지입니다.

---

## 2. 예상 폴더 구조

```text
isaac_sim/
├─ scripts/
├─ worlds/
│  └─ mars_exploration_world.usd
├─ assets/
├─ package.xml
└─ setup.py
```

---

## 3. 주요 폴더 설명

### `worlds/`

Isaac Sim에서 사용할 USD 월드 파일을 저장하는 폴더입니다.

예상 파일은 다음과 같습니다.

```text
mars_exploration_world.usd
```

이 파일에는 다음 요소들이 포함될 수 있습니다.

```text
- 화성 지형
- 탐사 로버
- 로봇팔
- Cargo
- 광석 또는 자원 객체
- 기지 또는 하역 지점
- 카메라
- Depth 센서
- LiDAR
```

---

### `assets/`

시뮬레이션에 필요한 3D 에셋을 저장하는 폴더입니다.

예상 에셋은 다음과 같습니다.

```text
- rover
- robot_arm
- minerals
- base_station
- solar_panel
- mars_terrain
- cargo
```

초기 단계에서는 비워두고, 에셋이 확정되면 추가하면 됩니다.

---

### `scripts/`

Isaac Sim 환경을 Python으로 자동 구성하거나 테스트할 때 사용하는 스크립트를 넣는 폴더입니다.

이 폴더는 필수는 아닙니다.  
Isaac Sim에서 직접 월드를 구성하고 `.usd` 파일로 저장한다면 `scripts/`는 비워둬도 됩니다.

나중에 필요해질 수 있는 예시는 다음과 같습니다.

```text
load_world.py
spawn_rover.py
spawn_robot_arm.py
setup_camera.py
setup_lidar.py
spawn_minerals.py
spawn_base_station.py
reset_world.py
```

---

## 4. Isaac Sim과 ROS2의 관계

Isaac Sim은 가상환경과 센서 데이터를 생성합니다.  
ROS2 노드들은 그 데이터를 받아서 인식, 판단, 실행을 수행합니다.

흐름은 다음과 같습니다.

```text
Isaac Sim Camera / Depth / LiDAR
→ ROS2 Topic
→ perception_node
```

반대로 ROS2에서 생성된 제어 명령은 Isaac Sim 로버와 로봇팔로 전달됩니다.

```text
mobile_base_executor_node
→ /cmd_vel 또는 wheel command
→ Isaac Sim Rover

arm_executor_node
→ arm / gripper command
→ Isaac Sim Robot Arm
```

---

## 5. 이 모듈에 들어가면 안 되는 것

`isaac_sim`에는 아래 기능을 직접 구현하지 않습니다.

```text
- AI 인식 모델 코드
- 강화학습 정책 코드
- Mission Manager 로직
- Navigation Manager 로직
- Arm Executor 로직
```

이 모듈은 **시뮬레이션 환경 구성**에 집중합니다.

---

## 6. 설계 의도

`isaac_sim`을 별도로 분리하면 Isaac Sim 환경 담당자가 AI나 ROS2 제어 코드와 독립적으로 작업할 수 있습니다.

장점은 다음과 같습니다.

```text
- 시뮬레이션 환경 관리가 명확해짐
- 에셋과 월드 파일 위치가 정리됨
- AI/Navigation/Nodes 모듈과 역할 충돌 감소
```

---

## 7. 한 줄 요약

`isaac_sim`은 화성 탐사 로버 프로젝트의 Isaac Sim 월드와 에셋을 관리하는 시뮬레이션 환경 모듈입니다.
