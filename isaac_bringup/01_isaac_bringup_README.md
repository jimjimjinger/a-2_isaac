# isaac_bringup

## 1. 모듈 역할

`isaac_bringup`은 전체 ROS2 시스템을 실행하기 위한 launch 관리 모듈입니다.

이 모듈에는 실제 로봇 제어, AI 추론, 주행 판단, 로봇팔 동작 구현 코드를 넣지 않습니다.  
각 패키지에 흩어져 있는 노드들을 한 번에 실행하거나, 기능별로 나누어 실행하기 위한 launch 파일을 관리합니다.

즉, `isaac_bringup`은 시스템의 **실행 진입점** 역할을 합니다.

---

## 2. 예상 폴더 구조

```text
isaac_bringup/
├─ launch/
│  ├─ full_system.launch.py
│  ├─ sim.launch.py
│  ├─ ai.launch.py
│  ├─ navigation.launch.py
│  └─ nodes.launch.py
├─ package.xml
└─ setup.py
```

---

## 3. 주요 파일 설명

### `full_system.launch.py`

전체 시스템을 한 번에 실행하는 최종 launch 파일입니다.

실제 프로젝트에서는 다음과 같은 launch 파일들을 내부에서 함께 실행하도록 구성할 수 있습니다.

```text
sim.launch.py
ai.launch.py
navigation.launch.py
nodes.launch.py
```

최종 실행 명령 예시는 다음과 같습니다.

```bash
ros2 launch isaac_bringup full_system.launch.py
```

---

### `sim.launch.py`

Isaac Sim 연동 또는 시뮬레이션 관련 실행을 관리하는 launch 파일입니다.

주요 목적은 다음과 같습니다.

```text
- Isaac Sim 관련 설정 실행
- ROS2 Bridge 연동 확인
- 시뮬레이션 환경과 ROS2 노드 연결 준비
```

단, Isaac Sim 프로그램 자체를 반드시 이 launch 파일에서 실행해야 하는 것은 아닙니다.  
초기 단계에서는 Isaac Sim을 직접 실행하고, ROS2 노드만 launch로 관리해도 됩니다.

---

### `ai.launch.py`

AI 관련 노드를 실행하는 launch 파일입니다.

실행 대상 예시는 다음과 같습니다.

```text
perception_node
driving_policy_node
```

이 launch 파일은 카메라/Depth 데이터를 분석하는 AI 노드와 강화학습 기반 주행 판단 노드를 실행하는 역할을 합니다.

---

### `navigation.launch.py`

주행 관련 노드를 실행하는 launch 파일입니다.

실행 대상 예시는 다음과 같습니다.

```text
navigation_manager_node
mobile_base_executor_node
```

`navigation_manager_node`는 주행 흐름을 관리하고, `mobile_base_executor_node`는 실제 로버 주행 명령을 실행합니다.

---

### `nodes.launch.py`

미션 관리, 배터리 감시, 로봇팔 실행 관련 노드를 실행하는 launch 파일입니다.

실행 대상 예시는 다음과 같습니다.

```text
mission_manager_node
battery_monitor_node
arm_executor_node
```

---

## 4. 이 모듈에 들어가면 안 되는 것

`isaac_bringup`에는 아래 기능을 직접 구현하지 않습니다.

```text
- 광석 인식 AI 코드
- 강화학습 정책 코드
- 로버 주행 제어 코드
- 로봇팔 pick & place 코드
- Isaac Sim 월드 구성 파일
```

이 모듈은 오직 **실행 구성**을 담당합니다.

---

## 5. 설계 의도

프로젝트가 커지면 노드가 많아집니다.  
각 노드를 터미널에서 하나씩 실행하면 실수가 많아지고, 실행 순서 관리도 어렵습니다.

따라서 `isaac_bringup`에서 launch 파일을 관리하면 다음 장점이 있습니다.

```text
- 전체 시스템을 한 번에 실행 가능
- 기능별 실행 파일 분리 가능
- 팀원 간 실행 방식 통일
- 발표 시 재현성 향상
```

---

## 6. 한 줄 요약

`isaac_bringup`은 전체 ROS2 노드를 실행하기 위한 launch 관리 모듈입니다.
