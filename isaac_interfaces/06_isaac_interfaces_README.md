# isaac_interfaces

## 1. 모듈 역할

`isaac_interfaces`는 ROS2 노드 간 통신 규격을 정의하기 위한 패키지입니다.

이 모듈은 실행 노드가 아닙니다.  
각 노드들이 주고받을 메시지, 서비스, 액션 형식을 정의하는 역할을 합니다.

---

## 2. 현재 상태

현재 단계에서는 폴더만 생성해두고, 세부 인터페이스 파일은 나중에 추가합니다.

예상 구조는 다음과 같습니다.

```text
isaac_interfaces/
├─ msg/
├─ srv/
├─ action/
├─ package.xml
└─ CMakeLists.txt
```

---

## 3. 인터페이스가 필요한 이유

ROS2 노드들은 Topic, Service, Action을 통해 데이터를 주고받습니다.

예를 들어 `perception_node`가 광석 위치를 계산했다면, 그 결과를 `mission_manager_node`나 `navigation_manager_node`가 이해할 수 있는 형식으로 전달해야 합니다.

이때 필요한 것이 메시지 정의입니다.

---

## 4. 예상 msg 파일

나중에 추가할 수 있는 메시지 파일은 다음과 같습니다.

```text
DetectedObject.msg
ObjectPose.msg
TerrainState.msg
RoverStatus.msg
BatteryState.msg
MissionState.msg
RLState.msg
SelectedAction.msg
```

각 역할은 다음과 같습니다.

### `DetectedObject.msg`

광석 또는 객체 인식 결과를 정의합니다.

### `ObjectPose.msg`

광석의 3D 위치 정보를 정의합니다.

### `TerrainState.msg`

지형 경사, 장애물 여부, 주행 가능 여부를 정의합니다.

### `RoverStatus.msg`

로버 위치, 속도, 자세, 주행 상태를 정의합니다.

### `BatteryState.msg`

배터리 잔량, 충전 상태, 부족 여부를 정의합니다.

### `MissionState.msg`

현재 미션 단계, 모드, 성공/실패 상태를 정의합니다.

### `RLState.msg`

강화학습 정책 입력으로 사용할 통합 상태값을 정의합니다.

### `SelectedAction.msg`

강화학습 정책이 선택한 다음 주행 행동을 정의합니다.

---

## 5. 예상 srv 파일

나중에 추가할 수 있는 서비스 파일은 다음과 같습니다.

```text
ResetSimulation.srv
CheckSystemReady.srv
SaveExplorationMap.srv
```

### `ResetSimulation.srv`

시뮬레이션 초기화 요청과 응답을 정의합니다.

### `CheckSystemReady.srv`

시스템 준비 상태 확인 요청과 응답을 정의합니다.

### `SaveExplorationMap.srv`

탐사 맵 저장 요청과 응답을 정의합니다.

---

## 6. 예상 action 파일

나중에 추가할 수 있는 액션 파일은 다음과 같습니다.

```text
NavigateTask.action
ExecuteDriveTask.action
ExecuteArmTask.action
```

### `NavigateTask.action`

mission_manager_node가 navigation_manager_node에 보내는 고수준 주행 요청을 정의합니다.

### `ExecuteDriveTask.action`

navigation_manager_node가 mobile_base_executor_node에 보내는 실제 주행 실행 요청을 정의합니다.

### `ExecuteArmTask.action`

광석 집기, cargo 적재, 기지 하역, 태양광판 전개 같은 시간이 걸리는 로봇팔 작업 실행을 정의합니다.

---

## 7. Topic, Service, Action 구분

```text
Topic
= 계속 갱신되는 센서/상태/AI 결과 전달

Service
= 짧은 요청-응답

Action
= 시간이 걸리는 작업 실행
```

예시는 다음과 같습니다.

```text
Topic:
perception_node → mission_manager_node
battery_monitor_node → mission_manager_node

Action:
mission_manager_node → navigation_manager_node
navigation_manager_node → mobile_base_executor_node
mission_manager_node → arm_executor_node

Service:
mission_manager_node → reset_simulation
mission_manager_node → check_system_ready
```

---

## 8. 한 줄 요약

`isaac_interfaces`는 프로젝트 노드들이 안정적으로 통신할 수 있도록 msg, srv, action 형식을 정의하는 통신 규격 모듈입니다.
