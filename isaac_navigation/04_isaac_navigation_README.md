# isaac_navigation

## 1. 모듈 역할

`isaac_navigation`은 화성 탐사 로버의 주행 흐름 관리와 실제 이동 실행을 담당하는 모듈입니다.

AI가 “어떤 주행 행동이 적절한지” 판단하면, Navigation 모듈은 이를 바탕으로 실제 주행 흐름을 관리하고 로버 이동 명령을 실행합니다.

---

## 2. 예상 폴더 구조

```text
isaac_navigation/
├─ isaac_navigation/
│  ├─ __init__.py
│  ├─ navigation_manager_node.py
│  ├─ mobile_base_executor_node.py
│  │
│  └─ navigation_primitives/
│     ├─ __init__.py
│     ├─ drive_to_target.py
│     ├─ avoid_obstacle.py
│     └─ stop_rover.py
│
├─ package.xml
└─ setup.py
```

---

## 3. 핵심 노드 설명

### `navigation_manager_node.py`

주행 흐름을 관리하는 상위 주행 노드입니다.

Mission Manager가 “이동이 필요하다”고 판단하면, Navigation Manager가 **어떻게 이동할지**를 관리합니다.

주요 역할은 다음과 같습니다.

```text
- 자율주행 모드 관리
- 수동 조종 모드 관리
- 이동 목표 처리
- 장애물 회피 판단
- 주행 가능 영역 확인
- driving_policy_node가 선택한 주행 행동 반영
- mobile_base_executor_node에 주행 실행 요청
```

즉, `navigation_manager_node`는 로버의 주행 관리자입니다.

---

### `mobile_base_executor_node.py`

실제 로버 이동 명령을 실행하는 노드입니다.

Navigation Manager가 이동 요청을 보내면, 이 노드는 실제 로버 제어 명령을 생성합니다.

주요 역할은 다음과 같습니다.

```text
- 전진 명령 실행
- 정지 명령 실행
- 회전 명령 실행
- 목표 지점 이동 명령 실행
- /cmd_vel 또는 wheel command 발행
```

즉, `mobile_base_executor_node`는 주행 실행자입니다.

---

## 4. Navigation Manager와 Mobile Base Executor 차이

```text
navigation_manager_node
= 어떻게 이동할지 관리

mobile_base_executor_node
= 실제로 로버를 움직임
```

예시는 다음과 같습니다.

```text
Mission Manager:
광석 위치로 이동해야 한다

Navigation Manager:
장애물을 피해서 오른쪽으로 우회해야 한다

Mobile Base Executor:
오른쪽 회전 및 전진 속도 명령을 발행한다
```

---

## 5. `navigation_primitives/` 폴더 설명

`navigation_primitives/`는 로버 주행 단위 동작을 구현하는 폴더입니다.

### `drive_to_target.py`

로버를 목표 위치까지 이동시키는 단위 동작 코드입니다.

### `avoid_obstacle.py`

장애물을 회피하는 단위 동작 코드입니다.

### `stop_rover.py`

로버를 정지시키는 단위 동작 코드입니다.

---

## 6. 통신 흐름

주행 관련 흐름은 다음과 같습니다.

```text
perception_node
→ driving_policy_node
→ navigation_manager_node
→ mobile_base_executor_node
→ Isaac Sim Rover
```

Mission Manager가 이동이 필요하다고 판단하는 경우에는 다음 흐름이 추가됩니다.

```text
mission_manager_node
→ navigation_manager_node
```

즉, Mission Manager는 “이동해야 한다”를 결정하고, Navigation Manager는 “어떻게 이동할지”를 결정합니다.

---

## 7. 이 모듈에 들어가면 안 되는 것

`isaac_navigation`에는 아래 기능을 직접 넣지 않습니다.

```text
- 광석 인식 AI
- 강화학습 모델 학습 코드
- 로봇팔 pick & place
- 배터리 상태 감시
- 전체 미션 상태 머신
```

---

## 8. 한 줄 요약

`isaac_navigation`은 로버가 어디로, 어떤 방식으로 이동할지 관리하고 실제 주행 명령을 실행하는 주행 모듈입니다.
