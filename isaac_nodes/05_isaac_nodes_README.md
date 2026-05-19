# isaac_nodes

## 1. 모듈 역할

`isaac_nodes`는 전체 미션 관리, 배터리 감시, 로봇팔 작업 실행을 담당하는 모듈입니다.

이 모듈은 AI 인식이나 주행 실행 자체보다는, **현재 미션 상태를 기준으로 몸을 움직일지, 팔을 움직일지, 복귀할지, 하역할지, 충전 대응을 할지 판단하는 역할**을 담당합니다.

---

## 2. 예상 폴더 구조

```text
isaac_nodes/
├─ isaac_nodes/
│  ├─ __init__.py
│  ├─ mission_manager_node.py
│  ├─ battery_monitor_node.py
│  ├─ arm_executor_node.py
│  │
│  └─ manipulation_primitives/
│     ├─ __init__.py
│     ├─ pick_mineral.py
│     ├─ place_to_cargo.py
│     ├─ unload_to_base.py
│     └─ deploy_solar_panel.py
│
├─ package.xml
└─ setup.py
```

---

## 3. 핵심 노드 설명

### `mission_manager_node.py`

전체 미션 흐름을 관리하는 중앙 노드입니다.

Mission Manager는 주행을 직접 실행하거나 로봇팔을 직접 제어하지 않습니다.  
대신 현재 미션 상태를 보고 어떤 실행 모듈을 호출할지 결정합니다.

주요 판단 예시는 다음과 같습니다.

```text
광석이 멀리 있음
→ 로버 이동 필요
→ navigation_manager_node에 이동 요청

광석 근처 도착
→ 로봇팔 작업 필요
→ arm_executor_node에 pick 요청

cargo 적재 완료
→ 기지 복귀 필요
→ navigation_manager_node에 복귀 요청

기지 도착
→ 하역 필요
→ arm_executor_node에 unload 요청

배터리 부족
→ 태양광판 전개 필요
→ arm_executor_node에 deploy_solar_panel 요청
```

즉, Mission Manager는 **미션 단계에 따라 몸을 움직일지, 팔을 움직일지, 대기할지, 복귀할지 결정하는 상위 상태 관리자**입니다.

---

### `battery_monitor_node.py`

배터리 상태를 감시하는 노드입니다.

주요 역할은 다음과 같습니다.

```text
- 배터리 잔량 확인
- 배터리 부족 여부 판단
- 충전 상태 확인
- mission_manager_node에 배터리 상태 전달
```

배터리가 부족하면 Mission Manager는 주행을 중단하거나 태양광판 전개 작업을 요청할 수 있습니다.

---

### `arm_executor_node.py`

로봇팔 작업을 실행하는 노드입니다.

Mission Manager가 로봇팔 작업을 요청하면, Arm Executor가 해당 조작 단위 동작을 호출합니다.

주요 역할은 다음과 같습니다.

```text
- 광석 집기
- cargo에 적재
- 기지에서 하역
- 태양광판 전개
```

---

## 4. `manipulation_primitives/` 폴더 설명

`manipulation_primitives/`는 로봇팔 조작 단위 동작을 구현하는 폴더입니다.

### `pick_mineral.py`

로봇팔로 광석을 집는 동작을 구현합니다.

### `place_to_cargo.py`

집은 광석을 로버의 cargo에 적재하는 동작을 구현합니다.

### `unload_to_base.py`

기지에 도착한 뒤 cargo에 실린 광석을 하역하는 동작을 구현합니다.

### `deploy_solar_panel.py`

배터리 부족 시 로봇팔을 이용해 태양광판을 전개하는 동작을 구현합니다.

---

## 5. Mission Manager와 Navigation Manager 차이

```text
mission_manager_node
= 미션상 이동이 필요한지, 팔 작업이 필요한지 결정

navigation_manager_node
= 이동이 필요할 때 어떻게 이동할지 관리

mobile_base_executor_node
= 실제 이동 명령 실행

arm_executor_node
= 실제 로봇팔 작업 실행
```

예시는 다음과 같습니다.

```text
mission_manager_node:
광석 위치로 이동해야 한다

navigation_manager_node:
장애물을 피해서 이동해야 한다

mobile_base_executor_node:
실제 속도 명령을 발행한다
```

또는:

```text
mission_manager_node:
광석을 집어야 한다

arm_executor_node:
pick_mineral 동작을 실행한다
```

---

## 6. 이 모듈에 들어가면 안 되는 것

`isaac_nodes`에는 아래 기능을 직접 넣지 않습니다.

```text
- 카메라 기반 광석 인식
- 강화학습 정책 학습
- 로버 주행 단위 동작
- Isaac Sim 월드 에셋 관리
```

---

## 7. 한 줄 요약

`isaac_nodes`는 전체 미션 상태를 판단하고, 주행 모듈 또는 로봇팔 실행 모듈로 작업을 분기시키는 미션/로봇팔 관리 모듈입니다.
