# isaac_ai

## 1. 모듈 역할

`isaac_ai`는 카메라/Depth/LiDAR 기반 환경 인식과 강화학습 기반 주행 판단을 담당하는 AI 모듈입니다.

이 모듈은 로버를 직접 움직이는 실행 모듈이 아닙니다.  
센서 데이터를 분석하고, 로버가 다음에 어떤 주행 행동을 선택해야 하는지 판단하는 역할을 합니다.

---

## 2. 예상 폴더 구조

```text
isaac_ai/
├─ isaac_ai/
│  ├─ __init__.py
│  ├─ perception_node.py
│  ├─ driving_policy_node.py
│  ├─ rl_trainer.py
│  │
│  ├─ vision/
│  │  ├─ __init__.py
│  │  ├─ mineral_detector.py
│  │  ├─ depth_estimator.py
│  │  ├─ obstacle_detector.py
│  │  └─ terrain_analyzer.py
│  │
│  └─ rl/
│     ├─ __init__.py
│     ├─ driving_policy.py
│     ├─ reward_function.py
│     ├─ rl_environment.py
│     └─ policy_loader.py
│
├─ models/
│  └─ mineral_detector.pt
│
├─ policies/
│  └─ driving_policy.pt
│
├─ package.xml
└─ setup.py
```

---

## 3. 핵심 노드 설명

### `perception_node.py`

카메라, Depth, LiDAR 데이터를 입력받아 환경 인식을 수행하는 ROS2 노드입니다.

주요 역할은 다음과 같습니다.

```text
- 광석 인식
- 광석 3D 위치 추정
- 장애물 인식
- 지형 경사 판단
- 주행 가능 영역 판단
```

중요한 점은 카메라 자체가 광석을 인식하는 것이 아니라는 점입니다.  
카메라는 이미지 데이터를 제공하고, 실제 인식은 `perception_node`가 수행합니다.

입력 예시는 다음과 같습니다.

```text
/camera/image_raw
/camera/depth_image_raw
/camera/camera_info
/lidar/points
```

출력 예시는 다음과 같습니다.

```text
/mineral_pose
/obstacle_info
/terrain_state
/drivable_area
```

---

### `driving_policy_node.py`

`perception_node`의 결과와 로버 상태를 바탕으로 다음 주행 행동을 선택하는 노드입니다.

이 노드는 전체 미션 시퀀스를 생성하지 않습니다.  
현재 상태를 보고 **다음 주행 행동 하나**를 선택합니다.

선택 가능한 행동 예시는 다음과 같습니다.

```text
FORWARD
SLOW_FORWARD
TURN_LEFT
TURN_RIGHT
STOP
AVOID_OBSTACLE
RETURN_TO_BASE
```

강화학습을 적용할 경우 `policies/driving_policy.pt` 파일을 불러와 현재 상태에 대한 행동을 선택합니다.

---

### `rl_trainer.py`

주행 안정성을 위한 강화학습 정책을 학습시키는 코드입니다.

이 파일은 실시간 실행 노드라기보다는 학습용 코드에 가깝습니다.

학습 과정은 다음과 같습니다.

```text
1. Isaac Sim 환경에서 로버 주행 시도
2. 현재 상태 관찰
3. 행동 선택
4. 결과 확인
5. 보상 계산
6. 정책 업데이트
7. driving_policy.pt 저장
```

---

## 4. `vision/` 폴더 설명

`vision/`은 `perception_node.py`가 내부적으로 호출하는 비전 처리 로직을 분리한 폴더입니다.

### `mineral_detector.py`

RGB 이미지에서 광석 또는 자원 객체를 탐지하는 코드입니다.

### `depth_estimator.py`

Depth 이미지와 카메라 정보를 이용해 광석 또는 장애물의 거리와 3D 위치를 계산하는 코드입니다.

### `obstacle_detector.py`

카메라/Depth/LiDAR 데이터를 이용해 전방 장애물과 위험 구역을 판단하는 코드입니다.

### `terrain_analyzer.py`

지형의 경사도와 주행 가능 영역을 판단하는 코드입니다.

---

## 5. `rl/` 폴더 설명

`rl/`은 강화학습 정책과 학습 관련 로직을 분리한 폴더입니다.

### `driving_policy.py`

현재 상태를 입력받아 다음 주행 행동을 선택하는 정책 코드입니다.

### `reward_function.py`

강화학습에서 사용할 보상 함수를 정의하는 코드입니다.

예상 보상 기준은 다음과 같습니다.

```text
목표에 가까워짐: +
충돌 발생: -
전복 발생: 큰 -
위험 경사 진입: -
안정적으로 이동: +
```

### `rl_environment.py`

Isaac Sim 기반 강화학습 환경을 정의하는 코드입니다.

### `policy_loader.py`

학습된 `driving_policy.pt` 파일을 불러오는 코드입니다.

---

## 6. 모델 파일 설명

### `models/mineral_detector.pt`

광석 인식 모델 파일입니다.

### `policies/driving_policy.pt`

강화학습 학습 결과로 저장되는 주행 정책 모델 파일입니다.

이 파일은 `driving_policy_node.py`가 불러와 사용합니다.

---

## 7. 이 모듈에 들어가면 안 되는 것

`isaac_ai`에는 아래 기능을 직접 넣지 않습니다.

```text
- 로버 바퀴 제어 명령 실행
- 로봇팔 pick & place 실행
- 미션 전체 상태 머신
- launch 파일 관리
- Isaac Sim 월드 에셋 관리
```

---

## 8. 한 줄 요약

`isaac_ai`는 센서 데이터를 분석해 환경을 인식하고, 강화학습 정책으로 다음 주행 행동을 선택하는 AI 모듈입니다.
