# 👁️🦾 T2 Perception + M0609 — 담당자 브리프

> **광물 인식 (Vision) + 매니퓰레이션 (M0609)**
> 단일 트랙이지만 **두 영역**. 본인 페이스로 진행 가능.

---

> 📦 **이 트랙이 작업하는 패키지 위치**: [PACKAGE_MAPPING.md](PACKAGE_MAPPING.md) 참조 (팀 레포 9개 패키지 중 어디서 코딩하는지 명시).


## 📑 목차

1. [왜 이 트랙이 차별화 핵심인가](#1-왜-이-트랙이-차별화-핵심인가)
2. [당신이 만들 2개 큰 모듈](#2-당신이-만들-2개-큰-모듈)
3. [Vision (광물 인식) — 색 기반](#3-vision-광물-인식--색-기반)
4. [M0609 매니퓰레이션 — Tier 1.5](#4-m0609-매니퓰레이션--tier-15)
5. [추가 보조 작업 (T1/T4)](#5-추가-보조-작업-t1t4)
6. [인터페이스](#6-인터페이스)
7. [일정과 마일스톤](#7-일정과-마일스톤)
8. [흔한 함정](#8-흔한-함정)
9. [도구와 참고 자료](#9-도구와-참고-자료)
10. [DoD](#10-dod)

---

## 1. 왜 이 트랙이 차별화 핵심인가

### 클론 vs 우리 — 본질적 차이

| | 클론 (RLRoverLab) | 우리 |
|---|------------------|------|
| 광물 위치 인지 | **GT 좌표 cheat** | **카메라 → vision detection** |
| 매니퓰레이션 | **없음** | **M0609로 진짜 pick & place** |

→ T2의 역할 = **"진짜 자율 시스템"의 입증**.

### 물류창고 팀 vs 우리

| | 물류창고 팀 | 우리 |
|---|------------|------|
| 매니퓰레이션 | 단순 ↑↓ 잡기 | M0609 6축 IK + 시각 인식 결합 |
| 환경 인지 | 알려진 선반 위치 | 미지 화성 지형의 vision 탐사 |

→ **물류창고 팀이 못 하는 영역**.

### 발표 임팩트 포인트

T2가 만들 시연:
1. 📷 화면에 광물 인식 → 좌표 표시 → 가치 점수 표시
2. 🦾 로버가 광물 앞에 도달 → M0609이 잡아서 cargo bin으로
3. 카고에 광물 누적 → 가치 점수 누적

→ 발표 영상의 **가장 시각적으로 멋진 부분**.

---

## 2. 당신이 만들 2개 큰 모듈

```
T2 = Vision (인식) + M0609 (조작)
   │
   ├ 1. Vision 광물 detection      (25h)
   │    └ 카메라 이미지 → 광물 좌표 + 가치 점수
   │
   ├ 2. M0609 Manipulation Tier 1.5 (30h)
   │    └ scripted trajectory + 광물 텔레포트
   │
   ├ 3. T1/T4 보조 작업              (15h)
   │    └ 시간 남으면 다른 트랙 도와줌
   │
   └ 합계: 70h (시간 가용 65h, 살짝 over)
```

**핵심 단순화**:
- Vision = **색 기반** (HSV threshold). CNN 학습 안 함.
- M0609 = **scripted**. 진짜 IK + force feedback 안 함.

이게 8일에 가능한 이유.

---

## 3. Vision (광물 인식) — 색 기반

### 광물 시각 디자인 — T1과 협의

광물을 **명확한 단색 USD**로 만들면 detection 매우 쉬워짐:

| 광물 타입 | RGB | HSV hue 범위 | 가치 점수 |
|---------|-----|------------|:---:|
| **mineral_blue** | (50, 100, 240) | hue 210~230° | 10 |
| **mineral_red** | (230, 60, 60) | hue 0~10° (or 350~360°) | 25 |
| **mineral_yellow** | (240, 220, 50) | hue 50~70° | 50 |

→ T1에 요청: 광물 USD를 위 색으로 생성.

### Detection 알고리즘

```python
import cv2
import numpy as np

def detect_minerals(rgb_image, estimated_pose):
    """
    rgb_image: (480, 640, 3) numpy array
    estimated_pose: T5의 추정 위치 (x, y, z, yaw)
    
    returns: list of detection dicts
    """
    # 1. RGB → HSV
    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
    
    detections = []
    
    # 2. 각 광물 타입별 detection
    color_configs = [
        {"name": "mineral_blue",   "hue_range": (105, 120), "value": 10},
        {"name": "mineral_red",    "hue_range": (0, 10),    "value": 25},
        {"name": "mineral_yellow", "hue_range": (25, 35),   "value": 50},
    ]
    
    for cfg in color_configs:
        # 3. Color mask
        h_low, h_high = cfg["hue_range"]
        mask = cv2.inRange(hsv,
                          (h_low, 100, 100),     # S/V min
                          (h_high, 255, 255))    # S/V max
        
        # 4. Morphology (노이즈 제거)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5)))
        
        # 5. Connected components
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
        
        # 6. 각 객체별 처리
        for i in range(1, num_labels):  # 0은 배경
            area = stats[i, cv2.CC_STAT_AREA]
            if area < 50:  # 너무 작으면 무시
                continue
            
            cx, cy = centroids[i]
            
            # 7. 2D 픽셀 → 3D 월드 좌표 (estimated_pose 사용)
            world_pos = pixel_to_world(cx, cy, estimated_pose, 
                                       camera_intrinsics, camera_extrinsics)
            
            # 8. Confidence (영역 크기 기반)
            confidence = min(area / 500.0, 1.0)
            if confidence < 0.5:
                continue
            
            detections.append({
                "class_name": cfg["name"],
                "world_position": world_pos,
                "confidence": confidence,
                "value_score": cfg["value"],
                "mineral_id": -1,  # 매칭은 mission FSM이
                "bbox_size_m": estimate_bbox_size(area, world_pos)
            })
    
    return detections
```

### 2D → 3D 투영

```python
def pixel_to_world(px, py, estimated_pose, K, T_cam_robot):
    """
    카메라 픽셀 → 월드 좌표
    
    가정: 광물은 지면 (z ≈ 0)에 있음 → ray-ground intersection
    """
    # 1. 픽셀 → 카메라 frame ray
    ray_camera = np.linalg.inv(K) @ np.array([px, py, 1.0])
    ray_camera /= np.linalg.norm(ray_camera)
    
    # 2. 카메라 frame → robot frame
    ray_robot = T_cam_robot[:3, :3] @ ray_camera
    
    # 3. robot frame → world frame
    rx, ry, rz, yaw = estimated_pose
    R = rotation_matrix_z(yaw)
    ray_world = R @ ray_robot
    camera_world = np.array([rx, ry, rz]) + R @ T_cam_robot[:3, 3]
    
    # 4. Ray와 z=0 평면의 교점
    t = -camera_world[2] / ray_world[2]
    intersection = camera_world + t * ray_world
    
    return intersection
```

→ **T5의 estimated_pose 사용** (GT cheat 아님). 위치 추정 정확도에 따라 detection 좌표 정확도 결정.

### Mineral ID 매칭

T1의 meta.json에는 각 광물에 unique id가 있음. T2 detection이 같은 광물인지 판정:

```python
def match_to_meta(detection_world_pos, meta_minerals, threshold=1.0):
    """detection 좌표 vs meta의 광물 좌표 매칭"""
    best_match = None
    min_dist = threshold
    for mineral in meta_minerals:
        d = np.linalg.norm(detection_world_pos - mineral["position"])
        if d < min_dist:
            min_dist = d
            best_match = mineral["id"]
    return best_match
```

→ 매칭 실패하면 mineral_id = -1 (T3가 처리).

---

## 4. M0609 매니퓰레이션 — Tier 1.5

### Tier 분류

| Tier | 구현 | 시간 |
|------|------|:---:|
| Tier 1 | 광물 도달 = 사라지는 애니메이션만 | 15h |
| **Tier 1.5** ⭐ | **Scripted trajectory + 광물 텔레포트** | **30h** |
| Tier 2 | MoveIt + 실제 IK + grasp pose detection | 65h |

→ **Tier 1.5 권장**. 시각적 "잡는" 동작 + 시뮬레이션 단순.

### M0609 USD 부착

```
1단계: USD asset 확보
  ├ ROS-Industrial의 doosan_robot 패키지에 URDF 존재
  ├ URDF → USD 변환 (Isaac Sim 도구)
  └ 또는 단순 6-link 매니퓰레이터 직접 모델링 (1시간)

2단계: Rover에 부착
  ├ Rover Body 위에 articulation joint로 연결
  ├ Base 위치: body 중앙 (0, 0, 0.3) — 마스트 카메라 충돌 회피
  └ 접힌 자세 (collapsed)로 시작

3단계: USD에 joint 설정
  ├ joint_1 ~ joint_6 (6축)
  ├ 각 joint 회전 한계, 속도 한계
  └ effort_limit 적절히
```

### Scripted Trajectory

```python
class M0609Controller:
    def __init__(self, env):
        self.env = env
        self.joint_targets = np.zeros(6)
        self.state = "IDLE"
    
    def pick_sequence(self, mineral_world_pos):
        """5단계 scripted pick"""
        sequence = [
            ("EXTEND_PRE",   self._joints_extend_above(mineral_world_pos), 2.0),
            ("DESCEND",      self._joints_descend(mineral_world_pos),       1.5),
            ("GRASP",        self._joints_close_gripper(),                   0.5),
            ("LIFT",         self._joints_extend_above(mineral_world_pos), 1.0),
            ("STOW",         self._joints_home(),                            2.0),
        ]
        
        for step_name, joint_target, duration in sequence:
            self.state = step_name
            self._move_to(joint_target, duration)
            yield step_name
        
        # 광물을 cargo bin으로 텔레포트 (Tier 1.5 cheat)
        self._teleport_mineral_to_cargo(mineral_id)
        self.state = "DONE"
    
    def _move_to(self, target, duration):
        """관절 목표값으로 부드럽게 이동 (linear interpolation)"""
        steps = int(duration / self.env.physics_dt)
        start = self.joint_targets.copy()
        for i in range(steps):
            t = i / steps
            self.joint_targets = start + t * (target - start)
            self.env.set_m0609_joint_targets(self.joint_targets)
            self.env.step_physics()
    
    def _teleport_mineral_to_cargo(self, mineral_id):
        """광물 USD prim을 cargo bin 위치로 이동"""
        mineral_prim = stage.GetPrimAtPath(f"/World/Minerals/mineral_{mineral_id}")
        mineral_prim.GetAttribute("xformOp:translate").Set(cargo_bin_pos)
        # 또는 invisible 처리
```

### IK 단순화

진짜 IK 안 풀어도 됨. **각 광물 위치마다 미리 계산된 joint 값** 사용:

```python
def _joints_extend_above(self, world_pos):
    """매니퓰레이터를 world_pos 위로 hover"""
    # 단순화: world_pos → robot frame → joint angle table lookup
    relative_pos = self._world_to_arm_frame(world_pos)
    
    # IK 대신 미리 정의된 조인트 자세 (광물이 평탄한 지면에 있다고 가정)
    if relative_pos[0] > 0.5:  # 앞쪽
        return np.array([0, -π/4, π/2, 0, π/4, 0])
    elif relative_pos[0] < -0.5:  # 뒤쪽
        return np.array([π, -π/4, π/2, 0, π/4, 0])
    else:
        return np.array([0, 0, 0, 0, 0, 0])
```

→ **시뮬레이션이라서 가능**. 진짜 로봇이면 IK 필수, 우리는 안 함.

---

## 5. 추가 보조 작업 (T1/T4)

T2 본 작업 + 약 15h 여유:

| 보조 작업 | 시간 | 누구 도움? |
|----------|:---:|:--------:|
| 광물 USD 생성 (3색 단순 메쉬) | 3h | T1 |
| Replicator 광물 데이터셋 생성 (stretch) | 8h | T1 |
| UI 광물 마커 위젯 | 5h | T4 |
| 발표 영상 편집 보조 | 5h | T4 |

→ 본인 작업 끝나면 PM(T4)에게 물어보고 도움.

---

## 6. 인터페이스

### Consume (입력)

| 인터페이스 | Producer | 사용처 |
|----------|:--------:|--------|
| **Isaac Sim camera** | Isaac Sim | RGB 이미지 (640×480) |
| **I5** /rover/estimated_pose | T5 | 2D→3D 투영 시 사용 |
| **I3** /mission/pick_request | T3 | M0609 시작 트리거 |
| **I1** meta.json | T1 | 광물 id 매칭 |

### Produce (출력)

| 인터페이스 | Consumer | 빈도 |
|----------|:--------:|:----:|
| **I2** /perception/detections | T3, T4 | 10 Hz |
| **I4** /mission/pick_response | T3 | event (Pick 완료 시) |

→ 상세는 [interfaces/INTERFACE_CONTRACTS.md](../interfaces/INTERFACE_CONTRACTS.md).

---

## 7. 일정과 마일스톤

```
Day 1 (화)
  □ M0609 USD asset 확보 (가장 큰 unknown — spike!)
  □ Vision PoC: 클론 terrain에 단색 sphere 1개 두고 HSV threshold
  → EOD ⚠️ 게이트: "M0609 asset이 Isaac Sim에서 로드되는가?"
     실패 시 → T1과 함께 단순 모델 직접 생성

Day 2 (수) — Vision 완성
  □ HSV detection 3색 모두 동작
  □ 2D→3D 투영 (GT pose stub으로 우선 검증)
  □ ROS2 publish (/perception/detections)
  → EOD: 광물 좌표 detection 영상

Day 3 (목) — M0609 통합 시작
  □ M0609 rover에 부착
  □ 5단계 scripted trajectory 정의
  □ pick_sequence 동작 (광물 텔레포트)
  → EOD: 가짜 trigger로 pick 1회 동작

Day 4 (금) — 통합
  □ T5의 estimated_pose 받아서 detection
  □ T3의 pick_request 받아서 trigger
  □ I4 response 발행
  → EOD: end-to-end 단일 사이클 (vision → pick) 동작

Day 5 (토)
  □ 가치점수 (mineral_blue/red/yellow) 분리
  □ 다양한 조명 / 지형에서 robust 검증

Day 6 (일) — 폴리싱
  □ Edge case (광물 안 보임, 잘못된 detection)
  □ 보조 작업 시작 (T1/T4 도움)
  → 일요일 EOD ⚠️ 게이트: end-to-end 데모 1회

Day 7 (월) — 폴리싱 + 보조
Day 8 AM (수) — 최종
```

---

## 8. 흔한 함정

| 함정 | 증상 | 대응 |
|------|------|------|
| **M0609 USD asset 호환 안 됨** | Day 1에 막힘 | 단순 6-link 매니퓰레이터 직접 모델링 (1h) — 시각 효과는 충분 |
| **HSV 범위 잘못 설정** | detection 없거나 false positive 많음 | T1과 광물 색 합의 후 광물 표본 캡처 → 색 범위 측정 |
| **2D→3D 좌표가 GT랑 차이 큼** | 광물 위치 부정확 | 카메라 intrinsic/extrinsic 정확히 설정. Isaac Sim에서 확인 |
| **`mineral_id` 매칭 실패** | T3가 어떤 광물인지 못 앎 | 매칭 threshold 1m로 적절히, mineral_id=-1도 처리 |
| **pick_request 중복 수신** | M0609 동작 꼬임 | request_id로 중복 방지 |
| **scripted trajectory가 부자연** | "로봇팔이 점프하는" 동작 | 보간 step 충분히 (60 step / 1초 권장) |
| **광물 텔레포트 시 충돌** | PhysX 에러 | mineral_prim의 collision 비활성화 (cargo로 옮길 때) |
| **Camera 위치가 M0609과 충돌** | 시각 가림 | 마스트 카메라 (0, -0.2, 0.7) 위치 사용 |

---

## 9. 도구와 참고 자료

### Python 라이브러리

```bash
pip install opencv-python  # HSV detection
```

- `cv2.cvtColor`, `cv2.inRange`, `cv2.connectedComponentsWithStats`
- `cv2.morphologyEx` (노이즈 제거)

### Isaac Sim Camera API

```python
from isaacsim.sensors.camera import Camera
cam = Camera(prim_path=".../Camera_1P", resolution=(640, 480))
cam.initialize()
rgba = cam.get_rgba()  # (H, W, 4)
rgb = rgba[..., :3]
```

### 클론 참고 파일

- [03_eval_ros2.py:214-224](../rover/sim/scripts/03_eval_ros2.py#L214-L224) — 카메라 초기화 예
- [mission/camera_utils.py](../rover/sim/mission/camera_utils.py) — 듀얼 뷰포트 설정

### M0609 자료

- ROS-Industrial doosan-robot 패키지
- 또는 직접 모델링: Isaac Sim의 Articulation 도구

---

## 10. DoD

### 최소 (Day 6 EOD)
- ✅ HSV detection 3색 모두 동작 (단일 terrain에서 검증)
- ✅ 2D→3D 좌표 정확도 < 50cm (GT 대비)
- ✅ ROS2 /perception/detections publish 10Hz
- ✅ M0609 Tier 1.5 동작 (scripted trajectory + 텔레포트)
- ✅ /mission/pick_response 정상 발행
- ✅ end-to-end 단일 사이클 (vision → pick → response) 성공

### 권장 (Day 7-8)
- ✅ 5개 terrain에서 detection 작동 확인
- ✅ false positive rate < 5%
- ✅ pick success rate > 90% (Tier 1.5)
- ✅ 발표용 시연 영상

### Stretch
- ⏳ CNN 기반 detection (YOLO + Replicator)
- ⏳ MoveIt + 실제 IK (Tier 2)
- ⏳ depth camera (RGBD) 통합

---

## 🤝 다른 트랙과 동기화

- **Day 1 spike 결과 즉시 PM에게**: M0609 asset 호환성
- **T1과 광물 색 합의**: HSV 범위
- **Day 4 T3와 통합 미팅**: I3/I4 메시지 검증
- **매일 18:00 DIST**: PM이 통합 테스트

---

## 💪 한 마디

색기반 + scripted = 단순하지만 8일에 강력. CNN/IK 같은 "진짜" 솔루션은 다음 마일스톤. 지금은 **"시각적으로 멋진 데모"가 최우선**.

Day 1 M0609 spike만 잘 통과하면 나머지 7일은 페이스 안정적. spike 막히면 즉시 PM 호출.

화이팅 👁️🦾
