# YOLO Mineral Detection Plan

> **목표**: Mars terrain 의 Mineral 객체 3종을 wrist-camera 영상에서 YOLO 로 detect.
> **결과물**: `models/best.pt` + 추론 node (`isaac_perception` 패키지 내).
> **참고**: 3일차 강의 (카메라 센서, Replicator), 4일차 reference (`vision_tracker.py` HSV detector).

---

## 0. 클래스 정의

USD prim 의 이름 prefix 와 **실제 시각 색상 + 형태** 가 다름. 클래스는 **시각 기준**.

| Prim prefix | 실제 시각 색상 | 실제 형태 | YOLO class id | name |
|---|---|---|---|---|
| `blue_*` (예: `/World/Minerals/blue_0001`) | **cyan/teal** (밝은 청록) | **수정 결정 클러스터** (여러 결정이 모여있는 비정형 polytope) | **0** | blue_mineral |
| `red_*`  (예: `/World/Minerals/red_0002`) | **green** (진녹색) | **정육면체 박스** (가스 박스) | **1** | green_gas |
| `yellow_*` (예: `/World/Minerals/yellow_0007`) | **yellow** (밝은 노랑) | **수정 결정 클러스터** (spike 모양) | **2** | yellow_mineral |

**⚠️ 중요**: USD prim 이름이 "Cube" 라도 실제 mesh 는 cube 가 아닐 수 있음. blue/yellow 는 결정 클러스터, red 만 진짜 cube. → bbox 추정 시 fixed-size cube 가정 X. **실제 mesh 의 axis-aligned bounding box** 사용.

`dataset.yaml` 의 `names:` 순서: `[blue_mineral, green_gas, yellow_mineral]`

---

## 1. 전체 파이프라인

```
┌──────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│ 1. Data 수집      │───▶│ 2. Auto-annotate │───▶│ 3. Train YOLO    │
│  Isaac Sim 에서   │    │  Isaac Sim 의     │    │  ultralytics     │
│  RGB 프레임 캡쳐  │    │  ground-truth로   │    │  YOLOv8n/s       │
│                  │    │  bbox 자동 생성   │    │                  │
└──────────────────┘    └──────────────────┘    └──────────────────┘
                                                          │
                            ┌─────────────────────────────┘
                            ▼
                  ┌──────────────────────┐
                  │ 4. Deploy (best.pt)  │
                  │  wrist camera 영상에 │
                  │  실시간 추론          │
                  └──────────────────────┘
```

---

## 2. Phase 1 — 데이터 수집 (synthetic via Isaac Sim)

### 2.1 사용 센서 — Vehicle.usd 의 기존 navigation camera

`/home/rokey/dev_ws/rover_ws/src/Vehicle.usd` 안에 이미 부착된 센서 사용:

| Prim path | 종류 | 용도 |
|---|---|---|
| `/Vehicle/Vehicle/rover/Body/Camera` | RGB Camera (depth 활성화 가능) | YOLO 추론 + depth 로 3D 위치 추정 |
| `/Vehicle/Vehicle/rover/Body/Imu_Sensor` | IMU | rover pose / odometry (확장 영역) |

**Camera 위치** (rover Body 기준 local offset)
- X 방향 +0.37m (rover 앞쪽 forward)
- Z 방향 +0.27m (rover 위쪽 상단)
- → **차량 전방-상단 navigation 마스트 카메라**

**Wrist camera (`pickplace_visual_rover.py` 의 angle_bracket 부착 RealSense) 와 구분**
- Wrist camera: gripper 끝에 부착, **manipulation (pick&place) 용**
- **Navigation camera (이것!)**: rover Body 에 부착, **detection + 주행 시 자원 탐지용**

### 2.2 도구

- **Isaac Sim Camera wrapper** (`isaacsim.sensors.camera.Camera`) 로 기존 Camera prim 을 wrap → RGB + Depth 동시 활성:
  ```python
  cam = Camera(prim_path="/Vehicle/Vehicle/rover/Body/Camera",
               resolution=(640, 480))
  cam.initialize()
  cam.add_distance_to_image_plane_to_frame()      # depth 활성화
  rgb = cam.get_rgba()
  depth = cam.get_current_frame()["distance_to_image_plane"]
  ```
- (선택) **Isaac Replicator** (`omni.replicator.core`) — 자동 도메인 랜덤화 (조명, fog 등)

### 2.3 캡쳐 전략

scene: `terrain_00022.usd` + Vehicle.usd 로드 (Vehicle 의 navigation camera 가 viewpoint)

**중요 제약 — Mineral 의 동적 특성**
- ⚠️ **Mineral 위치는 매 USD load 마다 랜덤**으로 생성됨 (USD 안 script 또는 author 의 randomizer)
- ⚠️ **Play 시작 시 "튀어 나옴"** — 공중에서 떨어지고, 충돌로 bounce/굴러가다가 결국 정착
- ⚠️ **반드시 settle 완료 후 사진 촬영** — 움직이는 중 캡쳐 시 motion blur + ground-truth bbox 어긋남

| 변동 요소 | 범위 |
|---|---|
| **Rover spawn 위치** | terrain 내 다양한 XY (mineral 군집 근처) |
| **Rover heading** | yaw 0~360° (전방 카메라가 다양한 방향 봄) |
| **카메라 FOV** | Vehicle.usd 의 Camera 기본값 사용 |
| Sun 각도 (조명) | morning / noon / dusk (Mars sky color 변화) |
| Mineral 분포 | USD load 마다 랜덤 (자연 augmentation) |
| 프레임 수 | 1500~3000 장 (train:val = 8:2) |

**중요**: 카메라는 rover Body 에 고정된 navigation cam. rover 위치/회전이 바뀌면 카메라 자동으로 따라감. → "차량이 가는 길에 보이는 자원" 시점 그대로 학습 데이터 = 추론 시점과 동일.

**Episode 단위 캡쳐 (권장)**
- 1 episode = 1 USD load + settle + N 장 캡쳐 (다른 카메라 위치들)
- 매 episode 마다 mineral 배치 다름 → 데이터 자연 다양화
- 같은 episode 안에선 동일 분포지만 시점 다른 multi-view → 학습에 유리

### 2.3 Settle 감지 알고리즘 (필수)

캡쳐 전 mineral 들의 움직임 완료 확인. 두 가지 방법:

**A. Fixed-frame 방식 (단순)**
```python
SETTLE_FRAMES = 240   # 4초 @ 60Hz — 보수적으로 길게
for _ in range(SETTLE_FRAMES):
    world.step(render=False)   # render 안 함, 빠르게
# 그 후 캡쳐
```

**B. Velocity-based 방식 (정확)**
```python
SETTLE_VEL_THRESHOLD = 0.01   # m/s — 1cm/s 이하면 정지로 간주
SETTLE_STABLE_FRAMES = 30     # 30 frame 연속 정지면 settled
MAX_SETTLE_FRAMES = 600       # 10초 timeout

stable_count = 0
for k in range(MAX_SETTLE_FRAMES):
    world.step(render=False)
    max_vel = 0.0
    for prim_path in mineral_paths:
        rb = stage.GetPrimAtPath(prim_path)
        # PhysX 의 velocity 읽기 (omni.physx API)
        v = get_rigid_body_velocity(rb)
        max_vel = max(max_vel, np.linalg.norm(v))
    if max_vel < SETTLE_VEL_THRESHOLD:
        stable_count += 1
        if stable_count >= SETTLE_STABLE_FRAMES:
            print(f"[settle] done at frame {k}, max_vel={max_vel:.4f}")
            break
    else:
        stable_count = 0
else:
    print(f"[settle] WARN: timeout at {MAX_SETTLE_FRAMES} frames")
```

**C. Position-delta 방식 (대안)**
- N frame 전 vs 현재 mineral 위치 비교
- 평균 변화량 < 1mm 이면 settled

→ 권장: **B (velocity-based)** + fallback timeout. 빠르고 결정적.

### 2.5 스크립트 골격 (`isaac_perception/scripts/collect_yolo_data.py` — 새로 만들 것)

**Episode 루프 + rover 위치 변경 + settle 대기** 패턴 (Vehicle 의 navigation cam 사용):

```python
# 의사 코드
from isaacsim.sensors.camera import Camera

N_EPISODES = 100           # episode 마다 다른 mineral 분포 + rover 위치
VIEWS_PER_EPISODE = 15     # episode 당 15 view (rover 살짝씩 이동) → total ~1500 장

# Vehicle.usd 로드 + terrain_00022 로드 (또는 통합 scene)
load_terrain_and_vehicle()

# Vehicle 의 기존 navigation camera 사용
cam = Camera(
    prim_path="/Vehicle/Vehicle/rover/Body/Camera",
    resolution=(640, 480),
)
cam.initialize()
cam.add_distance_to_image_plane_to_frame()   # depth 도 활성

frame_idx = 0
for ep in range(N_EPISODES):
    # ─── 1. terrain reload (mineral 위치 랜덤 새로 생성) ───
    reload_terrain_with_random_minerals()
    minerals = enumerate_minerals(stage)

    # ─── 2. Rover spawn — 랜덤 위치 + 랜덤 yaw ───
    rover_xy = sample_rover_spawn_xy()        # mineral 군집 근처
    rover_yaw = random_yaw()                   # 0~360°
    teleport_rover_to(rover_xy, rover_yaw)

    # ─── 3. Play + settle 대기 (mineral + rover 모두) ───
    world.play()
    wait_for_settle(minerals + [rover_body],
                    vel_threshold=0.01,
                    stable_frames=30,
                    max_frames=600)

    # ─── 4. settled ground-truth 위치 캡쳐 ───
    settled_positions = {p: read_world_xyz(p) for p in minerals}

    # ─── 5. rover 살짝씩 움직이며 view 변경 + 캡쳐 ───
    for view in range(VIEWS_PER_EPISODE):
        # rover 를 조금씩 이동 (또는 회전) — 자연스러운 주행 시점 다양화
        nudge_rover(dx=random_step(0.5, 2.0),
                    dyaw=random_angle(-30, 30))
        # 다시 짧게 settle
        wait_for_settle([rover_body], vel_threshold=0.05,
                        stable_frames=15, max_frames=60)

        # render 안정화
        for _ in range(3):
            world.step(render=True)

        # RGB 캡쳐 (Camera 가 rover 와 함께 움직였음)
        rgb = cam.get_rgba()[..., :3]
        out_name = f"{frame_idx:06d}"
        split = "train" if frame_idx % 5 != 0 else "val"
        cv2.imwrite(f"dataset/images/{split}/{out_name}.png",
                    cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

        # (선택) depth 도 저장 (학습엔 안 쓰지만 디버그/3D 추정용)
        # depth = cam.get_current_frame()["distance_to_image_plane"]
        # np.save(f"dataset/depth/{split}/{out_name}.npy", depth)

        # label 생성 (settled_positions 의 mineral 들 → 현재 cam view 로 project)
        labels = []
        for prim_path, world_pos in settled_positions.items():
            cls_id = mineral_class_from_path(prim_path)   # blue→0, red→1(green), yellow→2
            # ⚠️ Fixed cube 가정 X — 실제 mesh 의 world bbox 8 corners 사용
            corners = mesh_world_bbox_corners(stage, prim_path)
            pxs = cam.get_image_coords_from_world_points(corners)
            bbox = bbox_from_corners(pxs, img_w=640, img_h=480)
            if bbox is None or bbox.area_px2 < MIN_AREA:
                continue
            cx, cy, w, h = bbox.normalized()
            labels.append(f"{cls_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
        with open(f"dataset/labels/{split}/{out_name}.txt", "w") as f:
            f.write("\n".join(labels))
        frame_idx += 1

    world.stop()
```

**키 포인트**
- **Vehicle 의 기존 Camera 사용** → 학습 시점 = 추론 시점 일치 (sim-to-sim gap 없음)
- USD reload → mineral 랜덤 위치 → settle → rover 짧게 이동 (multi-view) → 캡쳐
- 매 view 마다 rover 가 약간씩 이동/회전 → 자연스러운 주행 시점 시뮬레이션
- Mineral 은 episode 안에서 정지 (settled positions 캐싱 사용)
- depth 도 옵션으로 저장 가능 (3D 위치 추정용 데이터)

### 2.6 폴더 구조 (수집 후)

```
isaac_perception/
└─ dataset/
   ├─ images/
   │  ├─ train/  *.png  (~1200 장)
   │  └─ val/    *.png  (~300 장)
   ├─ labels/
   │  ├─ train/  *.txt
   │  └─ val/    *.txt
   └─ dataset.yaml
```

`dataset.yaml` 예시:
```yaml
path: /home/rokey/dev_ws/rover_ws/src/a2_isaac/isaac_perception/dataset
train: images/train
val:   images/val
nc: 3
names:
  - blue_mineral
  - green_gas
  - yellow_mineral
```

---

## 3. Phase 2 — Auto-annotation 검증

수집 직후 검증 단계:

1. 무작위 10장 골라 `cv2.rectangle` 로 bbox 오버레이 → 시각 확인
2. 잘못 짜인 bbox (out-of-frame, 너무 작음 < 16px) 자동 필터
3. minerals 가 차폐돼서 visible 영역 < 30% 인 경우 label 제외

스크립트: `isaac_perception/scripts/verify_dataset.py`

---

## 4. Phase 3 — YOLO 학습

### 4.1 의존성

```bash
pip install ultralytics opencv-python  # (isaac-python 환경 또는 별도 venv)
```

### 4.2 학습 스크립트 (`isaac_perception/scripts/train_yolo.py`)

```python
from ultralytics import YOLO

model = YOLO("yolov8n.pt")   # 또는 yolov8s.pt (정확도↑ 속도↓)
results = model.train(
    data="dataset/dataset.yaml",
    epochs=100,
    imgsz=640,
    batch=16,
    name="mineral_v1",
    patience=20,
)
# best 자동 저장: runs/detect/mineral_v1/weights/best.pt
```

### 4.3 학습 종료 후

```bash
cp runs/detect/mineral_v1/weights/best.pt isaac_perception/models/mineral_yolo_best.pt
```

`models/` 폴더에 commit (gitignore 안 됨 — `.pt` 추가 허용).

### 4.4 평가

- `mAP@0.5` ≥ 0.85 목표 (synthetic 데이터라 비교적 쉬움)
- 각 클래스별 confusion matrix 확인 (특히 green_gas vs others)

---

## 5. Phase 4 — 추론 통합 (`isaac_perception` 패키지)

### 5.1 새 detector 모듈

`isaac_perception/isaac_perception/yolo_mineral_detector.py`:

```python
from ultralytics import YOLO
import cv2
import numpy as np
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class Detection:
    cls_id: int           # 0, 1, 2
    cls_name: str         # "blue_mineral" 등
    conf: float
    bbox: tuple           # (x1, y1, x2, y2) px
    cx: float
    cy: float


class YoloMineralDetector:
    def __init__(self, model_path: str, conf_threshold: float = 0.5):
        self.model = YOLO(model_path)
        self.conf = conf_threshold
        self.names = ["blue_mineral", "green_gas", "yellow_mineral"]

    def detect(self, bgr: np.ndarray) -> List[Detection]:
        results = self.model(bgr, conf=self.conf, verbose=False)
        dets = []
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                dets.append(Detection(
                    cls_id=cls_id,
                    cls_name=self.names[cls_id],
                    conf=conf,
                    bbox=(x1, y1, x2, y2),
                    cx=(x1 + x2) / 2,
                    cy=(y1 + y2) / 2,
                ))
        return dets
```

### 5.2 기존 `vision_tracker_cyan.py` 와의 관계

- 기존 HSV-tracker: 단일 cyan cube 만 (pickplace 데모용)
- 새 YOLO detector: 3종 mineral 동시 detect
- 둘 다 유지. Use-case 별로 선택.

### 5.3 Pickplace 통합

`pickplace_visual_rover.py` 에서:
```python
# 기존: tracker = CyanCubeTracker()
# 변경 (선택): tracker = YoloMineralDetector("models/mineral_yolo_best.pt")
#              detections = tracker.detect(bgr)
#              # 가장 가까운 mineral 선택, picking_position 으로 사용
```

---

## 6. Phase 5 — 검증 시나리오

1. **정적 영상**: terrain_00022 에서 rover 위치 랜덤 100개 → navigation cam 캡쳐 → 추론 → mAP 측정
2. **실시간 주행 시뮬**: rover 가 terrain 안을 주행 → **`/Vehicle/Vehicle/rover/Body/Camera`** 라이브 추론 → 검출 마커 시각화. 차량이 가는 길에 보이는 광물 실시간 탐지.
3. **(확장) depth 활용**: navigation camera 의 depth 로 검출된 mineral 의 3D world 위치 추정 → planner 가 그 좌표로 주행
4. **점수 통합** (확장): `value_scorer.py` (기존 stub) 에서 클래스 별 점수 부여
   - blue_mineral: 10pt
   - green_gas: 25pt
   - yellow_mineral: 50pt

---

## 7. 마일스톤 / 체크리스트

- [ ] **M1**: `collect_yolo_data.py` 작성, 50장 sample 수집해 시각 검증
- [ ] **M2**: 1500장 + auto-annotate, `dataset.yaml` 작성
- [ ] **M3**: YOLOv8n 학습 → best.pt 생성, mAP@0.5 ≥ 0.85
- [ ] **M4**: `yolo_mineral_detector.py` 작성, 단일 이미지 추론 검증
- [ ] **M5**: pickplace 파이프라인에 YOLO 통합 (optional)
- [ ] **M6**: rover wrist camera 라이브 추론 (frame rate ≥ 10 fps)

---

## 8. 위험 요소 / 미해결 이슈

| 위험 | 대응 |
|---|---|
| Mineral 이 rock 에 가려 visible 영역 작음 | min_visible_area 필터로 학습 데이터 제외 |
| 조명 변화 부족 → real-world 일반화 안 됨 | Replicator 의 도메인 랜덤화 (조명, 색온도, fog) |
| `red_*` prim 이지만 green 색상 → 라벨링 혼동 | 클래스 매핑 헬퍼에서 prim name → class id 명시. **이 문서 0번 표 따름** |
| **Blue/yellow 가 cube 아닌 결정 클러스터** → 고정 cube bbox 부정확 | `UsdGeom.BBoxCache.ComputeWorldBound()` 로 mesh 의 실제 world bbox 8 corner 사용 |
| Synthetic 만 학습 → sim-to-real gap | 본 프로젝트는 simulation only 라 큰 이슈 없음 |
| Mineral 이 공중에 떠있음 (이전 PhysX 이슈) | 데이터 수집 전 settle 시킨 USD 사용 |
| Settle 안 됐는데 캡쳐 → bbox 어긋남 | §2.3 velocity-based settle 감지 필수 적용. timeout 시 episode skip |
| Episode 간 mineral 분포 너무 비슷 | USD load 마다 진짜 다른 random seed 가 적용되는지 검증 (sample 10 episodes 의 분포 비교) |
| Episode 길이 부족 → 학습 데이터 적음 | N_EPISODES × VIEWS_PER_EPISODE = 100×15 = 1500 기본. 필요시 더 증가 |

---

## 9. 파일 위치 요약 (구현 후 모양)

```
isaac_perception/
├─ YOLO_MINERAL_DETECTION_PLAN.md          ← 이 문서
├─ isaac_perception/
│  └─ yolo_mineral_detector.py             ← 추론 wrapper
├─ scripts/
│  ├─ collect_yolo_data.py                 ← Phase 1
│  ├─ verify_dataset.py                    ← Phase 2
│  └─ train_yolo.py                        ← Phase 3
├─ dataset/
│  ├─ dataset.yaml
│  ├─ images/{train,val}/
│  └─ labels/{train,val}/
└─ models/
   └─ mineral_yolo_best.pt                 ← 최종 weights
```

---

## 10. 참고 자료

- 3일차 강의 `1장/슬라이드8-9/camera.py` — Isaac Sim Camera 기본 사용
- 3일차 강의 `1장/슬라이드21-23/pinhole.py` — pinhole intrinsics
- 4일차 강의 `m0609_vision/vision_tracker.py` — HSV detector 패턴 (Detection dataclass 참고)
- Ultralytics YOLOv8 docs: <https://docs.ultralytics.com/>
- Isaac Sim Replicator: <https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html>
