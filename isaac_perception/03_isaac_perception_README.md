# isaac_perception

> **트랙 owner**: 최진우 (T2 — Perception + M0609 Manipulation Vision)
> **책임**: Mineral 인식 (YOLO v8) + Wrist cam visual servoing + Pick & Place 통합 데모
> **상태**: PoC 완성 — `rover_yolo_demo.py` 가 end-to-end 동작 (autopilot → pick → place)

---

## 1. 패키지 개요

Isaac Sim 카메라(RGB + Depth)에서 Mars 광물 3종을 YOLO v8 로 탐지하고, M0609 + RG2-FT 그리퍼로 직접 pick & place 까지 수행하는 통합 perception 패키지.

초기 stub 단계(HSV 단순 색기반)에서 **YOLOv8 학습 + 수동 데이터셋 캡쳐 파이프라인 + Pick&Place 통합 데모**로 확장됨.

### 핵심 기여

- **End-to-end demo** (`rover_yolo_demo.py`, 1744 lines) — Mars terrain 자동 주행 + nav cam YOLO 탐지 + wrist cam visual servoing + M0609 pick + RearBasket 적재
- **YOLO 데이터셋 파이프라인** — 수동 캡쳐 (terrain 위 mineral spawn) → MakeSense 라벨링 → train/val split → ultralytics 학습 → best.pt 배포
- **3-class mineral detector** (`models/mineral_yolo_best.pt`, 5.5 MB, YOLOv8n) — blue/yellow_mineral + green_gas
- **재사용 가능한 라이브러리**: `YoloMineralDetector` (best.pt wrapper), `CyanDetector` (HSV fallback), `ManualM0609Driver` (kinematic-only 환경용 FK driver)
- **ROS2 모킹 노드** — `perception_node` (mock publisher, real perception 으로 swap 가능한 인터페이스)

---

## 2. 폴더 구조 (현재 상태)

```text
isaac_perception/
├─ 03_isaac_perception_README.md         📄 이 문서
├─ YOLO_MINERAL_DETECTION_PLAN.md        📋 학습 계획 (Phase 1~4 상세)
│
├─ scripts/                              🚀 실행 가능한 도구들
│  ├─ rover_yolo_demo.py                ⭐ 메인 통합 데모 (autopilot + pick&place)
│  ├─ manual_capture.py                 📷 학습 데이터 수동 캡쳐 (terrain + mineral 3 색)
│  ├─ negative_capture.py               📷 negative sample 캡쳐 (mineral 없음)
│  ├─ build_yolo_dataset.py             🧱 dataset/manual/ → dataset/yolo/{train,val}
│  ├─ verify_dataset.py                 ✅ bbox annotation 시각 검증 (격자 preview)
│  ├─ render_labels.py                  ✅ 개별 .txt 라벨 → bbox 그림 preview
│  ├─ train_yolo.py                     🧠 ultralytics YOLOv8 학습 wrapper
│  ├─ detect_image.py                   🔍 best.pt 로 단일 이미지/폴더 추론
│  └─ m0609_kinematic_driver.py         🤖 PhysX kinematic 환경용 M0609 FK driver
│
├─ isaac_perception/                     📦 ROS2 / 재사용 라이브러리
│  ├─ __init__.py
│  ├─ perception_node.py                 🟡 mock ROS2 publisher (PerceptionResult)
│  ├─ yolo_mineral_detector.py           ✅ YoloMineralDetector wrapper (best.pt 추론)
│  ├─ cyan_detector.py                   ✅ CyanDetector — HSV blob fallback
│  ├─ vision/                            ⏳ stub (TODO comments only)
│  │  ├─ mineral_detector.py             ⏳
│  │  ├─ obstacle_detector.py            ⏳
│  │  ├─ terrain_analyzer.py             ⏳
│  │  └─ value_scorer.py                 ⏳ (빈 파일)
│  ├─ depth/
│  │  └─ depth_estimator.py              ⏳ stub
│  └─ lidar/                             ⏳ 미구현 (확장 영역)
│
├─ dataset/                              📊 학습 데이터
│  ├─ manual/                            📷 raw + .txt 라벨 (MakeSense workflow)
│  │  ├─ blue/      *.png + *.txt  (217 장)
│  │  ├─ yellow/    *.png + *.txt  (211 장)
│  │  ├─ green/     *.png + *.txt  (213 장)
│  │  ├─ negative/  *.png + 빈 .txt (104 장)
│  │  └─ _preview/                      🖼 render_labels.py 가 생성하는 bbox 미리보기
│  └─ yolo/                              🧱 ultralytics 학습용 (build_yolo_dataset.py 가 생성)
│     ├─ data.yaml                        # path / train / val / nc=3 / names
│     ├─ classes.txt                       # blue_mineral / yellow_mineral / green_gas
│     ├─ train/{images,labels}/  (316 장)
│     └─ val/{images,labels}/    (55 장)
│
├─ models/                               💾 학습된 weights
│  ├─ mineral_yolo_best.pt               ✅ 실사용 weights (5.5 MB, YOLOv8n, 3 class)
│  └─ mineral_detector.pt                ⏸ 옛 placeholder (1 byte)
│
├─ runs/                                 📝 학습/추론 출력
│  ├─ detect/                            (ultralytics 학습 로그 — 비어있을 수 있음)
│  └─ mineral/demo_shots/                 # rover_yolo_demo 의 P 키 스냅샷 저장
│
├─ resource/isaac_perception             ament resource marker
├─ setup.py / setup.cfg                  ROS2 / colcon 빌드 설정
```

---

## 3. Mineral 클래스 (3-class)

| YOLO class id | name | 실제 시각 | mesh 형태 | USD prim prefix |
|:-:|---|---|---|---|
| **0** | `blue_mineral`   | 밝은 cyan/teal | 수정 결정 클러스터 (비정형 polytope)  | `blue_*`   |
| **1** | `yellow_mineral` | 밝은 yellow    | spike 모양 수정 결정 클러스터         | `yellow_*` |
| **2** | `green_gas`      | 진녹색         | 정육면체 박스 (gas container)         | `red_*` ⚠ |

> ⚠️ USD prim 이름과 시각 클래스가 다름 — prim `red_*` 의 실제 mesh 는 **green gas box**. 라벨링은 시각 기준.
> 자세한 내용: [`YOLO_MINERAL_DETECTION_PLAN.md`](./YOLO_MINERAL_DETECTION_PLAN.md) §0

**dataset/yolo/data.yaml** 의 `names:` 순서 = `[blue_mineral, yellow_mineral, green_gas]`

> ⚠️ `cyan_detector.py` 와 `verify_dataset.py` 에 클래스 순서가 `[blue, green_gas, yellow]` 으로 잡혀있는 한쪽이 있음 — `data.yaml` 이 ground truth.

---

## 4. ⭐ 메인 데모 — `scripts/rover_yolo_demo.py`

terrain_00022 + Vehicle v2 + 학습 best.pt 로 실시간 광물 탐지 + 자동 주행 + M0609 pick & place + RearBasket release 까지 한 번에.

### 4.1 실행

```bash
# 기본 (best.pt + conf 0.5)
isaac-python scripts/rover_yolo_demo.py

# 임의 설정
isaac-python scripts/rover_yolo_demo.py --conf 0.3 --interval 2
```

| 인자 | 기본값 | 의미 |
|---|---|---|
| `--model` | `models/mineral_yolo_best.pt` | YOLO weights |
| `--conf`  | 0.5  | confidence threshold |
| `--iou`   | 0.45 | NMS IoU |
| `--interval` | 2 | N step 마다 inference (큰 값 → 부하 ↓) |
| `--resolution` | `1280x720` | nav + wrist 카메라 해상도 |
| `--out` | `runs/mineral/demo_shots` | P 키 스냅샷 저장 폴더 |

### 4.2 키 조작

| Key | 동작 |
|---|---|
| `W` `S` | 전진 / 후진 (manual override) |
| `A` `D` | 좌/우 회전 |
| `Space` | 정지 |
| `T` | autopilot 토글 (default ON) |
| `M` | manipulation 강제 abort → autopilot 복귀 |
| `P` | 현재 nav cam view + bbox screenshot 저장 |
| `ESC` | 종료 |

### 4.3 동작 흐름

```text
AUTOPILOT (nav cam YOLO)
   ↓ 가장 가까운 mineral 추적, 화면 중앙 정렬 (P-control steering)
ENGAGE_DISTANCE (0.9 m) 진입 시 creep 속도로 감속
   ↓ 0.25 m/s 로 살금살금
mineral 별 STOP_DISTANCE 도달 (blue=0.25m / yellow=0.75m / green=0.75m)
   ↓
MANIPULATION 진입 (rover 위치 freeze)
   ├─ HOME_PRE        : M0609 HOME 자세 + gripper 110mm open
   ├─ WRIST_SERVO     : wrist cam YOLO 로 mineral XY 보정 (30 frame 안)
   ├─ APPROACH_DESCEND: TCP 가 mineral 위 4cm 에 hover (한 번에)
   ├─ GRASP_CLOSE     : finger 닫음 (0.6 rad) + FixedJoint 부착
   ├─ ATTACH_LIFT     : TCP 가 mineral 위 45cm 로 lift
   ├─ JS_PRE          : joint-space dump trajectory (3 waypoint)
   ├─ RELEASE         : FixedJoint 제거 + mineral invisible (basket 수납)
   └─ JS_POST         : HOME 복귀 (2 waypoint)
   ↓
autopilot 재개 → 다음 mineral 탐색
```

### 4.4 카메라 3종 동시 사용

| 카메라 | path (vehicle_v2.usd 내) | 용도 |
|---|---|---|
| **Top cam** | `/World/_TopCam` (런타임 생성) | 메인 viewport — 탑뷰 |
| **Nav cam** | `Vehicle/rover/Body/Camera` | autopilot 주행 + 원거리 mineral 탐지 |
| **Wrist cam** | `Vehicle/onrobot_rg2ft/angle_bracket/realsense_d455/RSD455/Camera_OmniVision_OV9782_Color` | 근거리 visual servoing (TCP 위치 보정) |

Nav / Wrist 각각 별도 omni.ui window 에 bbox overlay 로 표시.

### 4.5 주요 상수 (튜닝 포인트)

| 상수 | 값 | 의미 |
|---|---|---|
| `ENGAGE_DISTANCE` | 0.9 m | creep 진입 거리 |
| `STOP_DISTANCE_PER_CLASS` | blue:0.25 / yellow:0.75 / green:0.75 | mineral 별 정지 거리 |
| `AUTO_LIN_SPEED` / `CREEP_LIN_SPEED` | 1.0 / 0.25 m/s | autopilot 정상 / creep 속도 |
| `STEER_GAIN` | 1.2 | nav cam 중앙 정렬 P 게인 |
| `HOVER_ABOVE_MINERAL` | 0.04 | TCP 가 mineral 위 hover 높이 (m) |
| `LIFT_HEIGHT` | 0.45 | grasp 후 TCP lift 높이 (m) |
| `GRIPPER_OPEN_RAD` / `CLOSED_RAD` | [0,0] / [0.6,0.6] | finger_joint 각도 (0=110mm open, 1.18=완전 닫힘) |
| `WAYPOINT_INTERP_FRAMES` | 120 | JS waypoint 간 선형 보간 frame (≈2초, 시각 관찰 가능) |

### 4.6 TCP (Tool Center Point) 자동 보정

IK 는 link_6 를 target 에 놓지만 실제 grip 은 finger midpoint 에서 일어남. 런타임 시작 시:

```python
# HOME 자세에서 한 번 캡쳐
tcp_world = (right_inner_finger.world + left_inner_finger.world) / 2
TCP_OFFSET_LOCAL = link6_rot.T @ (tcp_world - link6_pos)
```

이후 모든 `_ik_to(target_tcp_world)` 호출은 자동으로 `link6_target = target - link6_rot @ TCP_OFFSET_LOCAL` 보정.
결과: 한 번 측정 → blue/yellow/green 모두 동일 보정 적용. 일반적으로 `(ΔX≈-15mm, ΔY≈-23mm, ΔZ≈-136mm)` 정도.

### 4.7 환경 가정

- **terrain**: `isaac_sim/worlds/terrain_00022.usd` (내장 mineral prim 들 — `_scan_terrain_minerals()` 가 ground-truth XY 매칭 후 wrist cam refinement)
- **vehicle**: `isaac_sim/assets/vehicle/vehicle_v2.usd` — rover + M0609 + RG2-FT + D455 단일 articulation
- **Mars gravity**: PhysicsScene g = 3.72 m/s²
- **Anchor**: rover Body 를 `RoverAnchor` FixedJoint 로 world 에 고정 (terrain collision 없음 → settle 불가 회피)
- **휠 freeze**: 모든 rover/m0609 외 joint 에 stiffness 1e7 lock (정적 자세 유지)

---

## 5. 데이터셋 파이프라인

```
1. manual_capture.py / negative_capture.py
       ↓  PNG 저장 → dataset/manual/{blue,yellow,green,negative}/
2. (외부) MakeSense.ai / Roboflow / labelImg 로 .txt 라벨링
       ↓  같은 폴더에 *.txt (YOLO format)
3. render_labels.py (선택)
       ↓  dataset/manual/_preview/<color>/*.png 로 bbox 검증
4. build_yolo_dataset.py
       ↓  dataset/yolo/{train,val}/{images,labels} 생성 + data.yaml
5. verify_dataset.py
       ↓  16장 격자 preview 로 bbox 시각 검증
6. train_yolo.py
       ↓  ultralytics YOLOv8 학습 → runs/detect/<name>/weights/best.pt
       ↓  자동 복사 → models/mineral_yolo_best.pt
7. detect_image.py 또는 rover_yolo_demo.py
       ↓  best.pt 로 추론
```

### 5.1 manual_capture.py

- Scene: `terrain_00022.usd` + 3색 mineral 고정 spawn `(4.5,±1,1.0)`
- viewport active camera = `/World/DataCam` (Vehicle.usd 의 intrinsics 복사)
- `Q` 키로 현재 view → `manual_XXXX.png` 저장 (자동 번호)
- WASD / 마우스 우클릭 / 휠로 viewport 카메라 조작

### 5.2 negative_capture.py

- Scene: terrain 만 (mineral 없음)
- `Q` 키 → `negative_XXXX.png` + 빈 `.txt` 동시 저장 (negative sample)
- false positive 줄이는 용도

### 5.3 build_yolo_dataset.py

```bash
python3 scripts/build_yolo_dataset.py                    # 기본 85/15 split, symlink
python3 scripts/build_yolo_dataset.py --val-ratio 0.2
python3 scripts/build_yolo_dataset.py --copy             # symlink 대신 실제 복사 (export 용)
```

`manual/{blue,green,yellow,negative}/*.png + *.txt` → `yolo/{train,val}/{images,labels}/` + `data.yaml` 생성.
랜덤 split (seed=42, deterministic). 클래스별 bbox 통계도 출력.

### 5.4 verify_dataset.py

```bash
python3 scripts/verify_dataset.py                       # 16장 무작위, cv2.imshow
python3 scripts/verify_dataset.py --n 25 --split val
python3 scripts/verify_dataset.py --output /tmp/v.png   # imshow 없을 때 PNG 저장
```

격자 (cell 320×240) 에 bbox 그려서 시각 확인. 헤드리스 환경이면 `--output`.

### 5.5 train_yolo.py

```bash
pip install ultralytics  # 필수
python3 scripts/train_yolo.py                                    # yolov8n.pt, 100ep, imgsz=640
python3 scripts/train_yolo.py --model yolov8s.pt --epochs 200
python3 scripts/train_yolo.py --resume runs/detect/mineral_v1
```

옵션: `--model`, `--epochs`, `--imgsz`, `--batch`, `--name`, `--patience`, `--device`, `--no-copy`.
종료 시 `runs/detect/<name>/weights/best.pt` → `models/mineral_yolo_best.pt` 자동 복사.

> ⚠️ `train_yolo.py` 가 참조하는 기본 dataset path 는 `dataset/dataset.yaml` 인데 실제 `build_yolo_dataset.py` 는 `dataset/yolo/data.yaml` 을 만듦. 사용 시 `--data dataset/yolo/data.yaml` 명시 권장.

### 5.6 detect_image.py

```bash
python3 scripts/detect_image.py path/to/img.png                     # cv2.imshow
python3 scripts/detect_image.py path/to/folder/ --output /tmp/out/  # 폴더 일괄
python3 scripts/detect_image.py img.png --model models/x.pt --conf 0.3
```

`YoloMineralDetector` 사용 → bbox overlay PNG 저장 또는 표시.

---

## 6. 라이브러리 (`isaac_perception/` 패키지)

### 6.1 `YoloMineralDetector` (`yolo_mineral_detector.py`)

학습된 best.pt 를 wrap. 다른 모듈/스크립트에서 재사용.

```python
from yolo_mineral_detector import YoloMineralDetector
det = YoloMineralDetector("models/mineral_yolo_best.pt", conf=0.5, iou=0.45)
dets = det.detect(bgr_image)  # List[Detection]
for d in dets:
    print(d.cls_name, d.conf, d.bbox, d.cx, d.cy)
vis = YoloMineralDetector.draw_overlay(bgr, dets)
```

`Detection` dataclass: `cls_id, cls_name, conf, bbox=(x1,y1,x2,y2), cx, cy`.

### 6.2 `CyanDetector` (`cyan_detector.py`)

HSV 기반 cyan blob centroid detector — YOLO fallback / debug 용.
`dual_cam_pick_place/cyan_tracker.py` 패턴 포팅.

```python
from cyan_detector import CyanDetector
cd = CyanDetector(hsv_lower=(80,100,80), hsv_upper=(100,255,255),
                  min_area=200, morph_kernel=5)
det = cd.detect(bgr)
if det.found:
    print(det.cx, det.cy, det.area, det.bbox)
```

3D pose 추정 없음 (호출자 책임).

### 6.3 `perception_node.py` — ROS2 mock publisher

```python
ros2 run isaac_perception perception_node
```

`/perception_result` 토픽에 `isaac_interfaces/PerceptionResult` 메시지를 주기적으로 publish.
**현재는 mock** (parameter 로 고정값 발행). `_publish_mock_detection` 을 실제 perception 로 swap.

Parameters:
- `mock_detection_enabled` (bool, default True)
- `mock_period_sec` (float, default 8.0)
- `mock_object_id`, `mock_object_type`, `mock_x/y/z`, `mock_confidence`

### 6.4 `ManualM0609Driver` (`scripts/m0609_kinematic_driver.py`)

**PhysX kinematic 환경 전용** M0609 + RG2-FT FK driver.

PhysX 는 `ArticulationRootAPI` 가 적용된 prim 이 kinematic 일 때 articulation 을 무효화함. teleport 식 navigation 을 하는 데모에서 `SingleArticulation` 사용 불가 → USD 의 joint axis / pivot 을 직접 파싱해 FK 로 link world xform 을 set.

`SingleArticulation` 의 부분 인터페이스 (`num_dof`, `dof_names`, `body_names`, `get_joint_positions`, `set_joint_positions`, `get_jacobians`, `get_articulation_controller`) 를 mimic 해서 기존 IK 코드 재사용 가능.

> 현재 `rover_yolo_demo.py` 는 dynamic articulation 으로 동작하므로 이 driver 는 미사용. 다른 kinematic-only 데모용으로 보존.

### 6.5 Stub 파일들

- `vision/mineral_detector.py`, `obstacle_detector.py`, `terrain_analyzer.py`, `value_scorer.py`
- `depth/depth_estimator.py`
- `lidar/` (빈 폴더)

전부 TODO 주석만 있는 placeholder. 실제 동작은 `yolo_mineral_detector.py` (vision/mineral) 와 `rover_yolo_demo.py` 의 `_deproject_pixel_to_world` (depth/3D 추정) 가 대체.

---

## 7. 모델 / 학습 결과물

| 파일 | 크기 | 설명 |
|---|---|---|
| `models/mineral_yolo_best.pt` | 5.5 MB | 실사용 YOLOv8n weights (3 class) |
| `models/mineral_detector.pt` | 1 byte | 옛 placeholder, 무시 |

### 데이터셋 현황

| 폴더 | 장수 | 비고 |
|---|---:|---|
| `dataset/manual/blue/`     | 217 | 라벨 포함 |
| `dataset/manual/yellow/`   | 211 | 라벨 포함 |
| `dataset/manual/green/`    | 213 | 라벨 포함 |
| `dataset/manual/negative/` | 104 | 빈 .txt (false positive 억제) |
| `dataset/yolo/train/`      | 316 | build 후 (85% split) |
| `dataset/yolo/val/`        | 55  | build 후 (15% split) |

---

## 8. 실행 환경 & 의존성

### 8.1 Isaac Sim Python (isaac-python alias)

`~/.bashrc` 에 등록된 alias:
```bash
alias isaac-python="/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh"
```

대부분의 `scripts/` 는 Isaac Sim Python 으로 실행 (SimulationApp 필요).

### 8.2 시스템 Python (rclpy / colcon)

- `perception_node.py` — `rclpy`, `isaac_interfaces.msg.PerceptionResult`
- `train_yolo.py`, `detect_image.py`, `build_yolo_dataset.py`, `verify_dataset.py`, `render_labels.py` — 시스템 python3 + `ultralytics`, `cv2`, `numpy`

### 8.3 패키지 (ROS2 ament)

```python
# setup.py
entry_points = {"console_scripts": ["perception_node = isaac_perception.perception_node:main"]}
```

빌드: `colcon build --packages-select isaac_perception` (워크스페이스 루트에서).
의존: `isaac_interfaces` (PerceptionResult msg 정의), `rclpy`.

---

## 9. 인접 패키지와의 관계

| 패키지 | 인터페이스 |
|---|---|
| `isaac_sim` | terrain / vehicle / mineral USD 자산, 카메라 prim 경로 |
| `isaac_manipulation` | `pickplace_visual_rover.py` (4일차 reference), `camera_viewer.py` (별도 cv2 윈도우), `realsense_mount.py` (D455 attach) |
| `isaac_interfaces` | `PerceptionResult.msg` (mineral_detected/obstacle/terrain 필드) |

`rover_yolo_demo.py` 는 perception + manipulation 통합본이라 위 패키지들의 자산/스크립트를 모두 참조.

---

## 10. 알려진 한계 / TODO

- **ROS2 통합 미완성** — `rover_yolo_demo.py` 는 ROS2 토픽 publish 안 함. perception_node 는 mock. 둘을 연결해야 다른 노드가 결과 사용 가능.
- **vision/depth stubs 비어있음** — 실 perception 으로 채우려면 별도 작업.
- **wrist cam 의존 단일점** — wrist cam 이 mineral 못 잡으면 nav cam XYZ 로 fallback (정확도 ↓).
- **mineral hovering 높이** — `HOVER_ABOVE_MINERAL = 0.04` 는 sphere radius 5cm 가정. 다른 모양 mineral (cube) 은 추가 튜닝 필요.
- **YOLO 학습 결과 metric 미기록** — `runs/detect/` 가 비어있음. 재학습 시 mAP@50, P, R 기록 필요.
- **train_yolo.py 의 기본 dataset path 불일치** — `dataset/dataset.yaml` 아니라 `dataset/yolo/data.yaml` 이 맞음 (사용 시 `--data` 명시).

---

## 11. 한 줄 요약

> Isaac Sim 카메라(nav + wrist D455) 의 RGB+Depth 로 학습된 YOLOv8 가 mineral 3종을 실시간 탐지하고, M0609 + RG2-FT 가 wrist cam 기반 visual servoing 으로 TCP 보정된 정확한 위치에서 pick → RearBasket place 까지 자동 수행하는 통합 perception 패키지.

---

## 12. 변경 히스토리 (2026-05-24)

`rover_yolo_demo.py` 한 파일에 누적된 안정화/튜닝 작업. 카테고리별 정리.

### 12.1 grasp 방식 결정 — friction grasp 시도 → magic grasp 표준 채택

T2 트랙의 다른 데모들 (`pickplace_visual_rover.py`, `m0609_rover.py`, `m0609_dual_cam.py`, `m0609_pick_place_aruco*.py`) 와 동일하게 **`UsdPhysics.FixedJoint` 강제 부착** (magic grasp) 방식 유지.

- `m0609_pick_place_fixed_target.py` 의 `ParallelGripper` + `PickPlaceController` 표준 friction grasp 패턴을 시도
  - PhysicsMaterial 마찰력 강화 (mineral 1.2/1.0, finger 1.8/1.4, 이후 3.0/2.5, 3.5/3.0)
  - `_attach_object_to_link` / `_detach_grip_joint` / `_hide_prim` 호출 제거
  - cube scale, gripper closed rad, settle frames 등 종합 조합
- **결과: M0609 의 down-reach 한계로 finger 가 cube 옆면이 아닌 위 표면 모서리에서 닫혀 friction grasp 항상 실패**
  - `[grasp diag]` 진단 결과 Δz=+59~+87mm 일관됨 (rover/arm base 위치 → cube z 까지 IK 자연 도달 한계)
  - arm base z ≈ 0.96, cube z = 1.05 → arm 이 위로 reach 하면서 동시에 옆 60cm 도달은 reach 한계
- magic grasp 으로 복귀. PhysicsMaterial 자체는 그대로 둠 (부착 시점 contact 안정성 미세 향상)

### 12.2 IK reach 개선 — `IK_JOINT_LIMITS_DEG` 확장 + nullspace bias 약화

이전 IK 가 M0609 의 보수적 joint limit 과 강한 home 자세 nullspace bias 때문에 APPROACH_DESCEND 가 `marginal timeout` (800 step ≈ 13초) 으로 통과 → cycle 시간 폭증.

```python
# 변경 전
IK_NULL_GAIN_PER_JOINT = np.array([0.0, 0.1, 0.1, 1.5, 1.5, 1.5])
IK_JOINT_LIMITS_DEG = [(-120,120), (-60,120), (-30,180), (-120,120), (-10,170), (-120,120)]

# 변경 후 (M0609 hardware spec 으로 확장 + nullspace 7배 약화)
IK_NULL_GAIN_PER_JOINT = np.array([0.0, 0.05, 0.05, 0.2, 0.2, 0.2])
IK_JOINT_LIMITS_DEG = [(-360,360), (-125,125), (-150,150),
                       (-360,360), (-135,135), (-360,360)]
```

- `joint_5` (wrist pitch) 범위 (-10, 170) → (-135, 135) — wrist down 자세 자유롭게
- nullspace gain 7배 약화 — arm 이 home 자세로 강하게 끌려가는 힘 풀어 down-reach 가능

**효과**: `APPROACH_DESCEND → GRASP_CLOSE` 가 `marginal` 아닌 **OK** 로 한 번에 빠르게 통과. 전체 cycle 시간 50%+ 단축.

### 12.3 autopilot 접근 로직 — engage push + snapshot

기존 creep 접근 (느린 직진 + steering) 을 hard push 로 변경.

```python
ENGAGE_DISTANCE = 0.9        # m 이내 진입 시 push 모드 시작
ENGAGE_X_PUSH_PER_CLASS = {  # 1초 동안 forward 축으로 부드럽게 누적 이동
    "blue_mineral":   0.5,   # rover 가 cube 와 0.4m 까지 접근 (M0609 reach 안)
    "yellow_mineral": 0.5,
    "green_gas":      0.25,  # green 은 push 짧게
}
ENGAGE_PUSH_FRAMES = 60      # 60 frame (≈1초) 에 걸쳐 분할 적용 → 부드러운 이동
```

push 시작 시점에 **target snapshot 캡쳐** (`engage_push_target`) — wrist cam 도달 시점 conf drop 으로 detection 빠져도 deproject 가능:

```python
engage_push_target[0] = {
    'name': target.get("name"),
    'cx': target["cx"],
    'cy': target["cy"],
    'dist': target["dist"],
    'cam_pos': snap_cam_pos.copy(),  # push 시작 시점 카메라 pose 저장
    'cam_rot': snap_cam_rot.copy(),
}
```

push 완료 후 `_start_manipulation_if_possible(snapshot=...)` 호출 — det_summary 비어 있어도 snapshot 으로 deproject 진행. 이전 "conf 0.5 미만 → manip FAIL" 문제 해결.

### 12.4 mineral 좌표 정확도 — Cube sub-prim 우선 추출

`_scan_terrain_minerals()` 가 top-level prim 의 transform 대신 **`Cube` sub-prim 의 world XYZ 를 우선 추출**.

```python
for sub in Usd.PrimRange(child):
    if sub.GetName().lower() == "cube":
        cube_xyz = _read_world_xyz(str(sub.GetPath()))
        break
```

terrain_00022.usd 의 mineral prim 구조:
```
/World/Terrain/Minerals/blue_0001/Reference/Meshes/Sketchfab_model/...  ← visible mesh
/World/Terrain/Minerals/blue_0001/Reference/Cube                          ← collision/grasp 대상
```

cube 와 top-level prim 의 XYZ 차이 진단 print 도 추가:
```
[cube-offset] red_0002  top→cube ΔXYZ = (-4, +7, +49) mm
```

**특수 케이스 — green_gas**: cube 가 visible mesh 보다 Z 방향 +50mm offset → finger 가 cube 위치로 접근하면 visible mesh 가 finger 아래로 떨어진 모양 (둥둥). 해결:
- `_scan_terrain_minerals`: green_gas 의 경우 **XY = top-level (visible mesh 시각 중심), Z = cube (collision 정확)** 혼합 사용
- `WRIST_SERVO`: green_gas 만 wrist refresh skip (wrist deproject 부정확 회피)

### 12.5 mineral / finger PhysicsMaterial 추가

friction grasp 시도의 산물이지만 magic grasp 에서도 부착 시점 contact 안정성에 도움. 그대로 유지:

```python
MINERAL_FRICTION = (1.2, 1.0, 0.0)   # static, dynamic, restitution
FINGER_FRICTION  = (1.8, 1.4, 0.0)
```

`_create_physics_material` / `_bind_physics_material_to_subtree` 헬퍼 신설 → build_scene 에서 mineral subtree + finger subtree 의 모든 Mesh prim 에 binding.

### 12.6 invisible Cube sub-prim scale 별도 조정

class 별 두 가지 scale 변수:
- `MINERAL_SCALE_PER_CLASS` — top-level prim 의 전체 scale (visible mesh + cube 모두)
- `CUBE_SCALE_PER_CLASS` — Cube sub-prim 만의 scale (collision/grasp surface 만)

```python
MINERAL_SCALE_PER_CLASS = {
    "blue_mineral":   1.0,
    "yellow_mineral": 1.0,
    "green_gas":      0.5,   # green cube 가 너무 커서 절반 축소
}
CUBE_SCALE_PER_CLASS = {
    "blue_mineral":   1.0,    # original (magic grasp 이라 무관)
    "yellow_mineral": 1.0,
    "green_gas":      1.5,    # green 의 cube-mesh offset 보정용
}
```

### 12.7 hover 높이 — class 별 dict

```python
HOVER_ABOVE_MINERAL  = 0.10   # default fallback
HOVER_ABOVE_MINERAL_PER_CLASS = {
    "blue_mineral":   0.03,
    "yellow_mineral": 0.03,
    "green_gas":      0.10,
}
```

mineral 모양/크기 차이에 따라 finger midpoint 가 cube 위 적절한 위치에 오도록 class 별 분리. APPROACH_DESCEND IK target z = `mineral_xyz[2] + hover_h`.

### 12.8 dump trajectory 미세조정

`PLACE_TRAJ_PRE_DEG[2]` 의 joint_2 / joint_5 조정으로 dump 자세 최적화:

```python
PLACE_TRAJ_PRE_DEG = [
    [  0.0,  0.0, 90.0, 0.0, 90.0, 0.0],   # HOME
    [180.0,  0.0, 90.0, 0.0, 90.0, 0.0],   # joint_1 베이스 뒤
    [180.0, 12.5, 90.0, 0.0, 60.0, 0.0],   # dump 자세 (j2=12.5, j5=60)
]
```

### 12.9 UI — wrist cam 윈도 표시 문제

두 `ui.Window` 가 같은 docking 슬롯에 들어가 wrist cam 윈도가 nav cam 뒤에 숨음. 명시적 위치 + dockPreference 추가:

```python
yolo_window       = ui.Window("YOLO — Nav Cam (rover body)",
                              width=720, height=420,
                              position_x=20,  position_y=40,
                              dockPreference=ui.DockPreference.DISABLED)
yolo_wrist_window = ui.Window("YOLO — Wrist Cam (D455 RGB)",
                              width=720, height=420,
                              position_x=760, position_y=40,
                              dockPreference=ui.DockPreference.DISABLED)
```

### 12.10 진단 print 추가 (재현/디버깅용)

- `_start_manipulation_if_possible` 의 각 실패 경로마다 원인별 print:
  ```
  [manip-start FAIL] sm busy (state=...)
  [manip-start FAIL] no valid det (with finite dist) and no snapshot
  [manip-start FAIL] key (..., ..., ...) already in picked_set
  [match SKIP] closest prim ... 이미 picked_set 에 있음
  [match MISS] closest green_gas dist_xy=...m > 1.5m radius
  [manip-start] using push snapshot: ...                  ← snapshot 경로 진입 시
  ```
- `GRASP_CLOSE` 첫 진입 step 에 `[grasp diag] finger mid vs cube center ΔXYZ` 출력 (위치 부정확 진단용)
- `_scan_terrain_minerals` 의 `[cube-offset]` 출력 (visible mesh ↔ cube prim XYZ 차이 진단)

### 12.11 누적 결과 (2026-05-24 종료 시점)

- **Magic grasp 안정 동작** — blue/yellow/green 모두 pick → dump → release 사이클 일관
- **IK reach 개선** — APPROACH_DESCEND 가 timeout 없이 OK 로 빠르게 통과 (cycle 시간 50%+ 단축)
- **green_gas 위치 정확도** — visible mesh XY + cube Z 혼합 + wrist refresh skip 으로 시각적 정렬
- **autopilot robustness** — engage push snapshot 으로 conf drop 시에도 manipulation 진입 보장

### 12.12 다음 단계 (잠재 후속)

- PhysX `ContactReportAPI` 로 finger ↔ cube 실제 접촉 시점에만 FixedJoint 부착 (반 magic / 반 friction)
- M0609 의 진짜 reach 한계 해결: rover Body 자체를 cube 가까이 들어올리기, 또는 arm 의 link 가상 확장
- `_refresh_from_wrist` 의 Z 보정 — 현재 XY 만. wrist depth median 노이즈 줄이는 방법 추가 시 Z 도 가능
- mineral collision shape — convex hull fallback 대신 native cube collision (`box approximation`) 적용

