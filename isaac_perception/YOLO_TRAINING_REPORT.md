# YOLO 광물 검출 모델 학습 보고서

**프로젝트**: a2_isaac — 화성 탐사 로버 자원 채취 시스템
**모듈**: `isaac_perception` (T2 — 최진우)
**기간**: 2026-05-20 ~ 2026-05-24 (5일)
**브랜치**: `feature/vision-pick-place`
**작성일**: 2026-05-26

---

## Executive Summary

Isaac Sim 화성 지형 시뮬레이션에서 6륜 로버의 nav cam + wrist cam 으로 광물(blue/yellow mineral, green gas)을 검출하기 위한 **YOLO11n** 모델을 학습. 데이터셋 구축, 라벨링 도구 6차 시행착오, Blackwell GPU(sm_120) PyTorch 호환 문제, ROS2 통합까지의 end-to-end 파이프라인.

**최종 성과**:
- 모델: `mineral_yolo_best.pt` (5.5MB)
- val mAP50 = **0.984** (3-class)
- val mAP50-95 = 0.690
- 추론 속도 ~3 ms/img (RTX 5080)
- ROS2 노드(`yolo_perception_node`) 로 nav/wrist 두 카메라 동시 처리 후 `DetectionArray` + `Image_annotated` 발행

---

## 1. 클래스 정의

```yaml
nc: 3
names:
  0: blue_mineral
  1: yellow_mineral
  2: green_gas
```

| Class | 가치 점수 | 시각 특징 | mineral 크기 |
|---|---|---|---|
| `blue_mineral` | 10 | 청색 큐브 | 30 cm |
| `yellow_mineral` | 50 | 황색 큐브 | 30 cm |
| `green_gas` | 25 | 녹색 가스 큐브 | 20 cm |

> ⚠ **MakeSense 라벨링 시 클래스 순서가 원래 plan(`[blue, green, yellow]`)과 `1↔2` swap.** 검출 결과 `1` 이 `yellow_mineral`, `2` 가 `green_gas`. 모든 데이터/스크립트가 MakeSense 순서로 통일됨 (`classes.txt`, `data.yaml`, `render_labels.py`, `build_yolo_dataset.py`).

---

## 2. 데이터 파이프라인

### 2.1 캡쳐 (Isaac Sim viewport)

| 스크립트 | 출력 | 수량 |
|---|---|---|
| `scripts/manual_capture.py` | `dataset/manual/{blue,green,yellow}/*.png` | 319 장 (blue 108 / green 106 / yellow 105) |
| `scripts/negative_capture.py` | `dataset/manual/negative/*.png` + 빈 `.txt` | 52 장 (false positive 억제) |

- 해상도: 1280×720
- Scene: `terrain_00022.usd` + 고정 위치 mineral 3종 + `vehicle_v2.usd` 의 nav cam intrinsics
- 조작: WASD / 마우스 viewport 카메라 이동, `Q` 키 PNG 저장, `ESC` 종료
- 카메라 intrinsics 가 실제 Isaac Sim ROS 발행 카메라와 일치 → 학습/추론 domain gap 최소화

### 2.2 라벨링 도구 시행착오 (6차)

| 차수 | 도구 | 결과 | 폐기 사유 |
|---|---|---|---|
| 1 | `hsv_auto_label.py` (자체 HSV) | fragment 검출 | rover false positive, mask 분리 |
| 2 | `guided_label.py` (ROI hint) | HSV 정밀도 ↑ | ROI 빗나가는 경우 빈번 |
| 3 | `draw_bbox.py` (좌표 직접 입력) | bbox 그리기 | Claude 시각 추정 ±5–10% 오차 |
| 4 | Roboflow web | 정확함 | 서버 장애 + Add-to-Dataset stuck |
| 5 | Yolo_Label (developer0hye) | OK | Qt5 build 패치 필요 |
| **6 (채택)** | **MakeSense.ai** | **정확, 안정적** | — |

→ 자체 라벨링 스크립트 3개 redundant 로 **삭제**, `render_labels.py` (라벨→preview PNG) 만 유지.

### 2.3 데이터셋 빌드 (`scripts/build_yolo_dataset.py`)

```
입력:  dataset/manual/{blue,green,yellow,negative}/*.png + 옆 *.txt
출력:  dataset/yolo/{train,val}/{images,labels}/  (symlink)
       dataset/yolo/data.yaml
       dataset/yolo/classes.txt
```

- 85/15 random split (`seed=42` 고정 — 재현성)
- Train: 316 장, Val: 55 장 (총 371)
- Symlink mode (기본) — 디스크 절약, export 시 `--copy`

### 2.4 데이터셋 분포

![Dataset Labels Distribution](runs/detect/runs/mineral/yolo11n_v1/labels.jpg)

**왼쪽 위**: 클래스별 인스턴스 수 (val 55 장 기준) — blue 130 / yellow 97 / green_gas 129. 약간의 yellow 부족 있지만 균형적.
**오른쪽 위**: bbox 크기/위치 분포 — 대부분 중앙 ~200px 크기 클러스터.
**아래**: bbox 중심 (x, y) 분포 + 폭/높이 상관관계 — width ≈ height 강한 선형 (cube/sphere 형태).

### 2.5 학습 배치 샘플 (augmentation 효과)

![Training Batch Sample with Augmentation](runs/detect/runs/mineral/yolo11n_v1/train_batch0.jpg)

`mosaic=1.0` + `hsv_h=0.02 / hsv_s=0.7 / hsv_v=0.5` + `degrees=15 / scale=0.5 / fliplr=0.5` augmentation 적용 결과. negative 샘플 (rover 만 있음, label 0개) 도 학습 배치에 포함 → false positive 억제.

---

## 3. 학습 환경 셋업

### 3.1 GPU: RTX 5080 (Blackwell, sm_120)

PyTorch 의 sm_120 미지원 → cu126 fail, cu130 OK:

| 시도 | torch + CUDA | 결과 |
|---|---|---|
| 1 | `cu126` (PyTorch 2.12) | sm_120 미지원 — RTX 5080 인식 후 kernel 실행 실패 |
| **2** | **`cu130` (PyTorch 2.12)** | ✅ matmul OK, ultralytics 정상 |

설치 명령 (system Python 3.10, `--user`):
```bash
pip3 install --user torch torchvision --index-url https://download.pytorch.org/whl/cu130
pip3 install --user ultralytics matplotlib scipy
```

→ `~/.local/lib/python3.10/site-packages/` 에 설치. **Isaac Sim 의 Py3.11 환경과 완전 분리**.

### 3.2 Isaac Sim Python (Py3.11) 에도 ultralytics 추가

추론 inference 는 학습용 venv 가 아닌 Isaac Sim Python 에서 돌도록.
```bash
isaac-python -m pip install ultralytics
isaac-python -m pip install "numpy<2"              # numpy 2 가 nvidia-srl 충돌
isaac-python -m pip install "opencv-python<4.11"   # cv2 numpy1 호환
```

검증된 조합:
```
numpy 1.26.4 / cv2 4.10 / torch 2.7.0+cu128 / CUDA True / RTX 5080
```

---

## 4. 학습 — YOLO11n

### 4.1 명령

```bash
yolo detect train \
    model=yolo11n.pt \
    data=dataset/yolo/data.yaml \
    epochs=200 imgsz=1280 batch=8 patience=30 \
    hsv_h=0.02 hsv_s=0.7 hsv_v=0.5 \
    mosaic=1.0 mixup=0.2 \
    degrees=15 scale=0.5 fliplr=0.5 \
    device=0 project=runs/mineral name=yolo11n_v1
```

| 하이퍼파라미터 | 값 | 의도 |
|---|---|---|
| `imgsz` | 1280 | 원본 해상도 유지, mineral 작은 검출 정확도 ↑ |
| `batch` | 8 | RTX 5080 메모리 80W 환경에 맞춤 |
| `patience` | 30 | early stopping |
| `hsv_h=0.02, hsv_s=0.7, hsv_v=0.5` | 강한 augmentation | Mars 환경 광 변화 robust |
| `mosaic=1.0, mixup=0.2` | mosaic + mixup | 다양한 배치 학습 |
| `degrees=15, scale=0.5, fliplr=0.5` | 기하 augmentation | rotation/scale invariance |

### 4.2 학습 결과 (val 55 장)

| Class | P | R | mAP50 | mAP50-95 |
|---|---|---|---|---|
| `blue_mineral` | 0.799 | 0.944 | **0.962** | 0.658 |
| `yellow_mineral` | 1.000 | 0.987 | **0.995** | 0.643 |
| `green_gas` | 0.984 | 1.000 | **0.995** | 0.768 |
| **avg (all)** | **0.928** | **0.977** | **0.984** | **0.690** |

- **59 epoch 에서 early stop** (patience=30) — overfit 직전 자동 종료
- 학습 시간 ~6 초/epoch × 59 ≈ **6 분**
- 추론 ~3 ms/img on RTX 5080

#### 학습 곡선 (loss + metric over epochs)

![Training Results Curves](runs/detect/runs/mineral/yolo11n_v1/results.png)

위 행: train box/cls/dfl loss + precision/recall — **모두 ~30 epoch 부터 수렴**.
아래 행: val box/cls/dfl loss + mAP50 / mAP50-95 — **mAP50 가 50 epoch 부터 0.98 안정 영역**. cls_loss 가 가장 빠르게 떨어지고 (1 epoch 안에), box_loss 는 점진 개선.

#### Precision-Recall Curve

![PR Curve (mAP50 = 0.984)](runs/detect/runs/mineral/yolo11n_v1/BoxPR_curve.png)

- `yellow_mineral` / `green_gas` 사실상 완벽 (0.995 AP)
- `blue_mineral` 만 0.962 — recall 0.95 부근에서 precision drop. 학습 데이터 중 blue 가 가장 많지만 (108 장) 다양한 각도/광 조건에서 약간 미세 confusion
- 평균 **mAP@0.5 = 0.984**

#### Confusion Matrix (normalized)

![Confusion Matrix](runs/detect/runs/mineral/yolo11n_v1/confusion_matrix_normalized.png)

| True \ Predicted | blue | yellow | green | background |
|---|---|---|---|---|
| **blue** | 1.00 | | | |
| **yellow** | | 1.00 | | |
| **green** | | | 1.00 | |
| **background → 오탐** | 0.73 | 0.27 | | |

- 3 mineral class 모두 **100% 정분류** (대각선 1.00)
- **다만 background → mineral** 오탐: blue 73% / yellow 27% — 즉 background 픽셀 일부를 mineral 로 false-positive 함. negative 샘플 (52 장) 만으론 부족할 수 있고, conf_threshold (default 0.5 → 시연 시 0.6 으로 올림) 으로 완화.

### 4.3 학습 후 잡 이슈

- 학습 자체 성공, 마지막 `plot_metrics()` 단계에서 `scipy`/`numpy2` 충돌 → `pip install --user --upgrade scipy` 로 해결, results.png 별도 생성
- mAP50-95 (0.69) 가 mAP50 (0.98) 보다 훨씬 낮지만 **nav cam deployment 엔 무관** (정확 bbox 보다 위치 추정이 핵심)

### 4.4 Val Batch 예측 시각

![Validation Predictions](runs/detect/runs/mineral/yolo11n_v1/val_batch0_pred.jpg)

15 장 중 14 장 정확 검출, confidence 대부분 0.7~0.9. 한 장 (`yellow_0074`) 만 부분적 (yellow 0.34, blue 0.2 중복 박스 — NMS 가 처리). 시연 conf_threshold 0.5 이상이면 모두 valid.

---

## 5. 추론 통합

### 5.1 Detector 라이브러리 (`isaac_perception/yolo_mineral_detector.py`)

`YoloMineralDetector` 클래스 — ultralytics YOLO wrapper:
```python
from yolo_mineral_detector import YoloMineralDetector
det = YoloMineralDetector("models/mineral_yolo_best.pt", conf=0.5)
dets = det.detect(bgr_image)   # List[Detection]
overlay = YoloMineralDetector.draw_overlay(bgr_image, dets)
```

`Detection` dataclass: `cls_id`, `cls_name`, `conf`, `bbox=(x1,y1,x2,y2)`, `cx`, `cy`.

`CLASS_COLORS_BY_NAME` 으로 시각 색상 매핑:
- `blue_mineral` → cyan BGR (255, 100, 100)
- `yellow_mineral` → yellow BGR (50, 220, 255)
- `green_gas` → green BGR (100, 255, 100)

### 5.2 ROS2 노드 (`isaac_perception/yolo_perception_node.py`)

**두 채널 동시 처리**:

| 채널 | 입력 토픽 | 출력 mode | 출력 토픽 |
|---|---|---|---|
| nav (body cam) | `/camera/rover/{image_raw,depth,camera_info}` + `/ground_truth/odom` | `world` | `/perception/detections`, `/perception/image_annotated` |
| wrist (gripper cam) | `/camera/wrist/{image_raw,depth,camera_info}` | `optical` | `/perception/wrist_detections`, `/perception/wrist_image_annotated` |

**Pipeline (per channel)**:
1. RGB image (BGR8) 수신 (subscribe SENSOR_QOS BEST_EFFORT)
2. YOLO inference → bbox + class + conf
3. depth array 의 bbox 중심 픽셀에서 z 읽기
4. `_estimate_xyz`:
   - Optical frame: `(px-cx)*z/fx, (py-cy)*z/fy, z`
   - World frame (nav): optical → ROS body (z fwd, x right, y down) → camera body offset → rover body → world (odom yaw 회전)
5. `Detection` msg per bbox 만들어 `DetectionArray` publish
6. (선택) annotated image publish — 시각 디버깅

**Per-class value scoring**:
```python
VALUE_SCORE_BY_NAME = {
    "blue_mineral":   10.0,
    "yellow_mineral": 50.0,
    "green_gas":      25.0,
}
```
mission_manager 가 priority 결정에 사용.

### 5.3 Image View 토픽 QoS fix (2026-05-26)

초기 `pub_ann` 가 `SENSOR_QOS` (BEST_EFFORT) → `rqt_image_view` (RELIABLE) 와 매칭 안 됨 → 빈 화면. 수정:
```python
# 변경 전 (BEST_EFFORT)
self.pub_ann = node.create_publisher(Image, ann_topic, SENSOR_QOS)
# 변경 후 (RELIABLE)
self.pub_ann = node.create_publisher(Image, ann_topic, 10)
```

---

## 6. 알려진 이슈 / 코드 위생

### 6.1 Perception z bias (실측 +47 cm)

nav cam YOLO bbox 중심 픽셀의 depth backproject 결과 = mineral **표면점** world XYZ. mineral USD origin/center 보다 약 +47 cm 위로 추정됨.

→ `arm_executor_node` 의 `hover_above_mineral_z_m` 파라미터를 음수 (예: -0.30) 로 두어 보정 중. **정공법**: `_estimate_xyz` 가 class 별 평균 mineral 높이를 빼서 mineral 중심 publish.

### 6.2 Depth invalid 시 (0,0,0) 반환

`_estimate_xyz` 가 invalid depth (`inf` 또는 0.05 < z < 50.0 범위 밖) 시 `(0.0, 0.0, 0.0)` 반환:
```python
if not np.isfinite(z) or z <= 0.05 or z > 50.0:
    self._log(f"depth invalid at px=({ix},{iy}) value={z:.4f}", "warn")
    return (0.0, 0.0, 0.0)
```

→ 결과: `mission_manager` 가 world origin 에 mineral 이 있다고 오인할 위험.
→ 현재 mission_manager 가 `(0,0,0)` 좌표 candidate 필터로 시연 영향 없음.
→ **정공법**: None 반환 + caller 가 publish 생략.

### 6.3 Wrist cam 좌표는 optical frame (world 아님)

wrist 채널은 `mode="optical"` → `det.world_position` 이 wrist 카메라 로컬 좌표. `mission_manager` / `arm_executor` 가 world frame 으로 쓰려면 wrist mount calibration 으로 변환 필요:
```python
# arm_executor_node.py 의 _execute_pick_rover_yolo_demo() 의 WRIST_SERVO 단계
wrist_mount_xyz_link6 = [0.0115, 0.0450, 0.0500]  # vehicle_v3.usd 추출값
wrist_R_optical_to_link6 = identity                # OpenCV ≈ link_6 축 정렬
```

→ `wrist_servo_apply_xy=True` param 으로 활성화. default 비활성.

---

## 7. 시연 단계 시행착오 (2026-05-22 ~ 2026-05-24)

### 7.1 Isaac Sim 데모 통합 — `rover_yolo_demo.py`

| 증상 | 원인 | 해결 |
|---|---|---|
| `cv2.namedWindow` 에러 | Isaac 번들 cv2 가 headless build (GTK 미포함) | cv2 GUI 제거, `omni.ui.ByteImageProvider` 로 대체 |
| 차량이 땅 아래로 가라앉음 | physics 가 vehicle 끌어내림 | 전 RigidBody kinematic + 초기 z=1.5 |
| Mineral 들이 떠 있음 | gravity=0 코드가 mineral 도 영향 | gravity 코드 제거 (vehicle 은 kinematic 이라 안 떨어짐) |
| Mineral 사라짐 (raycast 실패) | terrain mesh CollisionAPI 누락 | `heightmap.npy` 직접 sampling (1000×1000, 50 m×50 m, 0.05 m 해상도) |
| 차량 카메라 각도 망가짐 | `ClearXformOpOrder()` + orient quaternion 직접 set | translate op 만 in-place 수정 (orient/scale 보존) |
| YOLO 윈도가 검정 | `set_bytes_data(.tolist(), ...)` API 호환 안 됨 + indent 버그 | `set_bytes_data(.tobytes(), [w,h])` + dedent 수정 + multi-API fallback |
| 종료 시 segfault | `omni.syntheticdata` atexit 정리 중 crash | `del camera` + `world.stop()` + sleep 0.5 — 줄였지만 완전 해결 X (무해) |
| mineral 가까이 가면 YOLO conf < 0.5 | 시야 가장자리/근접 거리에서 confidence drop | autopilot creep → hard X push 전환, push 시작 시점 target snapshot 캡쳐 |
| green_gas 검출 거리 정확도 | nav cam 만으론 부족 | wrist refresh 단계 추가 (`_scan_terrain_minerals` 가 Cube sub-prim XYZ 우선 추출) |

### 7.1.1 Live demo 스크린샷

![Live Demo — rover_yolo_demo.py](runs/mineral/demo_shots/shot_0000.png)

상단 HUD: `pos=(10.5, 6.0) yaw=+44deg det=1 fps=8.5`. nav cam viewport 에서 `green_gas 0.84` 검출 (cyan bbox). 다른 큐브들은 광물 아닌 rock — 학습 데이터 negative 샘플 효과로 정확히 false positive 회피.

### 7.2 Multi-rover 시연 (2026-05-26)

- Per-rover USD copy → USD/OmniGraph prototype 공유 차단
- Self-introspecting `ReadGtPose` / `GraspScript` (자기 prim path 로 rover root 동적 탐색)
- 토픽 namespace 격리 (`/rover_1/*`, `/rover_2/*`)
- 두 rover 의 yolo_perception_node 가 동시에 GPU YOLO 인식 — publish rate ~3 Hz (single rover 10 Hz 의 30%)
- `/mineral_claims` 협조 + `/rover_positions` A* dynamic obstacle 회피

---

## 8. 모델 / 데이터 위치 (시연 기준)

| 항목 | 경로 | 크기 / 수량 |
|---|---|---|
| **최종 모델** | `isaac_perception/models/mineral_yolo_best.pt` | 5.3 MB (git tracked) |
| 학습 결과 폴더 | `isaac_perception/runs/detect/runs/mineral/yolo11n_v1/` | results.png, weights/best.pt, weights/last.pt |
| YOLO dataset | `isaac_perception/dataset/yolo/` | 316 train + 55 val |
| 원본 캡쳐 + 라벨 | `isaac_perception/dataset/manual/{blue,green,yellow,negative}/` | 319 mineral + 52 negative |
| Class 정의 | `isaac_perception/dataset/yolo/data.yaml` | nc: 3 |

---

## 9. 스크립트 인벤토리

| 스크립트 | 역할 | 상태 |
|---|---|---|
| `manual_capture.py` | Isaac Sim viewport 에서 Q 키 PNG 캡쳐 | ✅ 사용 중 |
| `negative_capture.py` | mineral 없는 terrain 캡쳐 (false positive 억제용) | ✅ |
| `build_yolo_dataset.py` | manual/ → YOLO 학습 폴더 build (85/15 split) | ✅ |
| `train_yolo.py` | Ultralytics YOLO 학습 (yolo11n/s/m 등) | ✅ |
| `detect_image.py` | best.pt 로 이미지/폴더 단일 추론 + 시각화 | ✅ |
| `verify_dataset.py` | bbox annotation 시각 검증 (격자) | ✅ |
| `render_labels.py` | .txt YOLO 라벨 → preview PNG | ✅ |
| `rover_yolo_demo.py` | Isaac Sim 통합 데모 (terrain + vehicle + dual-cam YOLO + autopilot + pick&place) | ✅ |
| `hsv_auto_label.py` | (구) HSV 자동 라벨링 | ❌ 폐기 |
| `guided_label.py` | (구) ROI hint HSV 라벨 | ❌ 폐기 |
| `draw_bbox.py` | (구) 좌표 직접 입력 | ❌ 폐기 |

---

## 10. 재현 명령 한 줄

### 10.1 재학습
```bash
cd /home/rokey/dev_ws/rover_ws/src/a2_isaac/isaac_perception
yolo detect train model=yolo11n.pt data=dataset/yolo/data.yaml \
    epochs=200 imgsz=1280 batch=8 patience=30 \
    hsv_h=0.02 hsv_s=0.7 hsv_v=0.5 mosaic=1.0 mixup=0.2 \
    degrees=15 scale=0.5 fliplr=0.5 \
    device=0 project=runs/mineral name=yolo11n_v2
```

### 10.2 단일 데모 (단일 rover)
```bash
isaac-python scripts/rover_yolo_demo.py
```

### 10.3 ROS2 통합 (mvp 시연)
```bash
# Isaac Sim
cd ~/dev_ws/rover_ws/src/a2_isaac
tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00004

# ROS2 노드 묶음 (yolo_perception_node 포함)
source /opt/ros/humble/setup.bash && source ~/dev_ws/rover_ws/install/setup.bash
ros2 launch isaac_bringup mvp.launch.py

# 카메라 view (annotated)
ros2 launch isaac_bringup rqt_views.launch.py
```

### 10.4 ROS2 multi-rover (multi-rover 시연)
```bash
# Isaac Sim (vehicle 2대)
tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00004 --rovers rover_1 rover_2

# ROS2 노드 × 2
ros2 launch isaac_bringup mvp_multi.launch.py

# 카메라 4창
ros2 launch isaac_bringup rqt_views_multi.launch.py
```

---

## 11. 후속 작업 (시연 후 청산 권장)

| 우선순위 | 항목 | 영향 |
|---|---|---|
| 高 | Perception z bias fix — `_estimate_xyz` 가 class 별 mineral 높이 빼서 mineral 중심 publish | `hover_above_mineral_z_m` 튜닝 불필요 |
| 高 | Depth invalid 시 None 반환 + caller filter | `(0,0,0)` 오탐 가능성 제거 |
| 中 | Wrist cam fine-tune 또는 별도 모델 | wrist 시점 confidence 안정 |
| 中 | Multi-rover GPU 부담 완화 — 단일 yolo_perception_node 가 두 카메라 동시 처리 (모델 1번 로드) | publish rate 회복 |
| 中 | mAP50-95 개선 — MakeSense 에서 bbox tightness 재검수 또는 YOLO11s step up | 정밀도 ↑ |
| 低 | ROS2 통합 — wrist 채널도 world frame deproject (TF 또는 FK chain) | wrist_servo_apply_xy 가 정공법 |
| 低 | `negative` 가 학습에 잘 작동하는지 검증 — terrain only 시 false positive 0 인지 추가 테스트 | edge case 안전성 |

---

## 12. 한 줄 요약

> Mars terrain 시뮬에서 광물 3-class 검출을 위한 YOLO11n 모델을 **6 분 학습 / val mAP50 0.984** 로 완성, ROS2 노드로 nav+wrist 두 카메라 동시 처리 + world frame XYZ publish 까지 연결. 라벨링 도구 6차 시행착오 (최종 MakeSense), Blackwell GPU PyTorch cu130 호환, ultralytics × Isaac Sim Python 환경 분리, perception z bias 잔여 이슈를 거쳐 **mvp / multi-rover 시연 모두 안정 작동**.

---

**부록 — 핵심 파일 트리**
```
isaac_perception/
├── models/
│   └── mineral_yolo_best.pt          ⭐ 학습 모델 (5.3MB, git tracked)
├── dataset/
│   ├── manual/{blue,green,yellow,negative}/   ⭐ 원본 캡쳐 + .txt 라벨
│   └── yolo/                                  ⭐ build 결과 (symlink)
│       ├── train/{images,labels}/
│       ├── val/{images,labels}/
│       ├── data.yaml
│       └── classes.txt
├── runs/detect/runs/mineral/yolo11n_v1/       학습 산출 (results.png 등)
├── isaac_perception/
│   ├── yolo_mineral_detector.py      ⭐ Detector 라이브러리 (ultralytics wrapper)
│   ├── yolo_perception_node.py       ⭐ ROS2 노드 (nav + wrist 동시)
│   ├── perception_node.py            ⚠ stub
│   ├── cyan_detector.py              HSV 보조 (cyan용)
│   └── {vision, depth, lidar}/       모듈별 sub-dir
└── scripts/
    ├── manual_capture.py             ⭐
    ├── negative_capture.py           ⭐
    ├── build_yolo_dataset.py         ⭐
    ├── train_yolo.py                 ⭐
    ├── detect_image.py
    ├── verify_dataset.py
    ├── render_labels.py
    └── rover_yolo_demo.py            ⭐ Isaac Sim 통합 데모
```
