# 📡 Interface Contracts — Day 1 합의 사항

> **본 문서는 5인 트랙이 병렬 작업하기 위한 cross-track 인터페이스 명세입니다.**
> Day 1 (5/19) 킥오프 회의에서 합의 → 사인 → 변경 freeze.
> Day 2 이후 변경은 PM 승인 + 전체 회의 필요.

---

## 📋 트랙 ↔ 담당자

| 트랙 | 담당자 | 영역 |
|:----:|:-----:|------|
| **T1** | **김현중** | Environment (절차생성 지형, basecamp, Mars physics) |
| **T2** | **최진우** | Perception + M0609 (vision, manipulation) |
| **T3** | **이찬휘** | Driving (mission FSM, A*, coverage, PPO wrapper) |
| **T4** | **성선규** | Integration + PM (ROS2 wiring, UI, demo) — 사용자 본인 |
| **T5** | **이지민** | Localization + Infra (TRN, EKF, sensor fusion) |

→ 본 문서에서는 "T1 (김현중)" 처럼 트랙 ID와 사람 이름을 병기합니다.

## 📋 합의 항목 — Critical 5개

| ID | 인터페이스 | Producer | Consumer | 종류 | 빈도 |
|:--:|------------|:--------:|:--------:|:----:|:----:|
| **I1** | Terrain Asset | T1 (김현중) | T2 (최진우), T3 (이찬휘), T4 (성선규), T5 (이지민) | 파일 | static |
| **I2** | `/perception/detections` | T2 (최진우) | T3 (이찬휘), T4 (성선규) | ROS2 | 10 Hz |
| **I3** | `/mission/pick_request` | T3 (이찬휘) | T2 (최진우) | ROS2 | event |
| **I4** | `/mission/pick_response` | T2 (최진우) | T3 (이찬휘) | ROS2 | event |
| **I5** | `/rover/estimated_pose` | T5 (이지민) | T3 (이찬휘), T4 (성선규) | ROS2 | 30 Hz |

5개 외 UI 관련 인터페이스(I6~I10)는 [`deferred_interfaces.md`](deferred_interfaces.md)에서 관리. T3 (이찬휘) ↔ T4 (성선규) 사이 Day 4+ 합의.

**I11** `/arm/joint_command` — vehicle_v3 로봇의 저수준 팔 제어 인터페이스 (2026-05-22 추가, 본 문서 하단 참조). v3 그래프-내장 로봇 아키텍처 도입에 따른 신규 로봇 제어 인터페이스.

---

## 🌐 공통 규칙

### 좌표계
- **모든 위치/방향은 world frame** (Isaac Sim 월드 좌표계)
- 원점: 지형 (0, 0, 0)
- X축 = 전방, Y축 = 좌측, Z축 = 상방 (right-hand rule)
- 단위: 미터 (m), 라디안 (rad)

### Timestamp
- 모든 ROS2 메시지의 `header.stamp`는 ROS time 사용
- Producer가 publish 시점 기준 stamp

### Frame ID
- 모든 위치 정보 메시지의 `header.frame_id = "world"`

### 단위
- 거리: meter
- 각도: radian (degree 사용 시 필드명에 `_deg` suffix)
- 시간: second (millisecond 사용 시 `_ms`)
- 마찰: 무차원 (PhysX 표준)
- 확률: 0.0 ~ 1.0

### 메시지 발급 ID
- 모든 unique ID는 **int32 양수** (음수는 invalid/sentinel)
- `mineral_id`: T1 (김현중)이 meta.json에서 발급 (1부터 순차)
- `request_id`: T3 (이찬휘)가 PickRequest 발행 시 발급 (1부터 순차)

---

## I1. Terrain Asset (T1 김현중 → 모두)

### 핵심 한 줄
**T1 (김현중)이 시뮬 시작 *전*에 디스크에 떨궈놓는 정적 파일 묶음.** 하나의 화성 지형 = `terrain_NNNNN/` 디렉터리 하나. T2~T5(최진우/이찬휘/성선규/이지민)가 시뮬 로드 시 읽어감. (I2~I5는 런타임 ROS2 메시지지만, I1만 파일 계약)

### 누가 뭘 읽나

| 파일 | Consumer | 용도 |
|------|---------|------|
| `terrain_only.usd` | Isaac Sim (모두) | 지형 메쉬 로드 (렌더링/충돌) |
| `rocks_merged.usd` | Isaac Sim (모두) | 암석 메쉬 로드 |
| `obstacle_grid.npy` | **T3 (이찬휘)** | A* 경로 계획 입력 (2D 통과 가능/불가) |
| `heightmap.npy` | **T5 (이지민)** ⭐ | TRN 매칭용 글로벌 높이 지도 |
| `meta.json` | **모두** | 광물/베이스캠프/스폰 위치 등 시뮬 셋업 |
| `index.json` (루트) | T3 (이찬휘) 학습 루프 | train/holdout split 목록 |

### 디렉터리 구조

```
generated_terrains/             # T1 (김현중)이 시드별로 생성한 지형들을 누적
├── terrain_00001/              # 지형 1개 = 50m×50m 화성 표면 1장
│   ├── terrain_only.usd
│   ├── rocks_merged.usd
│   ├── obstacle_grid.npy
│   ├── heightmap.npy
│   └── meta.json
├── terrain_00002/
│   └── ...
└── index.json                  # 전체 목록 + train/holdout split
```

PPO 학습은 매 episode마다 여기서 무작위로 한 디렉터리를 뽑아 로드.

### obstacle_grid.npy 형식

```python
shape: (1000, 1000)
dtype: int8
값:   0 = safe (통과 가능)
      1 = obstacle (큰 바위, 절벽)
좌표: grid[i, j] = (origin.x + j*res, origin.y + i*res)
      row-major, j=x축, i=y축

예시 (numpy 로드):
import numpy as np
grid = np.load("terrain_00001/obstacle_grid.npy")
print(grid.shape)   # (1000, 1000)
print(grid.dtype)   # int8
print(grid[500, 500])  # 0 또는 1
```

### heightmap.npy 형식 (⭐ T5 (이지민) TRN이 사용)

```python
shape: (1000, 1000)
dtype: float32
값:   각 cell의 높이 (m)
좌표: grid[i, j] = height at world position (origin.x + j*res, origin.y + i*res)
      row-major, j=x축, i=y축

예시:
heightmap = np.load("terrain_00001/heightmap.npy")
print(heightmap[500, 500])  # e.g., 0.34 (m)
print(heightmap.min(), heightmap.max())  # e.g., -1.2, 4.7
```

→ **T1 (김현중)과 T5 (이지민) 사이 가장 중요한 합의**. 좌표계 한 번 틀리면 TRN 전체 망가짐.

### 관련 문서

- **I1 전체 가이드**: [`I1_TERRAIN_ASSETS.md`](I1_TERRAIN_ASSETS.md) — 5개 파일 + master scene composition + reference 그래프
- **meta.json 필드 가이드**: [`META_JSON_FIELDS.md`](META_JSON_FIELDS.md) — 실제 샘플 값 + 라인별 주석
- **JSON Schema**: [`terrain_meta_schema.json`](terrain_meta_schema.json) (JSON Schema 7)
- **예시**: [`example_terrain_meta.json`](example_terrain_meta.json)

### meta.json 핵심 필드 요약

| 필드 그룹 | 키 | Consumer |
|---|---|---|
| 정체성 | `terrain_id`, `version`, `seed`, `generated_at` | 모두 |
| 공간 | `size_m`, `resolution_m`, `origin` | 이찬휘 T3, 이지민 T5 (좌표계 합의) |
| 스폰 | `spawn_locations[]` | 이찬휘 T3, 이지민 T5 |
| 베이스캠프 | `basecamp.{center,radius,marker_usd,...}` | 이찬휘 T3 FSM, 성선규 T4 UI |
| 광물 | `minerals[].{id,type,position,value}` | 최진우 T2, 이찬휘 T3, 성선규 T4 |
| 영역 마찰 | `physics_zones[]` | 이지민 T5 Mars Tier 2 |
| 미니맵 | `minimap.{grid_size,cell_size_m,origin}` | 이찬휘 T3 Coverage, 성선규 T4 |
| 메트릭 | `difficulty.{score,rock_density,...}` | 모두 (curriculum) |

**Required fields**: `terrain_id`, `version`, `seed`, `size_m`, `resolution_m`, `origin`, `spawn_locations`, `basecamp`, `minerals`, `minimap`, `difficulty`

**Tier 2 호환 필드** (현재 null/[], 삭제 금지):
- `basecamp.shape`, `basecamp.entry_points`, `basecamp.collision_usd_path`
- `generation_params` (재현용, 런타임 사용 X)

→ 풀 예시 + 라인별 주석: [**META_JSON_FIELDS.md**](META_JSON_FIELDS.md) (실제 첫 샘플 값 사용).

### index.json 형식

```json
{
  "version": "1.0",
  "generated_at": "2026-05-21T03:45:00",
  "terrains": [
    {"id": "terrain_00001", "split": "train",   "difficulty": 0.35, "seed": 12345},
    {"id": "terrain_00002", "split": "train",   "difficulty": 0.42, "seed": 12346},
    {"id": "terrain_00003", "split": "holdout", "difficulty": 0.61, "seed": 12347}
  ]
}
```

- `split`: `"train"` | `"holdout"` — T3 (이찬휘) 학습 루프가 sampling 시 필터
- `difficulty`: `meta.json.difficulty.score` 와 일치 (검색용 캐시)

### 검증 방법

```bash
# 모든 terrain의 meta.json이 schema 통과하는지
python3 interfaces/validate_terrain_meta.py generated_terrains/
```

---

## I2. `/perception/detections` (T2 최진우 → T3 이찬휘, T4 성선규)

### 메시지 정의 (인라인 형식)

```
# DetectionArray.msg (publish 단위)
std_msgs/Header header           # frame_id = "world", stamp = publish 시각
Detection[] detections           # 빈 배열 OK

# Detection.msg (단일 광물)
string class_name                # "blue_mineral" | "yellow_mineral" | "green_gas"
geometry_msgs/Point world_position  # x, y, z (m)
float32 confidence               # 0.0 ~ 1.0
float32 value_score              # blue_mineral=10, green_gas=25, yellow_mineral=50
int32 mineral_id                 # T1 (김현중) meta.minerals[].id 매칭 (-1=매칭실패)
geometry_msgs/Vector3 bbox_size_m  # 월드 단위 bounding box (현재 0,0,0 — Phase 2+ 채움)
int32 bbox_xmin                  # 이미지 픽셀 bbox (UI overlay 합성용)
int32 bbox_ymin
int32 bbox_xmax
int32 bbox_ymax
```

원본 파일: [`msg/Detection.msg`](msg/Detection.msg), [`msg/DetectionArray.msg`](msg/DetectionArray.msg)

### 예시 dummy payload

**케이스 A: 광물 2개 검출**
```yaml
header:
  stamp: {sec: 12, nanosec: 340000000}
  frame_id: "world"
detections:
  - class_name: "blue_mineral"
    world_position: {x: 8.05, y: 4.12, z: 0.10}
    confidence: 0.87
    value_score: 10.0
    mineral_id: 1
    bbox_size_m: {x: 0.0, y: 0.0, z: 0.0}
    bbox_xmin: 312
    bbox_ymin: 228
    bbox_xmax: 345
    bbox_ymax: 261
  - class_name: "yellow_mineral"
    world_position: {x: -11.05, y: 11.42, z: 0.10}
    confidence: 0.92
    value_score: 50.0
    mineral_id: 4
    bbox_size_m: {x: 0.0, y: 0.0, z: 0.0}
    bbox_xmin: 95
    bbox_ymin: 240
    bbox_xmax: 130
    bbox_ymax: 282
```

**케이스 B: 아무것도 검출 안 됨 (빈 배열도 publish)**
```yaml
header:
  stamp: {sec: 13, nanosec: 440000000}
  frame_id: "world"
detections: []
```

**케이스 C: meta 매칭 실패 (mineral_id = -1)**
```yaml
header:
  stamp: {sec: 14, nanosec: 540000000}
  frame_id: "world"
detections:
  - class_name: "green_gas"
    world_position: {x: 20.5, y: -8.3, z: 0.10}   # meta에 없는 위치
    confidence: 0.61
    value_score: 25.0
    mineral_id: -1                                  # 매칭 실패
    bbox_size_m: {x: 0.0, y: 0.0, z: 0.0}
    bbox_xmin: 540
    bbox_ymin: 198
    bbox_xmax: 568
    bbox_ymax: 224
```

### T2 (최진우) 내부 구현 — HSV 색기반 detection

```
Isaac Sim camera (640×480 RGB) ← 마스트 카메라 (높이 0.7m, 30° 아래)
        │
        ▼
HSV threshold (3색: blue/red/yellow)
        │
        ▼
Connected components + morphology
        │
        ▼
2D pixel → 3D world (T5 (이지민) estimated_pose 사용, GT 아님)
        │
        ▼
T1 (김현중) meta의 mineral_id와 매칭
```

→ T5 (이지민)의 estimated_pose가 부정확하면 detection 좌표도 부정확. **T5 (이지민) 정확도가 detection 품질에 영향**.

### 발급 규칙

- **10 Hz publish** (카메라 갱신 주기 동기)
- detections 배열이 **비어있어도 publish** (T3 (이찬휘)가 "아무것도 못 봄" 상태 인지)
- `confidence` < 0.5 인 detection은 제외 (T2 (최진우)가 사전 필터)
- 동일 mineral이 연속 프레임에서 검출되면 같은 `mineral_id` 부여 (T1 (김현중) meta의 ID와 매칭)
- 매칭 실패 시 `mineral_id = -1` (T3 (이찬휘)가 처리)

### 광물 시각 디자인 (T1 (김현중)과 합의 필요)

| 광물 타입 | 실제 시각 색 | mesh 형태 | value_score |
|---|---|---|:-:|
| `blue_mineral` | cyan/teal 결정 클러스터 | 비정형 polytope | 10 |
| `yellow_mineral` | 밝은 노랑 spike 결정 클러스터 | spike polytope | 50 |
| `green_gas` | 진녹색 가스 박스 | 정육면체 | 25 |

> ⚠️ `green_gas` 의 USD prim prefix 는 legacy 로 `red_*` 였음 — terrain 재생성 후 `green_gas_*` 로 통일됨.

### Mock 단계 (Day 1-2)
- T2 (최진우) stub은 GT 광물 좌표 + 노이즈를 detection으로 발행
- 진짜 vision은 Day 2-3 EOD 동작

### Consumer 동작
- **T3 (이찬휘) FSM**: detection 받으면 EXPLORE → APPROACH 전환 가능
  - 다수 광물 동시 detection 시 value_score / dist 기준 우선순위
- **T4 (성선규) UI**: detection 마커를 미니맵/3D view에 표시

---

## I3. `/mission/pick_request` (T3 이찬휘 → T2 최진우)

### 메시지 정의 (인라인 형식)

```
# PickRequest.msg
std_msgs/Header header
int32 mineral_id                 # 잡을 광물 (T2 (최진우) detection의 mineral_id 또는 T1 (김현중) meta의 id)
geometry_msgs/Point target_position  # 광물 좌표 (월드, m)
float32 timeout_s                # 기본 30.0
int32 request_id                 # T3 (이찬휘) 발급 unique seq (1부터)
```

원본: [`msg/PickRequest.msg`](msg/PickRequest.msg)

### 예시 dummy payload

```yaml
header:
  stamp: {sec: 25, nanosec: 120000000}
  frame_id: "world"
mineral_id: 3
target_position: {x: 14.9, y: 14.2, z: 0.10}
timeout_s: 30.0
request_id: 7
```

### 발급 규칙

- **이벤트 기반**: FSM이 PICK phase 진입 시 1회
- `request_id`로 중복 방지 (T2 (최진우)가 같은 ID 두 번 받으면 무시)
- `timeout_s`: 기본 30.0초 (Tier 1.5 manipulation 충분)
- `target_position`: T2 (최진우)가 detection에서 받았던 좌표 echo (검증용)

### Consumer 동작

T2 (최진우)가 받으면:
1. M0609을 target 좌표 위로 이동
2. Scripted grasp 시퀀스 실행
3. 광물 USD를 cargo bin으로 텔레포트
4. I4 (PickResponse)로 결과 응답

T2 (최진우)는 처리 중 새 PickRequest 받아도 큐잉하지 않고 무시 (FSM이 직렬 처리 보장).

---

## I4. `/mission/pick_response` (T2 최진우 → T3 이찬휘)

### 메시지 정의 (인라인 형식)

```
# PickResponse.msg
std_msgs/Header header
int32 mineral_id                 # I3의 mineral_id echo
int32 request_id                 # I3의 request_id echo (반드시)
string status                    # success | failed_grasp | timeout | no_object
float32 duration_s               # 실제 소요 시간
string failure_reason            # status != success 일 때만 free-form
```

원본: [`msg/PickResponse.msg`](msg/PickResponse.msg)

### 예시 dummy payload

**케이스 A: 성공**
```yaml
header:
  stamp: {sec: 32, nanosec: 850000000}
  frame_id: "world"
mineral_id: 3
request_id: 7
status: "success"
duration_s: 7.73
failure_reason: ""
```

**케이스 B: grasp 실패**
```yaml
header:
  stamp: {sec: 38, nanosec: 200000000}
  frame_id: "world"
mineral_id: 5
request_id: 12
status: "failed_grasp"
duration_s: 13.10
failure_reason: "Gripper closed but object slipped"
```

**케이스 C: timeout**
```yaml
header:
  stamp: {sec: 95, nanosec: 0}
  frame_id: "world"
mineral_id: 8
request_id: 21
status: "timeout"
duration_s: 30.0
failure_reason: "Reach timeout: arm did not converge"
```

### Status 값

| status | 의미 | T3 (이찬휘) 동작 |
|--------|------|---------|
| `success` | 정상 수집 완료 | FSM: cargo++ → 다음 phase |
| `failed_grasp` | 접근했으나 grasp 실패 | FSM: 다음 광물로 (재시도 X) |
| `timeout` | timeout_s 초과 | FSM: 다음 광물로 |
| `no_object` | target 위치에 광물 없음 | FSM: detection 무효 처리 |

### 발급 규칙
- **PickRequest 받은 후 timeout_s 안에 반드시 응답**
- `request_id`는 받은 그대로 echo
- `duration_s`: 실제 소요 시간 (디버깅용)

---

## I5. `/rover/estimated_pose` (T5 이지민 → T3 이찬휘, T4 성선규)

### 메시지 정의 (인라인 형식)
**ROS2 표준 사용**: `geometry_msgs/PoseWithCovarianceStamped`

```
std_msgs/Header header               # frame_id = "world"
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position       # x, y, z (m)
    geometry_msgs/Quaternion orientation
  float64[36] covariance               # 6x6 row-major, variance (not std)
                                       # 순서: [x, y, z, roll, pitch, yaw]
```

### 예시 dummy payload

**케이스 A: 초기 (정확도 높음, 공분산 작음)**
```yaml
header:
  stamp: {sec: 5, nanosec: 0}
  frame_id: "world"
pose:
  pose:
    position: {x: 0.05, y: 0.02, z: 0.18}
    orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}  # yaw 90° (z축 회전)
  covariance:                          # 6x6 row-major (36 floats)
    # x   y   z   r   p   yaw
    [0.01, 0,    0,    0,    0,    0,
     0,    0.01, 0,    0,    0,    0,
     0,    0,    0.01, 0,    0,    0,
     0,    0,    0,    0.005, 0,   0,
     0,    0,    0,    0,    0.005, 0,
     0,    0,    0,    0,    0,    0.01]
```

**케이스 B: 드리프트 누적 (TRN 보정 전, 공분산 증가)**
```yaml
header:
  stamp: {sec: 120, nanosec: 0}
  frame_id: "world"
pose:
  pose:
    position: {x: 14.32, y: 7.85, z: 0.21}    # GT: (14.50, 7.90)
    orientation: {x: 0.0, y: 0.0, z: 0.71, w: 0.704}
  covariance:                          # 분산 ~10배 증가
    [0.10, 0,    0,    0,    0,    0,
     0,    0.10, 0,    0,    0,    0,
     0,    0,    0.05, 0,    0,    0,
     0,    0,    0,    0.02, 0,    0,
     0,    0,    0,    0,    0.02, 0,
     0,    0,    0,    0,    0,    0.08]
```

**케이스 C: TRN 보정 직후 (공분산 다시 감소)**
```yaml
header:
  stamp: {sec: 121, nanosec: 0}
  frame_id: "world"
pose:
  pose:
    position: {x: 14.48, y: 7.92, z: 0.21}    # TRN 매칭으로 GT 근접
    orientation: {x: 0.0, y: 0.0, z: 0.71, w: 0.704}
  covariance:                          # TRN 보정으로 감소
    [0.02, 0,    0,    0,    0,    0,
     0,    0.02, 0,    0,    0,    0,
     0,    0,    0.02, 0,    0,    0,
     0,    0,    0,    0.005, 0,   0,
     0,    0,    0,    0,    0.005, 0,
     0,    0,    0,    0,    0,    0.015]
```

### 공분산 의미 가이드

| 분산 값 | 표준편차 (√) | 의미 |
|--------|:---:|------|
| 0.01 | 0.1 m | 매우 정확 (10cm 1σ) |
| 0.05 | 0.22 m | 보통 (~22cm 1σ) |
| 0.10 | 0.32 m | 부정확 (~32cm 1σ, TRN 필요) |
| 0.50 | 0.71 m | 매우 부정확 (~71cm 1σ, fallback 필요) |

### T5 (이지민) 내부 구현 — TRN + Multi-sensor Fusion

T5 (이지민)는 다음 4개 입력을 EKF로 융합하여 estimated_pose 생성. **GT는 정책에 직접 사용하지 않음**.

```
4개 센서 입력 (모두 Isaac Sim 시뮬 노이즈 포함):
  ① Wheel Odometry      ← joint_vel 적분 + Ackermann 모델
  ② IMU Integration     ← 자이로 + 가속도 적분 (드리프트 누적)
  ③ Sun Yaw             ← 광원 방향 → 절대 방위 (드리프트 0)
  ④ TRN ⭐              ← 로컬 heightmap을 T1 (김현중) global heightmap에 매칭
                           (Terrain Relative Navigation)
        │
        ▼
  EKF Fusion (drift correction by TRN every ~5s)
        │
        ▼
  estimated_pose + covariance (variance, not std)
```

**TRN (Terrain Relative Navigation)** = 실제 Perseverance 화성 착륙 시 사용 기법:
- RayCaster의 5m × 5m 로컬 heightmap 측정
- T1 (김현중)의 global heightmap.npy와 cross-correlation 매칭
- 가장 잘 맞는 위치로 EKF drift 보정 (loop closure 역할)

### 발급 규칙

- **30 Hz publish** (PPO step rate 동기)
- Covariance: 시간 지남에 따라 증가 (드리프트 누적). TRN 보정 시 감소.
- `header.frame_id = "world"` 고정
- 빈 publish 안 함 (항상 최신 추정치)

### 단계별 구현 (Day별)

| Day | T5 (이지민) 구현 수준 | Covariance 동작 |
|:---:|------------|----------------|
| Day 1 | Stub: GT + 가우시안 노이즈 (간단) | 고정 |
| Day 2 | + Wheel/IMU 적분 | 누적 |
| Day 3 | + TRN 단독 검증 | TRN 신뢰도 기반 |
| Day 4 | EKF 융합 (4개 센서) | 동적 (TRN 보정 시 감소) |
| Day 5+ | 노이즈 파라미터 튜닝, 노이즈 σ별 sweep | |

### Consumer 동작
- **T3 (이찬휘)**: 모든 모듈에서 `pose_provider.get_pose()` 추상화 통해 사용
  - FSM: "is rover home?" 판정
  - Coverage: 미니맵 셀 방문 처리
  - PPO Wrapper: command_manager의 robot frame 변환 입력
- **T4 (성선규) UI**: 추정 위치 + 공분산 타원 시각화

### Fallback 정책

T5 (이지민)가 publish 멈추면:
1. T3 (이찬휘)의 `PoseProvider`가 자동으로 GT stub 모드로 fallback
2. PM (성선규)이 즉시 alert 받음
3. T5 (이지민) 재시작 후 자동 ROS2 모드 복귀

→ T5 (이지민) 불안정성이 T3 (이찬휘) 작업 막지 않음.

---

## I11. `/arm/joint_command` (arm_executor → vehicle_v3 로봇)

> 2026-05-22 추가. vehicle_v3 그래프-내장 로봇 아키텍처 도입에 따른 신규 인터페이스. PM (T4 성선규) 승인.

### 핵심 한 줄
**M0609 6축 팔 + RG2-FT 그리퍼 관절을 직접 움직이는 저수준 제어 토픽.** 주행의 `/cmd_vel`에 대응하는 팔 버전. `arm_executor_node`(고수준 `ExecuteArmTask` 액션 서버)가 스크립트 시퀀스를 풀어 이 토픽으로 관절 위치를 지령하면, `vehicle_v3.usd` 내장 Action Graph(`ROS2SubscribeJointState`→`IsaacArticulationController`)가 해당 관절에 적용한다.

### 메시지 정의 (인라인 형식)
**ROS2 표준 사용**: `sensor_msgs/JointState`

```
std_msgs/Header header     # 미사용 가능 (stamp 참조 안 함)
string[]  name             # 움직일 관절 이름 (부분 집합 OK)
float64[] position         # 각 관절 목표 위치 (rad) — name 과 동일 길이
float64[] velocity         # 미사용 (빈 배열)
float64[] effort           # 미사용 (빈 배열)
```

### 관절 이름

| 부위 | 관절 이름 |
|------|----------|
| M0609 팔 | `joint_1` `joint_2` `joint_3` `joint_4` `joint_5` `joint_6` |
| RG2-FT 그리퍼 | `finger_joint` (knuckle/finger mimic 관절은 자동 추종) |

`name`에 넣은 관절만 제어된다 — 일부만 보내도 됨. `position` 길이는 `name`과 일치해야 함.

### 예시 dummy payload

**케이스 A: 팔 3축 + 그리퍼 닫기**
```yaml
name:     ["joint_1", "joint_2", "joint_4", "finger_joint"]
position: [0.6, -0.4, 0.5, 0.3]
velocity: []
effort:   []
```

**케이스 B: 그리퍼만 열기**
```yaml
name:     ["finger_joint"]
position: [0.0]
```

### 발급 규칙

- **위치 제어** — `position`만 사용 (velocity/effort는 무시).
- `arm_executor_node`가 스크립트 5단계(extend→descend→grasp→lift→stow)의 각 step 관절 목표를 발행.
- 그리퍼: `finger_joint` 위치로 개폐 (0 = 열림 / 커질수록 닫힘).
- HOME 자세: `joint_3 ≈ joint_5 ≈ 1.57 rad`, 나머지 ≈ 0 (접힘 상태).

### Producer / Consumer

| | |
|---|---|
| Producer | `arm_executor_node` (T2 최진우 / manipulation) — `ExecuteArmTask` 액션 step 실행 |
| Consumer | `vehicle_v3.usd` 내장 그래프 `ArmCtrl` 노드 |

### 계층 구조

```
T3 ──ExecuteArmTask 액션──▶ arm_executor_node ──/arm/joint_command──▶ vehicle_v3 ──▶ m0609 관절
  (고수준: "저 미네랄 집어")    (스크립트 5단계)       (저수준: 관절 위치)
```

- 고수준 `ExecuteArmTask` 액션은 `isaac_interfaces/action/ExecuteArmTask.action`에 기정의 (변경 없음).
- `vehicle_v3.usd` = 그래프 내장 로봇. terrain에 reference·play 하면 이 토픽이 자동 노출됨 (런처 코드 불필요).

---

## 🔒 변경 정책

### Day 1 sign-off
- 모든 트랙 owner가 본 문서에 ✅ 사인
- 사인 후 schema lock

### 변경 절차 (Day 2 이후)
1. 변경 제안자가 PM에게 alert
2. PM이 영향받는 트랙 owner 소집 (15분 회의)
3. 합의 시 본 문서 + 영향 파일 동시 갱신
4. CHANGELOG 섹션에 기록

### Breaking change 금지
- 필드 삭제 ❌
- 필드 타입 변경 ❌
- 필드 의미 변경 ❌
- 토픽 이름 변경 ❌

추가는 OK:
- 새 optional 필드 추가 ✅
- 새 status 값 추가 (default fallback 보장 시) ✅

---

## 📋 사인란 (Day 1 회의 후)

```
T1 김현중 Environment        (5060)     ___________  날짜 ____
T2 최진우 Perception+M0609   (5080)     ___________  날짜 ____
T3 이찬휘 Driving            (5080)     ___________  날짜 ____
T4 성선규 Integration+PM     (5070 Ti)  ___________  날짜 ____
T5 이지민 Localization+Infra (5080)     ___________  날짜 ____
```

---

## 📝 CHANGELOG

```
[Unreleased]
- Initial draft (Day 0)
- 5 critical interfaces defined

[2026-05-22]
- I11 `/arm/joint_command` 추가 — vehicle_v3 그래프-내장 로봇의 저수준 팔
  제어 인터페이스 (sensor_msgs/JointState). PM (T4 성선규) 승인.
- vehicle_v3 아키텍처(그래프를 코드로 빌드해 USD 에 bake) 도입에 따른 신규
  로봇 인터페이스. 고수준 ExecuteArmTask 액션은 변경 없음.
```
