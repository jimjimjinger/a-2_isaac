# 📡 Interface Contracts — Day 1 합의 사항

> **본 문서는 5인 트랙이 병렬 작업하기 위한 cross-track 인터페이스 명세입니다.**
> Day 1 (5/19) 킥오프 회의에서 합의 → 사인 → 변경 freeze.
> Day 2 이후 변경은 PM 승인 + 전체 회의 필요.

---

## 📋 합의 항목 — Critical 5개

| ID | 인터페이스 | Producer | Consumer | 종류 | 빈도 |
|:--:|------------|:--------:|:--------:|:----:|:----:|
| **I1** | Terrain Asset | T1 | T2, T3, T4, T5 | 파일 | static |
| **I2** | `/perception/detections` | T2 | T3, T4 | ROS2 | 10 Hz |
| **I3** | `/mission/pick_request` | T3 | T2 | ROS2 | event |
| **I4** | `/mission/pick_response` | T2 | T3 | ROS2 | event |
| **I5** | `/rover/estimated_pose` | T5 | T3, T4 | ROS2 | 30 Hz |

5개 외 UI 관련 인터페이스(I6~I10)는 [`deferred_interfaces.md`](deferred_interfaces.md)에서 관리. T3 ↔ T4 사이 Day 4+ 합의.

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
- `mineral_id`: T1이 meta.json에서 발급 (1부터 순차)
- `request_id`: T3가 PickRequest 발행 시 발급 (1부터 순차)

---

## I1. Terrain Asset (T1 → 모두)

### 형식
파일 시스템 기반. 각 terrain은 디렉터리 단위.

```
generated_terrains/
├── terrain_00001/
│   ├── terrain_only.usd        # 지형 메쉬
│   ├── rocks_merged.usd        # 암석 메쉬
│   ├── obstacle_grid.npy       # A* 입력 (numpy 2D int8: 0=safe, 1=obstacle)
│   ├── heightmap.npy           # 정밀 높이 (numpy 2D float32)
│   └── meta.json               # 모든 메타데이터
├── terrain_00002/
│   └── ...
└── index.json                  # 전체 목록 + train/holdout split
```

### meta.json 스키마

전체 스키마는 [`terrain_meta_schema.json`](terrain_meta_schema.json) 참조 (JSON Schema 7).
예시 데이터는 [`example_terrain_meta.json`](example_terrain_meta.json) 참조.

**Required fields**:
- `terrain_id`, `version`, `seed`
- `size_m`, `resolution_m`, `origin`
- `spawn_locations`, `basecamp`, `minerals`, `minimap`
- `difficulty`

**Optional fields (Tier 2 호환용, null 가능)**:
- `basecamp.shape`, `basecamp.entry_points`, `basecamp.collision_usd_path`
- `generation_params` (재현용, 런타임 사용 X)
- `physics_zones` (T5의 Mars Tier 2가 사용)

### Tier 1 vs Tier 2 확장 호환성

```
Tier 1 (8일 프로젝트):
  basecamp = {center, radius, marker_usd, visual_footprint_m, marker_height_m}
  
Tier 2 (후행 마일스톤, schema는 지금 호환):
  basecamp = {center, radius, marker_usd, ..., shape, entry_points, collision_usd_path}
```

→ Tier 2 필드는 지금 null로 두되 **schema에는 존재**. 후행 확장 시 추가 작업 없음.

### heightmap.npy 형식 (T5 TRN이 사용)

```
파일: terrain_NNNNN/heightmap.npy
형식: np.ndarray, shape=(1000, 1000), dtype=float32
의미: 각 cell의 높이 (m), origin from meta.origin, resolution from meta.resolution_m
좌표:
  grid[i, j] = height at world position (origin.x + j*res, origin.y + i*res)
  (row-major, j=x axis, i=y axis)
```

→ **T1과 T5 사이 가장 중요한 합의**. 좌표계 한 번 틀리면 TRN 전체 망가짐.

### 검증 방법

```bash
# 모든 terrain의 meta.json이 schema 통과하는지
python3 interfaces/validate_terrain_meta.py generated_terrains/
```

---

## I2. `/perception/detections` (T2 → T3, T4)

### 메시지 정의

- 단일 detection: [`msg/Detection.msg`](msg/Detection.msg)
- 배열 (publish 단위): [`msg/DetectionArray.msg`](msg/DetectionArray.msg)

### T2 내부 구현 — HSV 색기반 detection

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
2D pixel → 3D world (T5 estimated_pose 사용, GT 아님)
        │
        ▼
T1 meta의 mineral_id와 매칭
```

→ T5의 estimated_pose가 부정확하면 detection 좌표도 부정확. **T5 정확도가 detection 품질에 영향**.

### 발급 규칙

- **10 Hz publish** (카메라 갱신 주기 동기)
- detections 배열이 **비어있어도 publish** (T3가 "아무것도 못 봄" 상태 인지)
- `confidence` < 0.5 인 detection은 제외 (T2가 사전 필터)
- 동일 mineral이 연속 프레임에서 검출되면 같은 `mineral_id` 부여 (T1 meta의 ID와 매칭)
- 매칭 실패 시 `mineral_id = -1` (T3가 처리)

### 광물 시각 디자인 (T1과 합의 필요)

| 광물 타입 | RGB | HSV hue | value_score |
|---------|-----|:------:|:-----:|
| `mineral_blue` | (50, 100, 240) | 105~120° | 10 |
| `mineral_red` | (230, 60, 60) | 0~10° (or 350~360°) | 25 |
| `mineral_yellow` | (240, 220, 50) | 25~35° | 50 |

### Mock 단계 (Day 1-2)
- T2 stub은 GT 광물 좌표 + 노이즈를 detection으로 발행
- 진짜 vision은 Day 2-3 EOD 동작

### Consumer 동작
- **T3 FSM**: detection 받으면 EXPLORE → APPROACH 전환 가능
  - 다수 광물 동시 detection 시 value_score / dist 기준 우선순위
- **T4 UI**: detection 마커를 미니맵/3D view에 표시

---

## I3. `/mission/pick_request` (T3 → T2)

### 메시지 정의
[`msg/PickRequest.msg`](msg/PickRequest.msg)

### 발급 규칙

- **이벤트 기반**: FSM이 PICK phase 진입 시 1회
- `request_id`로 중복 방지 (T2가 같은 ID 두 번 받으면 무시)
- `timeout_s`: 기본 30.0초 (Tier 1.5 manipulation 충분)
- `target_position`: T2가 detection에서 받았던 좌표 echo (검증용)

### Consumer 동작

T2가 받으면:
1. M0609을 target 좌표 위로 이동
2. Scripted grasp 시퀀스 실행
3. 광물 USD를 cargo bin으로 텔레포트
4. I4 (PickResponse)로 결과 응답

T2는 처리 중 새 PickRequest 받아도 큐잉하지 않고 무시 (FSM이 직렬 처리 보장).

---

## I4. `/mission/pick_response` (T2 → T3)

### 메시지 정의
[`msg/PickResponse.msg`](msg/PickResponse.msg)

### Status 값

| status | 의미 | T3 동작 |
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

## I5. `/rover/estimated_pose` (T5 → T3, T4)

### 메시지 정의
**ROS2 표준 사용**: `geometry_msgs/PoseWithCovarianceStamped`

```
std_msgs/Header header               # frame_id = "world"
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position       # x, y, z (m)
    geometry_msgs/Quaternion orientation
  float64[36] covariance               # 6x6 row-major, variance (not std)
                                       # [x, y, z, roll, pitch, yaw]
```

### T5 내부 구현 — TRN + Multi-sensor Fusion

T5는 다음 4개 입력을 EKF로 융합하여 estimated_pose 생성. **GT는 정책에 직접 사용하지 않음**.

```
4개 센서 입력 (모두 Isaac Sim 시뮬 노이즈 포함):
  ① Wheel Odometry      ← joint_vel 적분 + Ackermann 모델
  ② IMU Integration     ← 자이로 + 가속도 적분 (드리프트 누적)
  ③ Sun Yaw             ← 광원 방향 → 절대 방위 (드리프트 0)
  ④ TRN ⭐              ← 로컬 heightmap을 T1 global heightmap에 매칭
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
- T1의 global heightmap.npy와 cross-correlation 매칭
- 가장 잘 맞는 위치로 EKF drift 보정 (loop closure 역할)

### 발급 규칙

- **30 Hz publish** (PPO step rate 동기)
- Covariance: 시간 지남에 따라 증가 (드리프트 누적). TRN 보정 시 감소.
- `header.frame_id = "world"` 고정
- 빈 publish 안 함 (항상 최신 추정치)

### 단계별 구현 (Day별)

| Day | T5 구현 수준 | Covariance 동작 |
|:---:|------------|----------------|
| Day 1 | Stub: GT + 가우시안 노이즈 (간단) | 고정 |
| Day 2 | + Wheel/IMU 적분 | 누적 |
| Day 3 | + TRN 단독 검증 | TRN 신뢰도 기반 |
| Day 4 | EKF 융합 (4개 센서) | 동적 (TRN 보정 시 감소) |
| Day 5+ | 노이즈 파라미터 튜닝, 노이즈 σ별 sweep | |

### Consumer 동작
- **T3**: 모든 모듈에서 `pose_provider.get_pose()` 추상화 통해 사용
  - FSM: "is rover home?" 판정
  - Coverage: 미니맵 셀 방문 처리
  - PPO Wrapper: command_manager의 robot frame 변환 입력
- **T4 UI**: 추정 위치 + 공분산 타원 시각화

### Fallback 정책

T5가 publish 멈추면:
1. T3의 `PoseProvider`가 자동으로 GT stub 모드로 fallback
2. PM이 즉시 alert 받음
3. T5 재시작 후 자동 ROS2 모드 복귀

→ T5 불안정성이 T3 작업 막지 않음.

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
T1 Environment       (5060)     ___________  날짜 ____
T2 Perception+M0609  (5080)     ___________  날짜 ____
T3 Driving           (5080)     ___________  날짜 ____
T4 Integration+PM    (5070 Ti)  ___________  날짜 ____
T5 Localization+Infra(5080)     ___________  날짜 ____
```

---

## 📝 CHANGELOG

```
[Unreleased]
- Initial draft (Day 0)
- 5 critical interfaces defined

[YYYY-MM-DD]
- (변경 사항 기록)
```
