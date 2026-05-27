# 레이캐스트 기반 로버 위치 보정 시스템 설계 문서

> T3 Driving | 작성일: 2026-05-27  
> 담당: 이찬휘  
> 대상 파일: `isaac_drive/raycast_map_viewer.py`, `isaac_sim/scripts/run_vehicle_v3.py`

---

## 1. 문제 정의 — 왜 위치 보정이 필요한가?

### 1-1. EKF(Extended Kalman Filter) 드리프트

로버의 현재 위치는 `/rover/estimated_pose` 토픽으로 받아온다.  
이 값은 **EKF가 출력하는 추정 위치**로, 내부적으로 다음 센서를 퓨전한다:

| 센서 | 역할 |
|------|------|
| 바퀴 인코더 (Wheel Odometry) | 이동 거리 추정 |
| IMU 각속도 (Gyro) | 회전량 적분 |

두 센서 모두 **상대적 누적 계산** 방식이기 때문에, 시간이 지날수록 오차가 쌓인다.  
이를 **드리프트(drift)** 라고 하며, 실내/실외 장거리 주행 시 수 미터의 오차가 발생할 수 있다.

```
실제 위치 (Ground Truth):  (+12.50,  +8.30)
EKF 추정 위치:             (+11.80,  +9.10)
오차 벡터 (Δ):             (- 0.70,  +0.80)  → |Δ| ≈ 1.06 m
```

### 1-2. 드리프트의 영향

- **커버리지 주행**: 이미 지나간 영역을 다시 주행하거나, 미도달 영역 발생
- **채집 임무**: 목표 지점 도달 실패
- **장애물 회피**: 장애물 위치를 잘못 추정하여 충돌 위험

### 1-3. 목표

> "EKF 위치(파란 삼각형)를 맵의 장애물 정보를 이용해 실제 위치에 가깝게 보정한다.  
>  보정된 위치를 노란 삼각형으로 표시한다."

---

## 2. 시스템 아키텍처

### 2-1. 전체 구성 (4-터미널)

```
┌─────────────────────────────────────────────────────────┐
│  터미널 1: Isaac Sim (run_vehicle_v3.py)                 │
│    - PhysX 레이캐스트 격자 계산                           │
│    - IPC 파일로 결과 저장: /tmp/a2_raycast.npz           │
│    - /ground_truth/odom 퍼블리시 (실제 절대좌표)         │
└──────────────────────┬──────────────────────────────────┘
                       │ IPC 파일 (/tmp/a2_raycast.npz)
┌──────────────────────▼──────────────────────────────────┐
│  터미널 2: ROS2 스택 (integrated_localization.launch.py) │
│    - EKF 위치 추정 → /rover/estimated_pose              │
│    - 커버리지 주행, 미션 매니저, YOLO 인식               │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│  터미널 3: raycast_relay_node                            │
│    - /tmp/a2_raycast.npz 읽기                           │
│    - OccupancyGrid 누적 맵 생성                          │
│    - /rover/raycast/built_grid 퍼블리시                  │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│  터미널 4: raycast_map_viewer (핵심)                     │
│    - 3개 OccupancyGrid + EKF pose + GT odom 구독        │
│    - 장애물 패턴 매칭으로 위치 보정 계산                  │
│    - matplotlib 실시간 시각화                            │
└─────────────────────────────────────────────────────────┘
```

### 2-2. IPC 데이터 구조 (`/tmp/a2_raycast.npz`)

Isaac Sim(Python 3.11)과 ROS2(Python 3.10)는 직접 통신이 불가능하므로  
NumPy 바이너리 파일(.npz)을 공유 메모리처럼 사용한다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `cell_world_xyz` | `(N, 3) float32` | 각 레이캐스트 셀의 **월드 절대좌표** |
| `cell_is_obs` | `(N,) bool` | 해당 셀이 장애물인지 여부 |
| `cell_is_miss` | `(N,) bool` | 레이가 바닥에 닿지 않은 셀 (낭떠러지) |
| `rover_pos` | `(3,) float32` | 로버 **실제 절대좌표** (Isaac Sim Ground Truth) |
| `rover_yaw` | `float` | 로버 실제 yaw |
| `grid_rows`, `grid_cols` | `int` | 격자 크기 (41×41) |

---

## 3. 레이캐스트 시스템

### 3-1. 파라미터

```python
RAYCAST_SIZE = (8.0, 8.0)   # 로버 기준 전후좌우 8m × 8m 범위
RAYCAST_RES  = 0.2           # 셀 간격 0.2m → 41 × 41 = 1,681개 레이
RAYCAST_POINT_WIDTH = 0.0    # 뷰포트 시각화 비활성 (투명)
```

### 3-2. 동작 원리

```
로버 중심에서 yaw 방향 정렬된 격자(grid)를 구성
  ↓
각 격자 셀마다 PhysX 레이캐스트: 셀 위치에서 아래(-Z) 방향으로 발사
  ↓
지면에 닿으면 높이 기록, 안 닿으면 "miss"(낭떠러지 의심)
  ↓
8-이웃 평균 높이와 비교해 prominence(돌출도) 계산
  ↓
prominence > threshold → 장애물(obstacle) 판정
  ↓
cell_world_xyz: 이 셀의 월드 절대좌표로 변환해 IPC에 저장
```

### 3-3. OccupancyGrid 누적

`raycast_relay_node`가 IPC를 읽어 200×200 격자(0.25m/셀, 50×50m 맵)에 누적:
- 장애물 셀: 값 증가 (최대 100)
- 빈 공간: 값 감소
- 미수신: -1 (unknown)

→ `/rover/raycast/built_grid` 토픽으로 발행

---

## 4. 위치 보정 알고리즘

### 4-1. 핵심 아이디어

> **레이캐스트로 감지한 장애물들의 배치 패턴(군집 형태)을  
>  원본 맵(`/map`)에서 찾아 내 실제 위치를 역산한다.**

```
레이캐스트 장애물 (세계좌표): R1(5.2, 3.1), R2(5.8, 4.3), R3(4.9, 5.0)
          ↓   패턴 매칭
원본 맵 장애물:              M1(5.3, 3.2), M2(5.9, 4.2), M3(5.0, 5.1)
          ↓
오프셋 = M - R = (+0.1, +0.1)
          ↓
보정된 로버 위치 = EKF 위치 + 오프셋
```

### 4-2. 1단계: 검색 영역 특정 (EKF 위치 기준)

EKF 위치가 정확하지 않더라도 **대략적인 위치는 알고 있다**는 전제.

```python
# 레이캐스트 장애물: EKF 위치 기준 반경 4.0m 내에서 검색
ray_list = _nearby_obstacles(built_grid, center=ekf_pos, radius=4.0m)

# 원본 맵 장애물: 패턴 매칭용으로 더 넓은 범위 검색
# (EKF 오차 최대 3m를 감안해 4 + 3 = 7m)
match_search_r = 4.0 + 3.0  # = 7.0m
orig_list_match = _nearby_obstacles(orig_map, center=ekf_pos, radius=7.0m,
                                    top_n=15, dedup=0.32m)
```

이 단계에서 "나는 EKF 위치 ±3m 안에 있다"고 가정하고 장애물 후보를 좁힌다.

### 4-3. 2단계: 패턴 매칭 (constellation matching)

**단순 최근접 매칭의 문제점:**
```
방식: 각 레이캐스트 장애물의 가장 가까운 맵 장애물을 찾아 오프셋 평균
문제: 노이즈 장애물 1개가 엉뚱한 맵 장애물에 매칭 → 평균값 왜곡
```

**패턴 매칭 방식 (anchor-based brute force):**

```
FOR 모든 (raycast_i, map_j) 앵커 쌍:
    shift = map_j - raycast_i  # 이 쌍이 일치한다고 가정할 때의 이동량

    IF |shift| > max_correction_mag(3.0m): SKIP

    score = 0
    FOR 나머지 레이캐스트 장애물 ray_k:
        shifted_ray_k = ray_k + shift
        nearest_map = 가장 가까운 맵 장애물 (이미 사용된 것 제외)
        IF distance(shifted_ray_k, nearest_map) < match_thresh(2.0m):
            score += 1

    IF score > best_score:
        best_score = score
        best_shift = 일치 쌍들의 오프셋 평균  ← 더 정밀하게 계산

IF best_score >= min_matches(2):
    correction = best_shift
    yellow = EKF + correction
```

**패턴 매칭이 강건한 이유:**
- 노이즈 장애물 1개가 엉뚱한 맵 장애물에 매칭되더라도,  
  나머지 장애물들도 같은 shift에서 매칭되어야만 score가 높아진다
- 즉, **전체 패턴이 일관되게 맞아야** 채택된다 (다수결 원리)

```
예시:
  레이캐스트: R1, R2, R3, R_noise

  앵커 (R_noise, M_엉뚱한):
    shift = M_엉뚱한 - R_noise
    R1+shift → 맵에 없음 (✗)
    R2+shift → 맵에 없음 (✗)
    R3+shift → 맵에 없음 (✗)
    score = 0  ← 채택 안 됨

  앵커 (R1, M1):
    shift = M1 - R1 = (+0.1, +0.1)
    R2+shift → M2 근처 (✓)
    R3+shift → M3 근처 (✓)
    R_noise+shift → 맵에 없음 (✗)
    score = 3  ← 최고 score, 채택!
```

### 4-4. 파라미터 정리

| 파라미터 | 기본값 | 의미 |
|----------|--------|------|
| `nearby_radius_m` | 4.0 m | 레이캐스트 장애물 표시 반경 |
| `nearby_radius_orig_m` | 4.0 m | 원본 맵 장애물 표시 반경 |
| `match_search_r` | 7.0 m | 패턴 매칭용 원본 맵 탐색 반경 |
| `max_match_dist_m` | 2.0 m | 장애물 매칭 허용 거리 (match threshold) |
| `max_correction_mag_m` | 3.0 m | 보정값 최대 허용 크기 |
| `nearby_top_n` | 3 | 표시할 최근접 장애물 수 |
| `nearby_dedup_m` | 0.8 m | 장애물 중복 제거 거리 |
| `min_matches` | 2 | 패턴 매칭 최소 일치 쌍 수 |

---

## 5. 시각화 (matplotlib 실시간 뷰어)

### 5-1. 마커별 의미

| 마커 | 색상 | 소스 토픽 | 의미 |
|------|------|-----------|------|
| ▲ 삼각형 (청록) | `#4fd1e1` | `/rover/estimated_pose` | EKF 추정 위치 (드리프트 있음) |
| ✕ X 마커 | `#4fd1e1` | `/ground_truth/odom` | Isaac Sim 실제 절대좌표 |
| ▲ 삼각형 (노란) | `#ffd24a` | 내부 계산 | 패턴 매칭 보정 위치 |
| ━ 궤적 (노란) | `#ffd24a` | EKF 누적 | 주행 경로 |

> **파란 삼각형(EKF)과 X(GT)의 거리** = 현재 EKF 드리프트 크기  
> **X(GT)와 노란 삼각형의 거리** = 보정 알고리즘의 오차

### 5-2. 좌상단 텍스트 패널

```
═══ ROVER POSITION ═══
EKF   x=+12.34  y= +8.56  yaw= +23.4°

═══ /map obstacles (≤4.0m) ═══
  1: (+11.20,  +7.40)  d= 1.62m
  2: (+13.10,  +9.20)  d= 1.98m

═══ raycast obstacles (≤4.0m, matched #) ═══
  1: (+11.28,  +7.35)  d= 1.55m    ← 원본 맵 M1에 매칭됨
  ?: (+14.50, +10.00)  d= 3.01m    ← 매칭 실패 (노이즈)

═══ POSE CORRECTION (pattern match) ═══
Δ     x= +0.08  y= -0.05  |Δ|= 0.09m  (score=2)
→ 추정위치  x=+12.42  y= +8.51
```

### 5-3. RViz 연동

`/rover/raycast/corrected_marker` (Marker, ARROW 타입)를 퍼블리시하여  
RViz에서도 노란 화살표로 보정 위치를 확인할 수 있다.

---

## 6. 알고리즘 진화 과정

### 6-1. 1차 시도: 단순 최근접 매칭 (Simple Nearest-Neighbor)

```python
def _compute_correction(orig_list, ray_list, max_match_dist, max_correction_mag):
    diffs = []
    for rx, ry in ray_list:
        nearest = min(orig_list, key=lambda m: hypot(m.x-rx, m.y-ry))
        if distance(nearest, ray) <= max_match_dist:
            diffs.append((nearest.x - rx, nearest.y - ry))
    correction = mean(diffs)
```

**문제점:**  
- 레이캐스트 노이즈 장애물이 엉뚱한 맵 장애물과 매칭되면 평균이 왜곡됨  
- 장애물이 1-2개뿐인 경우 단순 최근접은 잘못된 쌍에도 이동해버림  
- 어느 맵 장애물에 매칭할지 **문맥 없이 개별적으로** 결정 → 전체 일관성 없음

### 6-2. 2차 시도: 패턴 매칭 (Anchor-based Constellation Matching) ← 현재

```python
def _pattern_match_correction(orig_list, ray_list, match_thresh, max_correction_mag):
    best_score = 0
    for ray_i, map_j as anchor:
        shift = map_j - ray_i
        score = count(ray_k where (ray_k + shift) near some map_obs)
        if score > best_score:
            best_score = score
            best_shift = mean(consistent offsets)
    return best_shift if best_score >= min_matches
```

**개선 이유:**
- 모든 가능한 앵커 쌍을 시도해 **전체 패턴이 일관된 shift만 채택**
- 노이즈 장애물 1개가 score에 미치는 영향 최소화
- 일치 쌍들의 오프셋을 평균해 **서브-셀 정밀도** 확보

---

## 7. 보정 정확도의 한계와 향후 과제

### 7-1. 현재 한계

| 한계 | 원인 | 영향 |
|------|------|------|
| 장애물 수 부족 | 평탄한 지형에서 레이캐스트 장애물 감지 부족 | score < 2 → 보정 불가 |
| OccupancyGrid 해상도 | 0.25m/셀 → 위치 오차 ±0.125m | 보정 정밀도 제한 |
| EKF 오차가 너무 클 때 | 드리프트 > 3m → max_correction_mag 초과 | 보정 거부 |
| 동적 장애물 | 실시간 장애물은 원본 맵에 없음 | 잘못된 매칭 |

### 7-2. 향후 개선 방향

1. **회전 보정 추가**: 현재는 translation(평행이동)만 보정, yaw 오차도 보정
2. **ICP(Iterative Closest Point)**: 더 많은 반복으로 정밀 정렬
3. **신뢰도 기반 가중 평균**: 가까운 장애물일수록 가중치 높게
4. **칼만 필터 피드백**: 보정값을 `/initialpose`로 EKF에 직접 재주입
5. **레이캐스트 해상도 향상**: 0.2m → 0.1m (4배 더 많은 레이)

---

## 8. 파일 구조 요약

```
a2_isaac/
├── isaac_sim/scripts/
│   └── run_vehicle_v3.py          # PhysX 레이캐스트 + IPC 저장
│       ├── RAYCAST_SIZE = (8.0, 8.0)
│       ├── RAYCAST_RES  = 0.2
│       └── /tmp/a2_raycast.npz 생성
│
└── isaac_drive/isaac_drive/
    ├── raycast_relay_node.py      # IPC → OccupancyGrid 변환 + 퍼블리시
    └── raycast_map_viewer.py      # 시각화 + 위치 보정 계산 (핵심)
        ├── _nearby_obstacles()    # 반경 내 장애물 추출 (dedup 포함)
        ├── _pattern_match_correction()  # 패턴 매칭 보정 알고리즘
        └── _render()              # 500ms 주기 matplotlib 갱신
```

---

## 9. 실행 명령

```bash
# 터미널 1 — Isaac Sim
<isaac-python> isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00023

# 터미널 2 — ROS2 전체 스택
ros2 launch isaac_bringup integrated_localization.launch.py collection_goal:=100

# 터미널 3 — 레이캐스트 릴레이
ros2 run isaac_drive raycast_relay_node

# 터미널 4 — 시각화 + 위치 보정 뷰어
ros2 run isaac_drive raycast_map_viewer
```

---

*본 문서는 2026-05-27 개발 세션 기준으로 작성되었습니다.*
