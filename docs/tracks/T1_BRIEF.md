# 🗺️ T1 (김현중) — Environment 브리프

> **담당자: 김현중** (트랙 owner)
> 이 문서를 읽고 나면: 무엇을 만들지, 왜 만들지, 어떻게 만들지, 언제까지 만들지 명확해집니다.

---

> 📦 **이 트랙이 작업하는 패키지 위치**: [PACKAGE_MAPPING.md](PACKAGE_MAPPING.md) 참조 (팀 레포 9개 패키지 중 어디서 코딩하는지 명시).


## 📑 목차

1. [왜 이 트랙이 프로젝트의 심장인가](#1-왜-이-트랙이-프로젝트의-심장인가)
2. [당신이 만들 것 — 한눈에](#2-당신이-만들-것--한눈에)
3. [meta.json — 전체 스키마](#3-metajson--전체-스키마)
4. [필드별 의미와 영향](#4-필드별-의미와-영향)
5. [생성 알고리즘 — 10단계](#5-생성-알고리즘--10단계)
6. [다른 트랙이 당신 출력을 어떻게 쓰나](#6-다른-트랙이-당신-출력을-어떻게-쓰나)
7. [일정과 마일스톤](#7-일정과-마일스톤)
8. [흔한 함정 (미리 피하기)](#8-흔한-함정-미리-피하기)
9. [도구와 참고 자료](#9-도구와-참고-자료)
10. [DoD — 완료 기준](#10-dod--완료-기준)

---

## 1. 왜 이 트랙이 프로젝트의 심장인가

### 프로젝트 당위성

**시뮬레이터의 본질적 가치 = 현실에서 불가능하거나 비싼 반복 작업을 가능하게 함.**

- 화성에 로버 1대 보내는 비용: 조 단위
- 화성 표면 상태는 미지 (정확한 사전 지도 X)
- 실제 임무 전, "다양한 화성 환경에서 robust한 정책"을 만들어야 함
- → **시뮬레이터에서 수천 가지 화성 환경을 생성하고 테스트**가 핵심

### 클론 프로젝트의 결정적 한계

기존 클론 프로젝트(June2December/RLRoverLab)는:
- ❌ 단 **1개의 고정 화성 지형**만 사용
- ❌ 같은 지형에서 학습 → 같은 지형에서 평가 (일반화 X)
- ❌ 시각만 화성, **물리는 지구 (gravity 9.81)**

→ "화성 시뮬"이 아니라 "지구 위에 화성 텍스처 입힌 평지"였음.

### 우리 프로젝트의 코어

> **김현중 (T1)이 진짜 화성 환경을 다양하게 만들어내는 게, 이 프로젝트가 클론과 본질적으로 다른 시뮬레이션 프로젝트가 되는 이유.**

당신(김현중 (T1))이 하는 일:
1. **절차생성으로 N개의 서로 다른 화성 지형** (Perlin noise 기반)
2. **진짜 화성 물리** (gravity 3.72, regolith 마찰)
3. **광물 / 암석 / 베이스캠프의 무작위 배치**
4. **난이도 점수화** (학습/평가 정량 분석)

이게 없으면:
- 발표에서 "왜 시뮬레이터를 쓰나?"에 답 못 함
- 학습된 정책의 일반화 평가 불가능
- 클론과 차별점 사라짐
- **물류창고 팀에게 밀림**

→ 김현중 (T1)의 결과물은 단순한 asset이 아니라 **프로젝트의 raison d'être**입니다.

---

## 2. 당신이 만들 것 — 한눈에

```
generated_terrains/
├── terrain_00001/
│   ├── terrain_only.usd        ← 지형 메쉬 (Isaac Sim 로드)
│   ├── rocks_merged.usd        ← 암석 메쉬 (Isaac Sim 로드)
│   ├── obstacle_grid.npy       ← A* path planner용 (이찬휘 (T3)가 사용)
│   ├── heightmap.npy           ← 정밀 높이맵 (이지민 (T5) TRN이 사용)
│   └── meta.json               ← 모든 메타데이터 (최진우/이찬휘/성선규/이지민이 사용)
├── terrain_00002/
│   └── ...
└── ...
```

**최소 30개 / 권장 50개 / stretch 100개**

생성 후:
- `train/` (학습용 70%) vs `holdout/` (평가용 30%)로 분리
- holdout은 "정책이 한 번도 본 적 없는 지형" — 일반화 평가용

### 추가로 만드는 것 (단순)

**Mars 물리 설정 (Tier 1)**: rover_env_cfg.py 한 줄 추가
```python
sim: SimCfg = SimCfg(
    gravity=(0.0, 0.0, -3.72),  # ← 화성 중력
    physx=PhysxCfg(...)
)
```

**베이스캠프 visual (Tier 1)**: 패드 + 돔 + 안테나 USD 합성 (~3시간)

---

## 3. meta.json — 전체 스키마

각 terrain 디렉터리에 들어갈 `meta.json`:

```json
{
  // ═══ A. 정체성 / 재현성 ═══
  "terrain_id": "terrain_00001",
  "version": "1.0",
  "seed": 12345,
  "generated_at": "2026-05-21T03:45:00",

  // ═══ B. 공간 정의 ═══
  "size_m": [50.0, 50.0],
  "resolution_m": 0.05,
  "origin": {"x": -25.0, "y": -25.0},

  // ═══ C. 생성 파라미터 (입력) ═══
  "generation_params": {
    "terrain": {
      "type": "perlin",
      "octaves": 4,
      "frequency": 0.08,
      "amplitude_m": 3.0,
      "lacunarity": 2.0,
      "persistence": 0.5
    },
    "rocks": {
      "count": 80,
      "size_range_m": [0.3, 1.5],
      "min_spacing_m": 1.0,
      "slope_threshold_deg": 25,
      "asset_pool": ["rock_4", "rock_7"]
    },
    "minerals": {
      "count": 12,
      "min_spacing_m": 3.0,
      "exclude_basecamp_radius_m": 5.0,
      "value_distribution": {
        "blue":   {"prob": 0.5, "score": 10},
        "red":    {"prob": 0.3, "score": 25},
        "yellow": {"prob": 0.2, "score": 50}
      }
    },
    "physics_zones": {
      "type": "noise_based",
      "noise_frequency": 0.04,
      "sand_threshold": 0.3
    }
  },

  // ═══ D. 런타임 데이터 (출력) ═══
  "spawn_locations": [
    {"x": 5.0, "y": 3.0, "z": 0.2, "yaw": 1.57, "group": "default"}
  ],
  "basecamp": {
    "center": {"x": 0.0, "y": 0.0},
    "radius": 3.0,
    "marker_usd": "basecamp_dome.usd",
    "visual_footprint_m": [3.0, 3.0],
    "marker_height_m": 5.5,

    "shape": null,
    "entry_points": [],
    "collision_usd_path": null
  },
  "minerals": [
    {"id": 1, "type": "blue_mineral", "position": {"x": 8.0, "y": 4.0, "z": 0.1}, "value": 10},
    {"id": 2, "type": "green_gas",    "position": {"x": -5.2, "y": 7.3, "z": 0.1}, "value": 25}
  ],
  "physics_zones": [
    {
      "type": "sand",
      "polygon": [[-10, -10], [10, -10], [10, 0], [-10, 0]],
      "static_friction": 0.30,
      "dynamic_friction": 0.25
    },
    {
      "type": "rocky",
      "polygon": [[-25, -25], [-10, -25], [-10, -10], [-25, -10]],
      "static_friction": 0.55,
      "dynamic_friction": 0.50
    }
  ],
  "minimap": {
    "grid_size": [25, 25],
    "cell_size_m": 2.0,
    "origin": {"x": -25.0, "y": -25.0}
  },

  // ═══ E. 사후 분석 (출력) ═══
  "difficulty": {
    "score": 0.35,
    "rock_density": 0.12,
    "max_slope_deg": 15.0,
    "mean_slope_deg": 4.2,
    "passable_ratio": 0.78,
    "longest_corridor_m": 18.5
  }
}
```

---

## 4. 필드별 의미와 영향

### A. 정체성 / 재현성

| 필드 | 의미 | 왜 중요한가 |
|------|------|------------|
| `terrain_id` | 고유 ID | 학습/평가 결과 추적 |
| `version` | 스키마 버전 | 추후 필드 추가 시 마이그레이션 |
| `seed` | PCG 시드 | **재현성 — RL 디버깅 생명선** |
| `generated_at` | 생성 timestamp | 디버깅용 |

→ `seed` 항상 기록. 같은 seed → 같은 지형. 학습 실패 케이스 재현 시 필수.

### B. 공간 정의

| 필드 | 의미 | 영향 |
|------|------|------|
| `size_m` | 지형 전체 크기 (X, Y) | 크면 → 학습 시간 ↑, 메모리 ↑ |
| `resolution_m` | heightmap 픽셀 크기 | 작으면 → 디테일 ↑, 메모리 ↑ |
| `origin` | 좌표 원점 (좌하단) | 모든 다른 좌표의 기준 |

**권장값** (변경 비추, 클론과 동일):
- `size_m: [50.0, 50.0]`
- `resolution_m: 0.05` → 1000×1000 heightmap

→ 더 크면 1M step 학습이 너무 오래 걸림. 50×50이 sweet spot.

### C. 생성 파라미터 ⭐ (난이도 결정 핵심)

#### C-1. Terrain (지형 형상) — Perlin Noise

```
heightmap = Σ_{i=0..octaves} (amplitude * persistence^i) × perlin(freq * lacunarity^i)
```

| 필드 | 효과 | 권장값 (중간 난이도) |
|------|------|---------------------|
| `octaves` | noise layer 개수 — 클수록 디테일 ↑ | 4 |
| `frequency` | 변동 주파수 — 클수록 굴곡 잦음 | 0.08 |
| `amplitude_m` | 높이 변동 폭 (m) — 클수록 절벽급 | 3.0 |
| `lacunarity` | octave 간 frequency 배율 | 2.0 |
| `persistence` | octave 간 amplitude 배율 | 0.5 |

**난이도별 설정**:

| 난이도 | octaves | frequency | amplitude | 효과 |
|:---:|:---:|:---:|:---:|------|
| **Easy** | 2 | 0.05 | 1.5 | 매끄러운 사구 |
| **Medium** | 4 | 0.08 | 3.0 | 자연스러운 화성 |
| **Hard** | 6 | 0.12 | 5.0 | 절벽 + 급경사 |

#### C-2. Rocks (장애물)

| 필드 | 효과 |
|------|------|
| `count` | 암석 총 개수 — 많을수록 회피 난이도 ↑ |
| `size_range_m` | [최소, 최대] 크기 — 크면 한 번에 우회 거리 ↑ |
| `min_spacing_m` | 암석 간 최소 거리 — 작으면 빽빽한 군집 (협곡) |
| `slope_threshold_deg` | 이 슬로프 이상엔 안 놓음 (굴러떨어질 곳 회피) |
| `asset_pool` | 사용할 USD 종류 — 시각적 다양성 |

**권장값**: count=80, size=[0.3, 1.5], spacing=1.0, slope_thr=25

#### C-3. Minerals (수집 대상)

| 필드 | 효과 |
|------|------|
| `count` | 광물 총 개수 — 미션 길이 결정 |
| `min_spacing_m` | 광물 간 최소 거리 |
| `exclude_basecamp_radius_m` | 베이스 근처 제외 → 탐사 강제 |
| `value_distribution` | **가치 점수 시스템** ⭐ — 최진우 (T2)의 우선순위 결정 |

**가치 분포 의미**:
```
blue   (50%, 10점)  ← 흔함
red    (30%, 25점)  ← 보통
yellow (20%, 50점)  ← 희귀
```

→ 최진우 (T2) vision이 같은 거리에 여러 광물 보면 **노랑 우선** 수집하도록 학습 가능.
→ 발표 멘트: *"단순 수집이 아니라 과학적 우선순위 기반 자율 탐사"*

#### C-4. Physics Zones (Mars Tier 2, 이지민 (T5)가 이어받음)

| 필드 | 효과 |
|------|------|
| `noise_frequency` | 모래/암반 패치 크기 (작으면 큰 영역) |
| `sand_threshold` | 모래 영역 비율 (높으면 모래 적음) |

→ 이 정보로 이지민 (T5)이 PhysX physics material 적용. 김현중 (T1)은 영역 폴리곤만 생성하고 넘김.

### D. 런타임 데이터 (출력)

#### spawn_locations
사전 검증된 안전한 로버 스폰 후보. 클론의 `random_rover_spawns` 함수 참고.

**검증 기준**:
- 베이스캠프 외부
- 평탄 (슬로프 < 15°)
- 암석 없음
- 경계에서 충분히 떨어짐

**개수**: 최소 50개 (학습 시 reset마다 무작위 선택)

#### basecamp (Tier 1)
- `center` + `radius` + `marker_usd` = 필수
- `shape`, `entry_points`, `collision_usd_path` = **null로 비워둠** (Tier 2용)
- **이 3개 optional 필드는 절대 삭제하지 말 것** — 후행 확장 호환성

#### minerals (출력)
generation_params로부터 알고리즘이 산출한 결과:
```json
{"id": 1, "type": "blue_mineral", "position": {...}, "value": 10}
```
- **id는 김현중 (T1)이 1부터 순차 발급** (최진우 (T2)가 이 ID로 detection 매칭)
- 광물 USD는 별도 디렉터리에 종류별 ({type}.usd: `blue_mineral.usd` / `yellow_mineral.usd` / `green_gas.usd`) 미리 준비
- type ∈ {`blue_mineral`, `yellow_mineral`, `green_gas`} — YOLO model.names 와 동일

#### physics_zones (출력)
generation_params의 noise로부터 추출한 폴리곤 리스트. 이지민 (T5)이 이걸 받아서 PhysX에 적용.

#### minimap
이찬휘 (T3) Coverage Planner가 사용. 25×25 grid, 셀당 2m → 50m 영역 커버.

### E. 사후 분석 (출력)

generation 끝난 후 메트릭 계산:

| 필드 | 계산 방법 |
|------|----------|
| `rock_density` | rock 개수 / map area |
| `max_slope_deg` | gradient의 max |
| `mean_slope_deg` | gradient의 mean |
| `passable_ratio` | (slope < 25° & not rock) / total cells |
| `longest_corridor_m` | 안전 영역의 connected component 최장 길이 |
| `score` | 위 메트릭들의 weighted sum (0~1 정규화) |

**계산 예시**:
```python
score = (
    0.3 * (rock_density / 0.3) +
    0.4 * (mean_slope_deg / 30) +
    0.3 * (1 - passable_ratio)
)
```

→ E 트랙(이지민, eval)이 holdout 분석할 때 사용. Curriculum 학습에도 사용.

---

## 5. 생성 알고리즘 — 10단계

```
Input: seed + generation_params (난이도별 preset)
   │
   ▼
[1] Heightmap 생성
    perlin_2d() × octaves → np.ndarray (1000, 1000)
   │
   ▼
[2] Slope 분석
    Sobel filter → slope_deg per cell
   │
   ▼
[3] 베이스캠프 좌표 결정
    기본 (0, 0). 추후 변경 시 generation_params에 추가
   │
   ▼
[4] 암석 배치 (rejection sampling)
    무작위 (x, y) → 슬로프·spacing·basecamp 조건 통과 시 채택
   │
   ▼
[5] 광물 배치 (rejection sampling)
    무작위 (x, y) → 암석·spacing·basecamp 조건 통과
    + value_distribution으로 type 확률적 결정
   │
   ▼
[6] Physics zones 생성
    2nd Perlin noise (지형과 독립) → threshold로 polygon 추출
   │
   ▼
[7] Spawn locations 후보 추출
    안전 영역 sampling → 50개 후보
   │
   ▼
[8] USD export
    heightmap → mesh (PyMeshLab) → terrain_only.usd
    암석들 → rocks_merged.usd
    베이스캠프 visual → basecamp_dome.usd (또는 합성)
   │
   ▼
[9] obstacle_grid.npy 생성
    heightmap 슬로프 + 암석 위치 → binary grid (A*용)
   │
   ▼
[10] 사후 분석 + meta.json export
     difficulty 메트릭 계산 → meta.json 저장
   │
   ▼
Output: 1개 terrain 디렉터리 완성
```

이걸 30~50번 반복하면 batch 완성.

---

## 6. 다른 트랙이 당신 출력을 어떻게 쓰나

| 트랙 (담당자) | 무엇을 읽나 | 어떻게 쓰나 |
|------|------------|-----------|
| **T2 (최진우) Perception** | `minerals` 배열, `terrain_only.usd` | detection ↔ ID 매칭 / Replicator 합성 데이터 생성 |
| **T2 (최진우) M0609** | `minerals[i].position` | pick 좌표 |
| **T3 (이찬휘) Coverage** | `minimap.grid_size`, `basecamp` | 미니맵 셀 초기화 / 베이스 영역 제외 |
| **T3 (이찬휘) A\*** | `obstacle_grid.npy`, `heightmap.npy` | path 계산 입력 |
| **T3 (이찬휘) FSM** | `basecamp.center`, `basecamp.radius` | "is rover home?" 판정 |
| **T3 (이찬휘) PPO** | `terrain_only.usd`, `rocks_merged.usd` | Isaac Sim 로드 (그대로) |
| **T4 (성선규) UI** | `minimap`, `basecamp`, `minerals` | 시각화 렌더링 |
| **T5 (이지민) Mars** | `physics_zones` | PhysX material 적용 |
| **T5 (이지민) Eval** | `difficulty`, `terrain_id` | 평가 분류 / 차트 |

→ **출력 호환성이 깨지면 4명이 막힘**. meta.json schema 변경 시 PM(성선규 (T4))에게 즉시 alert.

---

## 7. 일정과 마일스톤

```
Day 1 (화) ─ 환경 셋업 + 첫 PoC
  □ Python 환경 (numpy, pymeshlab, perlin-noise)
  □ Perlin noise heightmap 1장 생성
  □ USD export 1개 성공
  □ 클론의 terrain1을 분석하여 좌표계 / 단위 파악
  → EOD: terrain_00001/ 디렉터리에 USD 1세트 + meta.json 1장

Day 2 (수) ─ 파라미터화 + 베이스캠프 visual
  □ generation_params 클래스 정의
  □ 암석 / 광물 / spawn / physics_zones 알고리즘 구현
  □ 베이스캠프 visual USD 합성 (패드 + 돔 + 안테나)
  → EOD ⚠️ 게이트: "5개 시드로 5개 terrain 생성, 시각적으로 다름 확인"

Day 3-4 (목-금) ─ Batch 생성
  □ 난이도 preset (easy/medium/hard) 정의
  □ 30개 batch 생성 (10 easy + 15 medium + 5 hard)
  □ 사후 분석 difficulty score 검증
  □ train/holdout split (21/9)
  → EOD: generated_terrains/ 30개 + index.json

Day 5-6 (토-일) ─ 검증 + Mars Physics Tier 1
  □ Isaac Sim에서 5개 무작위 추출 → 로딩 테스트
  □ obstacle_grid가 실제 암석 위치와 일치하는지 검증
  □ rover_env_cfg.py에 gravity=(0,0,-3.72) 적용
  □ 학습 시작 전 PPO eval 1회 (지구 vs 화성 차이 확인)
  → 일요일 EOD ⚠️ 게이트: "End-to-end 데모 성공"

Day 7-8 (월-수AM) ─ 폴리싱 + 발표 보조
  □ 50개로 확장 (stretch)
  □ 시각화 보조 (terrain 갤러리 그리드 view)
  □ 발표 슬라이드 자료 협력 (E 트랙과)
  → 수요일 정오: final freeze
```

**Day 2 EOD 게이트 통과 못 하면**: Tier 2 (지형 다양성 stretch) 포기, 단순 변형만으로 진행.

---

## 8. 흔한 함정 (미리 피하기)

| 함정 | 증상 | 대응 |
|------|------|------|
| **Resolution 너무 작게 설정** | 메모리 폭발, USD 거대화 | 0.05 유지 (1000×1000). 더 작게 가지 마세요 |
| **암석 너무 많이** | PhysX 솔버 OOM, 시뮬 멈춤 | count ≤ 150. 그 이상은 시뮬 불안정 |
| **좌표계 혼동** | A*가 엉뚱한 곳 막음, 광물 못 찾음 | **모든 좌표는 world frame, origin = (-25,-25)** 통일 |
| **seed 재현 안 됨** | "어제 만든 거 다시 못 만듦" | `np.random.seed(seed)` 호출 잊지 않기. random 모듈은 별도 seed |
| **암석/광물 z 좌표 오류** | 공중에 떠 있거나 지면에 묻힘 | `heightmap_at(x, y) + offset` 으로 항상 계산 |
| **basecamp optional 필드 누락** | 후행 Tier 2 확장 시 schema 깨짐 | `shape: null`, `entry_points: []` 빈값이라도 **반드시 포함** |
| **PyMeshLab USD export 실패** | "지형 안 보임" | USD Python API 직접 사용으로 fallback. 미리 1세트 검증 |
| **Mars 중력 적용 안 함** | "Earth vs Mars 비교" 시 차이 없음 | rover_env_cfg.py 수정 후 학습 재시작 |

---

## 9. 도구와 참고 자료

### Python 라이브러리

```bash
pip install numpy scipy noise pymeshlab opencv-python
```

- `noise` 또는 `perlin-noise` — Perlin / Simplex noise 생성
- `pymeshlab` — heightmap → triangle mesh
- `opencv-python` — gradient 계산, morphology 연산
- `scipy.ndimage` — connected components, dilation

### Isaac Sim USD API

```python
from pxr import Usd, UsdGeom, Gf, Sdf

stage = Usd.Stage.CreateNew("terrain_only.usd")
mesh = UsdGeom.Mesh.Define(stage, "/Terrain")
mesh.CreatePointsAttr(vertices)
mesh.CreateFaceVertexIndicesAttr(face_indices)
mesh.CreateFaceVertexCountsAttr([3] * len(face_indices)//3)
stage.GetRootLayer().Save()
```

### 클론의 참고 파일

- [terrain_utils.py](../rover/sim/rover_envs/envs/navigation/utils/terrains/terrain_utils.py) — heightmap 처리, spawn 생성 함수
- [mars_terrains.py](../rover/sim/rover_envs/assets/terrains/mars/mars_terrains.py) — 기존 terrain scene 구성
- [rover_env_cfg.py](../rover/sim/rover_envs/envs/navigation/rover_env_cfg.py) — SimCfg 위치 (gravity 추가할 곳)
- [terrain1/](../rover/sim/rover_envs/assets/terrains/mars/terrain1/) — 기존 단일 terrain 예시

### 외부 자료

- [Perlin Noise 시각화](https://rtouti.github.io/graphics/perlin-noise-algorithm)
- [Isaac Sim USD Procedural Generation](https://docs.isaacsim.omniverse.nvidia.com/latest/replicator_tutorials/index.html)
- [NASA Mars Terrain Reference](https://www.nasa.gov/mars-exploration-program/) — 실제 화성 지형 사진 참고

---

## 10. DoD — 완료 기준

### 최소 (Day 6 EOD)
- ✅ 30개 terrain 디렉터리 생성 완료
- ✅ 각 디렉터리에 5개 필수 파일 (USD ×2, npy ×2, json ×1)
- ✅ meta.json schema 검증 통과 (별도 `validate_meta.py` 스크립트로)
- ✅ Isaac Sim에서 5개 무작위 로드 성공
- ✅ Mars gravity 3.72 적용 확인
- ✅ train/holdout split (21개 / 9개)

### 권장 (Day 7-8)
- ✅ 50개로 확장
- ✅ 난이도 분포 균등 (easy 30% / medium 50% / hard 20%)
- ✅ 베이스캠프 visual (패드 + 돔 + 안테나) USD
- ✅ Earth vs Mars 비교 데모용 baseline 데이터

### Stretch
- ⏳ 100개 batch
- ⏳ 4개 난이도 (very easy 추가)
- ⏳ Replicator 통합 (텍스처 randomization)
- ⏳ Mars Tier 2 적용 PPO 재학습 1회

---

## 🤝 Sync 포인트

- **매일 09:30 standup**: 어제 진척, 오늘 계획, 블로커 공유
- **매일 18:00 DIST** (Daily Integration Smoke Test): 본인 결과물 + 다른 트랙 통합 검증
- **인터페이스 변경 발생 시 즉시**: PM(T4 (성선규))에게 alert → 전체 회의

---

## 💪 마지막 한 마디

T1이 빠르면 모든 트랙이 빠르고, T1이 느리면 모든 트랙이 막힙니다. **Day 2 EOD까지 5개 시드 5개 terrain**이 첫 관문. 거기 도달하면 8일 안에 완성 거의 확실.

질문 / 결정 필요 사항은 망설이지 말고 PM에게 즉시 ping. 혼자 고민하다 1시간 소실 vs PM에게 5분 물어보고 해결 — 후자 선택.

화이팅 🚀
