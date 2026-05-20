# 🗺️ I1 — Terrain Assets 풀 가이드

> I1 (Terrain Asset) 인터페이스의 모든 산출물 + master scene composition.
> [INTERFACE_CONTRACTS.md I1](INTERFACE_CONTRACTS.md#i1-terrain-asset-t1-김현중--모두) 계약 + 실제 파일 구조의 통합 설명.
> 생산자: **김현중 (T1)** · 소비자: 최진우/이찬휘/성선규/이지민 + Isaac Sim

---

## 디렉터리 트리 (실제 상태)

```
isaac_sim/
├─ assets/
│  ├─ generated_terrains/             ← I1 계약 디렉터리 (terrain별 데이터)
│  │  ├─ index.json                   # 루트, train/holdout split
│  │  └─ terrain_NNNNN/               # terrain 1장 = 50m × 50m 화성 표면
│  │     ├─ terrain_only.usd          # 지형 메쉬 (Isaac Sim 시각/물리)
│  │     ├─ rocks_merged.usd          # 80 sphere 암석 (Isaac Sim)
│  │     ├─ obstacle_grid.npy         # [이찬휘 T3] A* 통과/불통과
│  │     ├─ heightmap.npy             # [이지민 T5] TRN cross-correlation ⭐
│  │     └─ meta.json                 # 모두 — 광물/베이스캠프/스폰/난이도
│  │
│  └─ markers/                        ← 모든 terrain이 공유하는 USD
│     ├─ mineral_blue.usd             # [최진우 T2] HSV detection target
│     ├─ mineral_red.usd
│     ├─ mineral_yellow.usd
│     └─ basecamp_dome.usd            # [이찬휘 T3 FSM] "is rover home?" 시각
│
└─ worlds/
   ├─ terrain_NNNNN.usd               ← terrain별 master scene (보존, 덮어쓰기 X)
   └─ mars_exploration_world.usd      ← 최신 terrain alias (Isaac Sim 기본 entry point)
```

**구분이 중요**:
- `generated_terrains/terrain_NNNNN/` = **terrain별 데이터**. terrain 1000개 만들면 1000개 디렉터리
- `markers/` = **모든 terrain이 공유하는 USD**. 1000개 terrain이 같은 mineral_blue.usd를 reference
- `worlds/mars_exploration_world.usd` = **master scene**. 위 두 디렉터리의 USD를 reference로 묶은 entry point

---

## 5개 파일 한눈 요약

| 파일 | 형식 | 크기 (샘플) | Primary consumer | 무엇을 표현 |
|------|-----|:----------:|---|---|
| `terrain_only.usd` | USD Mesh | 1.4 MB | Isaac Sim | 200×200 polygon mesh (Mars 색 + 굴곡) |
| `rocks_merged.usd` | USD Xform + 80 Sphere | 5.8 KB | Isaac Sim | 80개 암석 위치/크기 |
| `obstacle_grid.npy` | (1000,1000) int8 | 1.0 MB | 이찬휘 T3 A* | 통과 가능/불가 2D 격자 |
| `heightmap.npy` | (1000,1000) float32 | 4.0 MB | 이지민 T5 TRN | 픽셀별 높이 (m) |
| `meta.json` | JSON | 9.9 KB | 4명 모두 | 광물/베이스/스폰/난이도 (의미론) |

각 파일 상세:
- **meta.json**: [META_JSON_FIELDS.md](META_JSON_FIELDS.md) 통짜 주석 가이드
- **나머지 4개**: 본 문서 아래 §3~§6
- **계약(누가 publish/subscribe)**: [INTERFACE_CONTRACTS.md I1](INTERFACE_CONTRACTS.md#i1-terrain-asset-t1-김현중--모두)

---

## ⭐ Master scene composition

`mars_exploration_world.usd` (2.5 KB)는 직접 geometry를 들고 있지 않음 — **4종류 USD를 reference로 합치는 지시서** + 자체 조명.

### Reference 그래프

```
                  mars_exploration_world.usd  (2.5 KB)
                  ─────────────────────────
                  │  /World (defaultPrim)
                  │  ├─ /World/Terrain       ──ref──→ ../assets/generated_terrains/terrain_00001/terrain_only.usd
                  │  ├─ /World/Rocks         ──ref──→ ../assets/generated_terrains/terrain_00001/rocks_merged.usd
                  │  ├─ /World/Basecamp      ──ref──→ ../assets/markers/basecamp_dome.usd
                  │  ├─ /World/Minerals/
                  │  │   ├─ mineral_01_blue   ──ref──→ ../assets/markers/mineral_blue.usd     (translate id=1 위치)
                  │  │   ├─ mineral_02_yellow ──ref──→ ../assets/markers/mineral_yellow.usd   (translate id=2 위치)
                  │  │   ├─ mineral_03_red    ──ref──→ ../assets/markers/mineral_red.usd      (...)
                  │  │   └─ ... (12개 instance)
                  │  └─ /World/Lights/
                  │      ├─ Sun (DistantLight)         ← reference 아님, master 안에 직접 정의
                  │      └─ Sky (DomeLight)            ← 동일
```

### 우리 첫 샘플 (terrain_00001) 의 광물 분포

| Mineral USD | 사용 횟수 | 광물 id |
|---|:---:|---|
| `mineral_blue.usd` | 4번 | id 1, 7, 9, 11 |
| `mineral_red.usd` | 2번 | id 3, 5 |
| `mineral_yellow.usd` | 6번 | id 2, 4, 6, 8, 10, 12 |
| **합계** | **12** | — |

3종류의 USD를 12번 reference (instance 효과). 위치는 각 reference 위에 `translate` 메타로 덮어씀.

### Isaac Sim이 master를 열 때 (composition 결과)

```
mars_exploration_world.usd 열기 (2.5 KB)
  │
  │  Isaac Sim의 USD composition engine이 reference 체인 resolution:
  │
  ├─ terrain_only.usd (1.4 MB)   로드 → /World/Terrain 자리에 mesh 합성
  ├─ rocks_merged.usd (5.8 KB)   로드 → /World/Rocks 자리에 80 sphere 합성
  ├─ basecamp_dome.usd (1.3 KB)  로드 → /World/Basecamp 자리에 pad+dome+antenna 합성
  ├─ mineral_blue.usd (931 B)    로드 → 4개 instance 합성 (translate 적용)
  ├─ mineral_red.usd (930 B)     로드 → 2개 instance
  └─ mineral_yellow.usd (933 B)  로드 → 6개 instance
  
  + 자체 조명 2개 (Sun + Sky)
  = Isaac Sim viewport에 완전한 화성 scene 렌더링
```

→ master 파일 자체는 2.5 KB로 작음. 무거운 데이터는 다 reference된 파일들에 있음.

### 왜 이렇게 분리했나

| 장점 | 설명 |
|------|-----|
| **markers/ 1개만 바꾸면 모든 terrain 자동 갱신** | 김현중이 mineral_blue.usd를 멋진 모형으로 교체 → 1000개 terrain × 평균 12광물 = 12000번 reference가 모두 새 모양. master scene/generator 코드 수정 불필요 |
| **terrain별 독립** | terrain_00002의 generated_terrains/에 USD를 다시 생성 — markers/는 그대로 공유 |
| **master scene 가벼움** | 2.5 KB라 git diff에서 변경사항 파악 쉬움 (큰 binary 안 들고 있음) |
| **Isaac Sim composition 강력함** | reference만 바꾸면 시각 즉시 갱신, reload만 하면 됨 |

---

## 3. `terrain_only.usd` 상세

### 한 줄
**Isaac Sim이 시각/PhysX collision으로 사용하는 polygon mesh.** heightmap.npy를 stride 다운샘플 → triangle mesh.

### 구조
```
/Terrain (Xform)                      ← root (reference 호환 위해)
  └─ /Terrain/TerrainMesh (Mesh)
       ├─ points              ~40,000 vertices (200×200, stride=5)
       ├─ faceVertexIndices   ~80,000 triangles (199×199 × 2)
       ├─ faceVertexCounts    [3, 3, 3, ...] 전부 3
       ├─ normals             per-face (uniform interpolation)
       ├─ extent              bbox for frustum culling
       ├─ subdivisionScheme   "none" (polygon, subdivision X)
       ├─ doubleSided         True
       └─ displayColor        (0.78, 0.45, 0.30) Mars red-orange
```

### Stride와 trade-off
1000×1000 heightmap을 통째 mesh로 만들면 1M vertex라 무거움. `mesh_stride` 파라미터로 다운샘플.

| stride | mesh size | USD 파일 | 시각 디테일 |
|:------:|:---------:|:--------:|:----------:|
| 2 | 500×500 = 250K vert | ~9 MB | 매우 세밀 |
| **5 (현재)** | 200×200 = 40K vert | **1.4 MB** | 충분 |
| 10 | 100×100 = 10K vert | 0.4 MB | 거칠 |

⚠️ **npy는 항상 풀해상도 1000×1000 유지**. mesh는 시각/충돌용 다운샘플일 뿐.

### 생성 알고리즘 ([world_composer.py](../../isaac_sim/scripts/world_composer.py))
heightmap[i, j]를 stride 간격으로 샘플링 → vertex 배열 → 각 quad(4 vertex)를 2개 triangle로 분할 → per-face normal 계산.

### 디버깅 히스토리 (왜 이 구조인지)
초기엔 mesh가 안 보였음. 3가지 fix 적용 후 해결:
1. `subdivisionScheme="none"` 명시 (기본 catmullClark이라 깨졌었음)
2. `extent` bbox 추가 (frustum culling 정상화)
3. `/Terrain (Xform) → /Terrain/TerrainMesh (Mesh)` 구조로 root를 Xform으로 (Mesh를 root로 두면 reference 시 type mismatch)

---

## 4. `rocks_merged.usd` 상세

### 한 줄
**80개 sphere prim 모음.** 현재 단순 sphere 더미 — 김현중 합류 후 진짜 rock mesh USD pool로 교체할 자리.

### 구조
```
/Rocks (Xform)
  ├─ /Rocks/rock_000 (Sphere)
  │   ├─ radius = 0.15~0.75       (meta.json rocks.size_range_m / 2)
  │   ├─ translate (x, y, z)      z = heightmap_at(x,y) + size*0.5
  │   └─ displayColor (0.45, 0.30, 0.25)  진한 갈색
  ├─ /Rocks/rock_001 (Sphere)
  └─ ... rock_079
```

### z 계산
`z = heightmap_at(x, y) + size * 0.5`
- 반지름만큼 떠 있음 = 지면에 박혀있는 시각 효과
- 우리 generator는 sphere라 size/2가 반지름

### 향후 (김현중)
[world_composer.py의 export_rocks_usd()](../../isaac_sim/scripts/world_composer.py)를 reference 방식으로 수정:
```python
# 지금:  UsdGeom.Sphere.Define(stage, ...) → 직접 sphere prim
# 후행:  markers/rock_default.usd를 reference (mineral과 같은 패턴)
```

---

## 5. `obstacle_grid.npy` 상세

### 한 줄
**2D binary grid (1000×1000 int8).** "여기 통과 가능? Y/N"만 알려주는 가장 단순한 형식.

### 형식
```python
shape: (1000, 1000)
dtype: int8
값:
  0 = safe (통과 가능)
  1 = obstacle (큰 바위 또는 절벽)
```

### 좌표 매핑 ⭐
```python
grid[i, j] = world position (origin.x + j*resolution, origin.y + i*resolution)
             └─ i = y축 (row, 0부터 위→아래)
             └─ j = x축 (col, 0부터 왼→오른쪽)
             └─ row-major
```

역변환:
```python
def world_to_cell(x, y):
    j = int((x - origin.x) / resolution)   # x → j
    i = int((y - origin.y) / resolution)   # y → i
    return i, j
```

### 생성 알고리즘
```
grid = (slope_deg > 25°)                  # 가파른 곳
       OR
       dilate(rock 위치, radius=size/res) # 암석 + 약간 여유
```

slope-based + rock-based의 OR. 우리 샘플에서 12.4%가 obstacle (124K cells).

### 사용 예 (이찬휘 T3)
```python
import numpy as np
import pyastar2d

grid = np.load("terrain_00001/obstacle_grid.npy")
weights = grid.astype(np.float32) * 1e6 + 1.0   # 1=normal, 1e6=blocked

start = (500, 500)   # world (0, 0)
goal  = (580, 660)   # world (4, 8)  — 주의: i=y(4), j=x(8)
path = pyastar2d.astar_path(weights, start, goal, allow_diagonal=True)
```

---

## 6. `heightmap.npy` 상세 ⭐ 가장 중요한 좌표 합의

### 한 줄
**2D 높이 지도 (1000×1000 float32).** 이지민 T5 TRN cross-correlation의 입력. terrain_only.usd의 원본 데이터.

### 형식
```python
shape: (1000, 1000)
dtype: float32
값: 각 cell의 높이 (m, world Z)
샘플 range: [-1.685, 1.642] m
```

### 좌표 매핑 (obstacle_grid와 동일)
```python
heightmap[i, j] = world Z at (origin.x + j*res, origin.y + i*res)
                  i = y, j = x, row-major
```

→ **obstacle_grid.npy와 shape/origin/resolution 완전 동일**. 같은 인덱스로 양쪽 접근 가능.

### 생성 알고리즘
```python
for each (i, j):
    x_world = j * resolution                          # j → world x
    y_world = i * resolution                          # i → world y
    h = pnoise2(x_world * freq, y_world * freq,       # frequency는 world meter 단위!
                octaves=4, persistence=0.5, lacunarity=2.0)
    heightmap[i, j] = h * amplitude_m
```

### 디버깅 히스토리 — Perlin frequency 단위
초기엔 `pnoise2(j * freq, i * freq)`로 했었음. 그러면 grid cell 단위로 wave가 들어가서 0.625m마다 풀 사이클 → mean slope **68°** (비현실적, passable 1%).

Fix: **x_world = j * resolution을 먼저 곱하기** → frequency가 world meter 단위로 동작. wavelength = 1/0.08 = 12.5m. mean slope **12.79°** (현실적).

### 사용 예 (이지민 T5 TRN)
```python
import numpy as np
global_heightmap = np.load("terrain_00001/heightmap.npy")

# 매 5초마다 (150 step at 30Hz)
local = env.scene["height_scanner"].data.ray_hits_w   # (100, 100)
estimated_pos, confidence = trn.localize(local, prior_pos, global_heightmap)
```

### ⚠️ 김현중 ↔ 이지민 핵심 합의
- `shape (1000, 1000) float32` 고정
- `origin (-25, -25)`, `resolution 0.05`
- `i = y축, j = x축`, row-major
- TRN이 이 합의 기반으로 cross-correlation 좌표 매칭 → 한 번 어긋나면 전체 망가짐

검증 방법: `heightmap_at(mineral.x, mineral.y) + 0.10 == mineral.z`가 항상 성립하는지.

---

## 4개 파일 정합 도식

```
                    ┌──────────────────────────────────────┐
                    │       heightmap.npy (1000×1000 float)│  ← 원본 데이터
                    │       origin (-25,-25), res 0.05     │
                    └─┬────────────────┬────────────────┬──┘
                      │                │                │
            stride=5 다운샘플      slope > 25° OR   이지민 TRN 사용
                      ▼              + rocks dilate   (cross-correlation)
        ┌─────────────────────┐         ▼
        │ terrain_only.usd    │  ┌──────────────┐
        │ 200×200 mesh        │  │ obstacle_grid│ ← 이찬휘 A* 입력
        │ Mars red color      │  │ .npy (int8)  │
        └─────────────────────┘  └──────────────┘

    독립 산출물:
        ┌────────────────────────┐
        │ rocks_merged.usd       │  ← Isaac Sim 시각,
        │ 80 sphere prims        │     위치는 obstacle_grid에도 반영
        └────────────────────────┘
```

핵심: **3개 파일(USD mesh + npy 2개)이 동일 heightmap에서 파생** → 좌표 정합 보장. rocks는 독립이지만 위치가 obstacle_grid에 반영돼 일관성 유지.

---

## terrain 새로 생성 시 흐름

```bash
python3 isaac_sim/scripts/mars_terrain_generator_v2.py \
    --seed 23456 --terrain-id terrain_00002
```

생성되는 것:
1. `generated_terrains/terrain_00002/{terrain_only.usd, rocks_merged.usd, obstacle_grid.npy, heightmap.npy, meta.json}` 5개 ✅
2. `generated_terrains/index.json` (기존 entry + 새 terrain 추가, I1 포맷) ✅
3. `worlds/terrain_00002.usd` ← 이 terrain 전용 master scene (terrain별 보존) ✅
4. `worlds/mars_exploration_world.usd` ← 최신 terrain alias로 갱신 ✅

> master scene 정책 (v2): terrain마다 `worlds/<terrain_id>.usd`를 보존하고, `worlds/mars_exploration_world.usd`는 항상 최신 terrain의 alias. 특정 terrain은 `worlds/<terrain_id>.usd`로, 최신은 `mars_exploration_world.usd`로 연다.

생성 안 되는 것 (이미 존재 시 보호):
- `markers/mineral_*.usd`, `markers/basecamp_dome.usd` — 김현중이 멋진 모형으로 교체해뒀을 수도 있어서 보호. 다시 만들고 싶으면 `rm` 후 generator 재실행.

---

## 자주 묻는 질문

**Q1. mars_exploration_world.usd 만 git에 올리면 다른 사람도 볼 수 있나?**
❌ 안 됨. reference만 들어있어서 `terrain_only.usd`/`rocks_merged.usd`/`markers/*.usd` 다 같이 있어야 함. git LFS 또는 별도 asset 저장소 권장.

**Q2. markers/의 mineral_blue.usd를 멋진 모형으로 교체하면?**
✅ 1000개 terrain × 평균 12광물 = 12000개가 동시에 자동 갱신. master scene/generator 코드 수정 불필요. Isaac Sim에서 Reload만 하면 됨.

**Q3. terrain_only.usd 만 별도로 Isaac Sim에 열어볼 수 있나?**
✅ 가능. mesh 단독 prim이라 Isaac Sim에서 File>Open하면 갈색 mesh 1장만 보임. 디버깅용으로 유용.

**Q4. obstacle_grid와 heightmap의 인덱스가 정확히 일치하는가?**
✅ 동일. 둘 다 (1000, 1000) shape, 같은 origin/resolution. `obstacle_grid[i,j]==1`인 cell의 높이는 `heightmap[i,j]`.

**Q5. rocks_merged.usd의 80개 sphere 위치는 어디서 결정?**
generator의 rejection sampling 결과 → meta.json의 `generation_params.rocks` 조건에 따라. 같은 seed면 동일 위치 (재현 보장).

---

## 김현중 합류 후 정리 후보

- [ ] markers/rock_default.usd 도입 + rocks_merged.usd가 reference 방식으로 (현재 직접 Sphere prim)
- [ ] master scene을 동시에 여러 terrain 보여주는 batch 시각화 옵션
- [ ] terrain_only.usd에 `UsdPreviewSurface` material 추가 (현재 displayColor만)
- [ ] obstacle_grid 생성에 dilation kernel 파라미터화 (현재 rock size 기반)
- [ ] heightmap에 noise multi-layer (Tier 2: dunes + craters 분리)
