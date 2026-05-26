# 📋 meta.json 필드 가이드

> 우리 첫 샘플 [`terrain_00001/meta.json`](../../isaac_sim/assets/generated_terrains/terrain_00001/meta.json)을 통짜로 보여주면서 라인별 주석.
> I1 전체(5개 파일 + master scene)는 [I1_TERRAIN_ASSETS.md](I1_TERRAIN_ASSETS.md) 참조.
> 풀 schema: [terrain_meta_schema.json](terrain_meta_schema.json) · 인터페이스 계약: [INTERFACE_CONTRACTS.md I1](INTERFACE_CONTRACTS.md#i1-terrain-asset-t1-김현중--모두)

소비자 4명 모두 시뮬 시작 시 1회 로드 → 광물 위치, 베이스캠프, 스폰, 난이도 등을 읽음.

---

```jsonc
{
  // ═══ A. 정체성 / 재현성 ═══════════════════════════════════════════
  "terrain_id": "terrain_00001",       // [모두]            디렉터리 이름과 일치 필수. 학습/평가 결과 추적 key
  "version": "1.0",                    // [모두]            schema 버전. 필드 추가/삭제 시 마이그레이션 트리거
  "seed": 12345,                       // [김현중 T1]       PCG 시드. 같은 seed + 같은 params → bit-identical
  "generated_at": "2026-05-20T00:29:45", // ISO 8601, 디버깅용 timestamp

  // ═══ B. 공간 정의 (좌표계 핵심) ════════════════════════════════════
  "size_m": [50.0, 50.0],              // [모두]            월드 크기 [X, Y] (m). 권장 50×50
  "resolution_m": 0.05,                // [이찬휘 T3, 이지민 T5]  npy grid cell (m). 1000×1000 = 50/0.05
  "origin": {"x": -25.0, "y": -25.0},  // [이찬휘 T3, 이지민 T5]  npy의 grid[0,0]이 가리키는 월드 좌표 (좌하단)
                                       // ⚠️ 김현중 ↔ 이지민 사이 가장 중요한 합의 (TRN이 좌표 어긋나면 전부 망가짐)

  // ═══ C. 생성 파라미터 (재현/디버그용. 런타임 안 읽힘) ════════════════
  "generation_params": {
    "terrain": {
      "type": "perlin",                // perlin | simplex | ridged  (현재 perlin만 구현)
      "octaves": 4,                    // noise layer 수. 클수록 디테일↑. 난이도: 2/4/6 (easy/med/hard)
      "frequency": 0.08,               // 1/freq = wavelength(m). 0.08 → 12.5m feature scale
      "amplitude_m": 3.0,              // 높이 변동 폭(m). 실제 range는 Perlin 특성상 더 작게 압축됨
      "lacunarity": 2.0,               // octave 간 frequency 배율
      "persistence": 0.5               // octave 간 amplitude 배율
    },
    "rocks": {
      "count": 80,                     // 목표 개수. rejection sampling이 실패하면 실제 더 적을 수 있음
      "size_range_m": [0.3, 1.5],      // 암석 반지름 범위(m)
      "min_spacing_m": 1.0,            // 암석 간 최소 거리
      "slope_threshold_deg": 25.0,     // 이보다 가파른 곳엔 안 놓음 (굴러떨어짐 방지)
      "asset_pool": ["rock_default"]   // 사용할 USD pool. 김현중 합류 후 다양화 예정
    },
    "minerals": {
      "count": 12,                     // 광물 개수 — 미션 길이 결정
      "min_spacing_m": 3.0,
      "exclude_basecamp_radius_m": 5.0,// 베이스 근처 제외 → 탐사 강제
      "value_distribution": {          // ⭐ [최진우 T2 vision] 같은 거리에 여러 광물이면 yellow 우선 학습 근거
        "blue":   {"prob": 0.5, "score": 10},  // 흔함
        "red":    {"prob": 0.3, "score": 25},  // 보통
        "yellow": {"prob": 0.2, "score": 50}   // 희귀
      }
    },
    "physics_zones": {                 // 영역 추출 알고리즘 메타. 현재 hardcoded라 큰 의미 없음
      "type": "noise_based",           // 합류 후 진짜 noise 기반으로 확장 예정
      "noise_frequency": 0.04,
      "sand_threshold": 0.3
    }
  },

  // ═══ D. 런타임 데이터 (Consumer가 실제 읽는 것) ══════════════════════

  // [이찬휘 T3 FSM, 이지민 T5 학습 루프]  매 episode reset 시 무작위 1개 선택 → 로버 위치/방향 설정
  // 검증 조건: basecamp 외부, slope < 15°, obstacle 아님, 경계 ≥ 2m
  // z = heightmap_at(x,y) + 0.18  (휠 ground clearance)
  "spawn_locations": [
    {"x": 8.61,  "y": 0.7,    "z": 0.66, "yaw": 5.659, "group": "default"},
    {"x": -8.44, "y": -11.81, "z": 0.84, "yaw": 1.555, "group": "default"},
    {"x": 22.11, "y": -4.01,  "z": -0.01,"yaw": 4.384, "group": "default"}
    // ... 50개. group은 향후 난이도별 분리 여지 ("easy_start", "hard_start")
  ],

  "basecamp": {
    "center": {"x": 0.0, "y": 0.0},    // [이찬휘 T3 FSM]   "is rover home?" 판정 기준점
    "radius": 3.0,                     // [이찬휘 T3]       베이스 영역 반지름 (m)
    "marker_usd": "basecamp_dome.usd", // ⚠️ 정보용만. composer는 markers/ 고정 파일명을 직접 사용
                                       //    → meta의 이 값 바꿔도 시각 변화 없음 (markers/ 파일 자체를 교체해야 함)
    "visual_footprint_m": [3.0, 3.0],  // 시각 영역 (UI 미니맵)
    "marker_height_m": 5.5,            // 안테나 포함 전체 높이
    // ─── Tier 2 필드 (현재 null/[], schema 호환 위해 반드시 유지) ───
    "shape": null,                     // 후행: 다각형 또는 USD collision.  ⚠️ 삭제 금지
    "entry_points": [],                // 후행: 입구 좌표 (도킹용).         ⚠️ 삭제 금지
    "collision_usd_path": null         // 후행: 벽 충돌 USD 경로.          ⚠️ 삭제 금지
  },

  "minerals": [
    {
      "id": 1,                                  // [김현중 T1 발급, 1부터 순차]   [최진우 T2] detection 매칭 key
      "type": "blue_mineral",                   // [최진우 T2]   markers/tier2_mineral/{type}.usd 자동 매핑
      "position": {"x": 17.3, "y": -1.99, "z": -0.33},  // [이찬휘 T3] I3 target_position. z = heightmap + 0.10
      "value": 10                               // [이찬휘 T3] value/distance 우선순위.  [성선규 T4 UI] 미니맵 라벨
    },
    {"id": 2, "type": "yellow_mineral", "position": {"x": 11.63, "y": -19.4, "z": -0.7}, "value": 50},
    {"id": 3, "type": "green_gas",      "position": {"x": -21.41, "y":  4.46, "z":  0.07}, "value": 25}
    // ... 총 12개. type ∈ {"blue_mineral", "yellow_mineral", "green_gas"} (YOLO model.names 와 동일)
  ],

  // [이지민 T5 Mars Tier 2]  영역별 PhysX RigidBodyMaterialCfg 적용 대상
  "physics_zones": [
    {
      "type": "sand",                                // sand | rocky | ice | regolith
      "polygon": [[-10,-10], [10,-10], [10,0], [-10,0]],  // 영역 폴리곤 (월드 좌표 m)
      "static_friction": 0.30,                       // 모래 → grip 낮음
      "dynamic_friction": 0.25
    },
    {
      "type": "rocky",
      "polygon": [[-25,-25], [-10,-25], [-10,-10], [-25,-10]],
      "static_friction": 0.55,
      "dynamic_friction": 0.50
    }
    // 현재 hardcoded 2개. 합류 후 noise 기반 자동 추출로 확장
  ],

  "minimap": {                         // [이찬휘 T3 Coverage planner]  셀 방문 처리 (Greedy frontier)
                                       // [성선규 T4 UI]                미니맵 시각 렌더링
    "grid_size": [25, 25],             // 25×25 셀
    "cell_size_m": 2.0,                // 셀당 2m → 50m 영역 커버
    "origin": {"x": -25.0, "y": -25.0} // 좌하단 월드 좌표. (x,y) → ((x-origin.x)/cell, (y-origin.y)/cell)
  },

  // ═══ E. 사후 분석 메트릭 (사용자 출력) ═══════════════════════════════
  "difficulty": {
    "score": 0.528,                    // [모두]            0~1 정규화 종합 점수. curriculum 학습 정렬
                                       // 공식: 0.3*(rock_density/0.03) + 0.4*(mean_slope/30) + 0.3*(1-passable)
    "rock_density": 0.032,             // 암석 개수 / 면적 (count/m²)
    "max_slope_deg": 45.12,            // gradient의 max
    "mean_slope_deg": 12.79,           // gradient의 mean.  목표 medium: 10~15°
    "passable_ratio": 0.876,           // (slope<25° AND not rock) cells / total cells
    "longest_corridor_m": 46.8         // 안전 영역 connected component의 최장 길이
  }
}
```

---

## 자주 묻는 질문

**Q1. `minerals[i].value` vs `value_distribution[type].score` — 왜 둘 다?**
전자 = 부여된 *실제* 점수. 후자 = base score (생성 파라미터). 런타임 consumer는 전자만 봄.

**Q2. `basecamp.marker_usd` 값을 바꾸면 모양이 바뀌나?**
❌ **안 바뀜**. composer가 `markers/basecamp_dome.usd`를 고정으로 reference. 모양 교체하려면 `isaac_sim/assets/markers/basecamp_dome.usd` 파일 자체를 새 USD로 덮어쓰기.

**Q3. heightmap.npy 좌표가 헷갈림.**
`grid[i, j]`의 world 좌표 = `(origin.x + j*resolution, origin.y + i*resolution)`. **i = y축, j = x축** (row-major). 김현중 ↔ 이지민 간 가장 중요한 합의.

**Q4. `spawn_locations`가 50개 미만이면?**
rejection sampling 실패. `passable_ratio` 낮은 Hard 난이도거나 `basecamp.radius` 너무 클 때 발생. 첫 샘플은 50/50.

**Q5. 우리 첫 샘플은 어떤 난이도?**
score 0.528 / mean_slope 12.79° / passable 87.6% → **medium** (target 0.3~0.6 범위 안).

---

## 김현중 합류 후 정리 후보

- [ ] `basecamp.marker_usd`를 composer가 실제로 읽도록 (현재 정보용)
- [ ] `physics_zones`를 진짜 noise 기반 영역 추출로 (현재 hardcoded 2개)
- [ ] minerals에 `marker_usd_override` 필드 추가 (type 매핑 위에 덮어쓰기)
- [ ] `generation_params.terrain.type`에 simplex / ridged 구현
- [ ] `spawn_locations.group`을 난이도별 분리 ("easy_start", "hard_start")
- [ ] `rocks.asset_pool`에 진짜 다양한 USD 사용 (현재 sphere 1종)
