# isaac_sim

> **트랙 owner**: 김현중 (T1 — Environment)
> **책임**: 화성 시뮬레이션 환경 — 절차생성 지형 + 화성 물성 + 베이스캠프 + ROS2 bridge

---

## 1. 모듈 역할

`isaac_sim`은 Isaac Sim 위에 화성 탐사 환경을 구성하는 모듈입니다.

**들어가는 것**: 절차생성 지형(USD/npy), 물성치 설정, 광물/베이스캠프 USD, master scene, Isaac Sim ↔ ROS2 bridge.

**안 들어가는 것**: AI 인식 모델 (→ isaac_perception), RL 정책 (→ isaac_rl), 미션 로직 (→ isaac_supervisor), 매니퓰레이션 (→ isaac_manipulation).

---

## 2. 폴더 구조 (현재 상태)

```text
isaac_sim/
├─ isaac_sim/
│  └─ sim_bridge_node.py            ⏳ stub — Isaac Sim ↔ ROS2 bridge (성선규 T4가 채울 예정)
├─ scripts/
│  ├─ procedural_terrain_generator.py   ✅ T1 1샘플 생성기 (성선규가 임시 작성 → 김현중 확장 예정)
│  ├─ basecamp_visual_builder.py        ⏳ 빈 파일 — 김현중이 채울 예정
│  └─ mars_physics_config.py            ⏳ 빈 파일 — gravity/friction/sun direction 들어갈 자리
├─ worlds/
│  └─ mars_exploration_world.usd    ✅ master scene (terrain + rocks + basecamp + 12 minerals + 조명)
├─ assets/
│  ├─ generated_terrains/
│  │  ├─ index.json                 ✅ train/holdout split
│  │  └─ terrain_00001/             ✅ 첫 1개 샘플 (I1 풀스펙)
│  │     ├─ terrain_only.usd        # 200×200 mesh (Mars red)
│  │     ├─ rocks_merged.usd        # 80 sphere
│  │     ├─ obstacle_grid.npy       # (1000,1000) int8
│  │     ├─ heightmap.npy           # (1000,1000) float32
│  │     └─ meta.json               # 풀 schema 통과
│  └─ markers/                      ✅ 모든 terrain 공유 USD
│     ├─ mineral_{blue,red,yellow}.usd  # 단색 sphere (HSV detection target)
│     └─ basecamp_dome.usd          # 패드+돔+안테나
├─ package.xml
└─ setup.py
```

---

## 3. 첫 샘플 생성 흐름 (현재 동작 중)

```bash
# 1샘플 생성 (procedural_terrain_generator.py)
python3 isaac_sim/scripts/procedural_terrain_generator.py \
    --seed 12345 --terrain-id terrain_00001

# 생성물:
#   isaac_sim/assets/generated_terrains/terrain_00001/  (5 files)
#   isaac_sim/assets/generated_terrains/index.json
#   isaac_sim/assets/markers/{mineral_*,basecamp_dome}.usd  (없으면 생성)
#   isaac_sim/worlds/mars_exploration_world.usd          (master scene 갱신)

# Isaac Sim에서 시각 확인
isaac isaac_sim/worlds/mars_exploration_world.usd
```

자세한 I1 파일 구조와 master scene composition은 [docs/interfaces/I1_TERRAIN_ASSETS.md](../docs/interfaces/I1_TERRAIN_ASSETS.md) 참조.

---

## 4. 김현중이 합류 후 확장할 영역

| 작업 | 위치 | 우선순위 |
|------|------|:--------:|
| `mars_physics_config.py` 채우기 (gravity 3.72, ground/wheel friction, sun direction) | scripts/ | 🔴 P0 |
| `procedural_terrain_generator.py`에 `--batch` 모드 + easy/medium/hard preset | scripts/ | 🔴 P0 |
| markers/의 mineral / basecamp / rock USD를 진짜 모형으로 교체 (같은 파일명) | assets/markers/ | 🟡 P1 |
| `basecamp_visual_builder.py` 채우기 (현재 generator에 임시 통합돼 있음) | scripts/ | 🟢 P2 |
| 30개 batch + train/holdout split 자동화 | scripts/ | 🟡 P1 |
| `rocks_merged.usd`를 sphere prim → markers/rock_default.usd reference 방식으로 | scripts/ | 🟢 P2 |

→ 진행 가이드: [docs/tracks/T1_BRIEF.md](../docs/tracks/T1_BRIEF.md), [docs/tracks/T1_CLAUDE.md](../docs/tracks/T1_CLAUDE.md)

---

## 5. ROS2 Bridge (sim_bridge_node.py) — 미구현

성선규 (T4) 트랙. Isaac Sim 센서/액추에이터를 ROS2 topic으로 노출:

```
Isaac Sim Camera/Depth/LiDAR  ──→ ROS2 topic ──→ 최진우 T2 perception_node
Isaac Sim Robot joints/IMU    ──→ ROS2 topic ──→ 이지민 T5 localization_node
ROS2 /cmd_vel  ──→ Isaac Sim Rover wheels  ←── 이찬휘 T3 mobile_base_executor_node
ROS2 arm cmd   ──→ Isaac Sim M0609         ←── 최진우 T2 arm_executor_node
```

→ 진행 가이드: [docs/tracks/T4_BRIEF.md §4](../docs/tracks/T4_BRIEF.md)

---

## 6. 의존 / 외부 패키지

```bash
# 절차생성 generator 의존성 (한 번만 설치)
pip install --user noise usd-core jsonschema scipy
```

Isaac Sim 자체는 별도 설치 (워크스페이스 외부). 본 패키지는 Isaac Sim에 USD를 *로드*시키는 generator + asset 보관 역할.

---

## 7. 한 줄 요약

> **김현중이 만드는 화성 환경의 raw geometry/물성 보관소.** ROS2 노드는 여기서 USD를 로드하고, 절차생성 결과(npy/json)를 읽어가서 자기 트랙 로직에 사용.
