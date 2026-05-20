# T1 (김현중) Environment — Claude Code Context

> 이 파일은 Claude Code가 자동 로드하는 트랙 컨텍스트입니다. 작업 시작 시 첨부.

## 너의 정체성
**T1 (김현중) 트랙 owner — 절차생성 Mars terrain + Mars Physics Tier 1 + basecamp visual**

GPU: 5060 (8GB) — Isaac Sim 작업 가능하지만 무거우니 batch는 headless로.

## 작업 시작 전 필독
1. [T1_BRIEF.md](T1_BRIEF.md) — 전체 onboarding (필드별 의미, 알고리즘, 일정)
2. [interfaces/INTERFACE_CONTRACTS.md](../interfaces/INTERFACE_CONTRACTS.md) — I1 섹션
3. [interfaces/terrain_meta_schema.json](../interfaces/terrain_meta_schema.json) — 출력 형식
4. [interfaces/example_terrain_meta.json](../interfaces/example_terrain_meta.json) — 참고 예시

## 내가 만드는 것

각 terrain 디렉터리:
```
generated_terrains/terrain_NNNNN/
├── terrain_only.usd        # Isaac Sim 메쉬
├── rocks_merged.usd        # 암석 메쉬
├── obstacle_grid.npy       # T3 (이찬휘) A*가 사용
├── heightmap.npy           # T5 (이지민) TRN이 사용 (좌표계 정확히!)
└── meta.json               # T2 (최진우)/T3 (이찬휘)/T4 (성선규)/T5 (이지민) 모두 사용
```

목표: 30~50개 batch, easy/medium/hard 분포.

## 핵심 작업 영역

```
rover/sim/rover_envs/envs/navigation/rover_env_cfg.py   ← gravity=3.72 추가
rover/sim/rover_envs/assets/terrains/mars/               ← 새 terrain 추가
tracks/    ← 작업 위치, 본인 코드는 여기에 내 작업 디렉터리
generated_terrains/                                       ← 출력
```

## 절대 손대지 마라
- `rover/sim/rover_envs/learning/` (RL 학습 코드 — T3 (이찬휘) 영역)
- `rover/sim/scripts/03_eval_ros2.py` (실행 진입점)
- 다른 트랙의 코드 (tracks/T2 (최진우)~T5 (이지민)/)
- 인터페이스 schema (PM 승인 필수)

## 도구
```bash
pip install numpy scipy noise pymeshlab opencv-python termcolor
```

USD Python API:
```python
from pxr import Usd, UsdGeom, Gf, Sdf
```

## 일정 핵심 마일스톤
- **Day 1 EOD**: terrain_00001 1세트 + meta.json 1장
- **Day 2 EOD** ⚠️: 5개 시드로 시각적으로 다른 5개 terrain (게이트)
- **Day 4 EOD**: 30개 batch + train/holdout split
- **Day 5+**: 검증, Mars Tier 1 (gravity 3.72) 적용

## 일일 워크플로
- 09:30 standup → PM이 DAILY_STATUS 갱신
- 18:00 DIST → PM이 통합 테스트, 필요 시 본인 호출
- 인터페이스 변경 필요 시 → PM에게 즉시 alert

## 트러블슈팅 우선 순위
1. 막혔으면 5분 안에 PM(T4 (성선규)) 호출
2. heightmap 좌표계 혼동? → T5 (이지민)와 즉시 sync
3. USD export 실패 → fallback: 직접 USD Python API
4. PyMeshLab 깨짐 → pip install 재설치, 또는 Open3D로 대체
