# isaac_manipulation

> **트랙 owner**: 최진우 (T2 — Perception + M0609 매니퓰레이션 측)
> **책임**: M0609 로봇팔 제어 + scripted pick/place/unload/deploy primitives

---

## 1. 모듈 역할

Doosan M0609 6축 매니퓰레이터를 ROS2 action server로 노출. 4개 primitive (pick_mineral / place_to_cargo / unload_to_base / deploy_solar_panel) 실행.

**Tier 1.5 전략**: 진짜 IK + force feedback X. **scripted joint trajectory + 광물 텔레포트**로 시각적 pick 동작 시연 충분. 자세한 결정: [docs/pm_tools/DECISIONS.md #005](../docs/pm_tools/DECISIONS.md).

---

## 2. 폴더 구조 (현재 상태)

```text
isaac_manipulation/
├─ isaac_manipulation/
│  ├─ __init__.py                            ⏳ stub
│  ├─ arm_executor_node.py                   ⏳ stub — ExecuteArmTask action server
│  └─ primitives/
│     ├─ pick_mineral.py                     ⏳ stub
│     ├─ place_to_cargo.py                   ⏳ stub
│     ├─ unload_to_base.py                   ⏳ stub
│     └─ deploy_solar_panel.py               ⏳ stub
├─ scripts/
│  └─ build_rover_m0609_scene.py             ✅ Day 1 spike 산출물 (§6)
├─ package.xml
└─ setup.py
```

**ROS2 노드 .py 는 0 byte stub.** scripts/ 는 Isaac Sim 직접 실행용 (ROS2 패키지 외부).

---

## 3. 작업 시작 가이드

| 자료 | 위치 |
|------|------|
| 트랙 onboarding | [docs/tracks/T2_BRIEF.md](../docs/tracks/T2_BRIEF.md) §4 M0609 |
| Claude Code context | [docs/tracks/T2_CLAUDE.md](../docs/tracks/T2_CLAUDE.md) |
| I3/I4 인터페이스 | [docs/interfaces/INTERFACE_CONTRACTS.md I3](../docs/interfaces/INTERFACE_CONTRACTS.md#i3-missionpick_request-t3-이찬휘--t2-최진우) |
| Action 정의 | `isaac_interfaces/action/ExecuteArmTask.action` |

---

## 4. drive_manager (이찬휘 T3)와의 관계

```
drive_manager_node (T3 이찬휘)
   │ PICK phase 진입 시
   │ ↓ action call: ExecuteArmTask
   │
arm_executor_node (T2 최진우)
   │ 1. M0609을 target 좌표 위로 이동 (scripted trajectory)
   │ 2. Grasp 시퀀스 (5단계)
   │ 3. 광물 USD를 cargo bin으로 텔레포트
   │ ↓ action result: success | failed_grasp | timeout | no_object
```

---

## 5. M0609 spike (Day 1 ✅ 2026-05-20 통과)

> M0609 USD asset 로드 + RG2 결합 + rover 결합 + 시뮬레이션 안정성 모두 통과.
> 산출물: [§6 build_rover_m0609_scene.py](#6-day-1-spike-산출물--rover--m0609--rg2-통합-씬-스크립트).

상세 onboarding: [T2_BRIEF.md §4](../docs/tracks/T2_BRIEF.md)

---

## 6. Day 1 spike 산출물 — rover + M0609 + RG2 통합 씬 스크립트

`scripts/build_rover_m0609_scene.py` (commit `94627ab`, 2026-05-20)

### 6-1. 무엇이 들어가나

| 단계 | 내용 |
|------|------|
| ① mars 환경 | T1(김현중)의 `mars_exploration_world.usd` 를 reference 로 로드 |
| ② PhysX | Mars gravity 3.72 m/s² PhysicsScene 추가 + terrain mesh collision 보강 |
| ③ Rover | `isaac_sim/assets/rover/Mars_Rover.usd` (T1 자산) reference, spawn (5, 0, 1.0). 휠/조향 drive freeze |
| ④ M0609 | `doosan-robot2/urdf/m0609_isaac_sim.urdf` URDF import (floating-base), `TransformPrimSRTCommand` 로 (5.15274, 0, 1.21232) 위치 적용 |
| ⑤ RG2 | `onrobot_rg2/urdf/onrobot_rg2.urdf` URDF import, RobotAssembler 로 M0609 `link_6` ↔ RG2 `angle_bracket` 결합 |
| ⑥ rover↔M0609 | rover/Body 자식으로 mount-point Xform 생성 (offset (0.15274, 0, 0.21232)) → RobotAssembler 가 m0609.base_link 를 그 위치로 정렬 |

핵심 hack:
- URDF import 후 prim 이동은 `TransformPrimSRTCommand` 사용 (단순 USD `AddTranslateOp` 만으로는 PhysX 와 sync 안 됨)
- RobotAssembler 의 mount alignment 가 두 body 를 같은 위치로 끌어오는 default 동작을 mount-point Xform 으로 우회
- 두 articulation (rover + m0609) 사이 fixed joint 는 RobotAssembler 가 안정적으로 생성 (직접 PhysicsFixedJoint 는 중력 충격에 분리됨)

### 6-2. 실행 방법

```bash
# 기본 실행 (Isaac Sim 창 뜨고 Spacebar 로 시뮬 시작)
isaac-python ~/dev_ws/rover_ws/src/a2_isaac/isaac_manipulation/scripts/build_rover_m0609_scene.py

# 또는 자동 Play 까지
isaac-python ~/dev_ws/rover_ws/src/a2_isaac/isaac_manipulation/scripts/build_rover_m0609_scene.py --auto-play
```

> **주의**: 어디서 실행하든 OK. 스크립트 내부에서 `os.chdir(tempfile.mkdtemp(...))` 로 작업 디렉토리를 임시 쓰기 가능 경로로 옮김 (URDF importer 가 cwd 에 임시 mesh USD 를 만들기 때문 — `/opt/ove/base_stack` 같은 read-only 위치에서 실행해도 무관).

예상 흐름:
1. Isaac Sim 윈도우 부팅 (5~10초)
2. mars 환경 로드 (~5초)
3. M0609 + RG2 URDF import + assembly (~10초)
4. "씬 준비 완료" 출력 → Spacebar 로 시뮬 시작
5. rover 자유낙하 → 휠 안착 → M0609 + RG2 동반

### 6-3. 검증된 동작

- ✅ rover spawn 후 자유낙하 → 휠 지면 접지
- ✅ M0609 base_link 가 rover Body 위 정확한 offset(0.15, 0, 0.21)에 부착
- ✅ RG2 angle_bracket 이 M0609 link_6 에 결합
- ✅ 시뮬 시작 후 분리 없이 일체로 움직임
- ⚠️ M0609 본체 색상은 흰색 (URDF in-memory stage 한계로 material override 보류, RG2 는 검정 적용됨)

### 6-4. Fine-tune 포인트

스크립트 상단 상수만 바꿔서 빠르게 iterate:

```python
SPAWN_X = 5.0                  # 동쪽 5m (basecamp 기준)
SPAWN_Y = 0.0
SPAWN_Z = 1.0                  # 자유낙하 거리 확보

M0609_MOUNT_OFFSET_X = 0.15274  # rover.Body 기준 m0609.base_link offset
M0609_MOUNT_OFFSET_Y = 0.0
M0609_MOUNT_OFFSET_Z = 0.21232  # GUI 시각 검증값
```

`M0609_MOUNT_OFFSET_Z` 조정 가이드:
- 작은 값 (0.0~0.2): M0609 가 rover Body 안으로 박혀 보임
- 0.21232: 정착 (현재 검증값)
- 큰 값 (0.5+): M0609 가 공중에 떠 보임

### 6-5. 다음 단계 (T2 후속 작업)

- Day 2: Vision PoC — HSV 광물 detection (`isaac_perception/`)
- Day 3: M0609 scripted trajectory primitives (이 README §2 의 4개 stub)
- Day 4: T5 estimated_pose + T3 pick_request 받아서 첫 통합

---

## 7. 한 줄 요약

> **최진우의 M0609 매니퓰레이션.** scripted trajectory + 광물 텔레포트 (Tier 1.5). 8일에 진짜 IK 안 함.
