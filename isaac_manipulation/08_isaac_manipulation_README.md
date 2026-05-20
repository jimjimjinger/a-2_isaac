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
├─ package.xml
└─ setup.py
```

**모든 .py가 0 byte stub.** 최진우 작업 영역.

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

## 5. M0609 spike (Day 1 ⚠️)

가장 큰 unknown. Day 1 ⚠️ 게이트:
- M0609 USD asset이 Isaac Sim에서 로드되는가?
- 실패 시 → 단순 6-link 매니퓰레이터 직접 모델링 (1시간)

상세: [T2_BRIEF.md §4](../docs/tracks/T2_BRIEF.md)

---

## 6. 한 줄 요약

> **최진우의 M0609 매니퓰레이션.** scripted trajectory + 광물 텔레포트 (Tier 1.5). 8일에 진짜 IK 안 함.
