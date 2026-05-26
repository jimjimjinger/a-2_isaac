# isaac_supervisor

> **트랙 owner**: 성선규 (T4 — Integration + PM)
> **책임**: 전체 미션 흐름 감독 + 배터리 모니터링

---

## 1. 모듈 역할

상위 미션 orchestration 노드. drive_manager의 FSM은 phase 전환 기반이고, supervisor는 그 위에서 **전체 미션 lifecycle** 관리:

- 미션 시작/종료 시그널
- 배터리 모니터링 (낮으면 강제 RETURN_BASE)
- 트랙별 노드 health check (T5 이지민 pose publish 멈춤 등)
- UI (T4 dashboard)로 미션 상태 publish

---

## 2. 폴더 구조 (현재 상태)

```text
isaac_supervisor/
├─ isaac_supervisor/
│  ├─ __init__.py                       ⏳ stub
│  ├─ mission_manager_node.py           ⏳ stub — top-level 감독
│  └─ battery_monitor_node.py           ⏳ stub — 배터리 모니터링
├─ package.xml
└─ setup.py
```

**모든 .py가 0 byte stub.** 성선규 (T4) 본인 작업 영역.

---

## 3. 작업 시작 가이드

| 자료 | 위치 |
|------|------|
| 트랙 onboarding | [docs/tracks/T4_BRIEF.md](../docs/tracks/T4_BRIEF.md) |
| Claude Code context | [docs/tracks/T4_CLAUDE.md](../docs/tracks/T4_CLAUDE.md) |
| 의존 메시지 | `isaac_interfaces/msg/MissionState.msg`, `BatteryState.msg` |

---

## 4. drive_manager (이찬휘 T3)와의 관계

```
mission_manager_node (T4 성선규)
   │ subscribe: /mission/status (T3 phase 정보)
   │ publish:   /mission/control (시작/종료/abort 명령)
   │
   ↓
drive_manager_node (T3 이찬휘)
   │ 자체 FSM: EXPLORE/APPROACH/PICK/RETURN
   │ → /cmd_vel 등 publish
```

**역할 구분**: supervisor는 *언제 시작/종료할지*, drive_manager는 *어떻게 주행할지*.

---

## 5. 한 줄 요약

> **성선규 본인 트랙의 미션 orchestration + battery 모니터.** drive_manager 위에서 전체 lifecycle 관리.
