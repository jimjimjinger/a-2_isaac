# isaac_interfaces

> **트랙 owner**: 성선규 (T4 — Integration + PM, 인터페이스 PM)
> **책임**: ROS2 노드 간 통신 규격 (msg/srv/action). 합의 후 freeze.

---

## 1. 모듈 역할

ROS2 노드 사이의 통신 데이터 형식을 정의하는 **CMake 인터페이스 패키지**.

- 5개 트랙이 사용할 메시지 타입을 한 곳에 모음
- Day 1 회의에서 합의 → freeze. 변경은 PM 승인 + 전체 회의 필요.

문서 영역과 빌드 영역이 분리되어 있음:
- **계약/스펙 문서**: `docs/interfaces/` (INTERFACE_CONTRACTS, I1_TERRAIN_ASSETS, META_JSON_FIELDS, schema 등) — 사람이 읽는 정의
- **실제 ROS2 빌드 정의**: `isaac_interfaces/{msg,srv,action}/` — colcon이 빌드하는 .msg/.srv/.action 파일

---

## 2. 폴더 구조 (현재 상태)

```text
isaac_interfaces/
├─ msg/                            ✅ 현재 4개 정의
│  ├─ PerceptionResult.msg          # I2 — T2 최진우 → T3 이찬휘/T4 성선규
│  ├─ SelectedDriveAction.msg       # RL → 주행 (내부)
│  ├─ MissionState.msg              # T3 → T4 UI
│  └─ BatteryState.msg              # T4 battery monitor
│
├─ srv/                            ✅ 현재 3개 정의
│  ├─ CheckSystemReady.srv
│  ├─ ResetSimulation.srv
│  └─ SaveExplorationMap.srv
│
├─ action/                         ✅ 현재 3개 정의
│  ├─ ExecuteArmTask.action         # I3/I4 — T3 이찬휘 ↔ T2 최진우 매니퓰레이션
│  ├─ ExecuteDriveTask.action       # T3 내부 또는 T4 → T3
│  └─ NavigateTask.action           # T3 내부
│
├─ CMakeLists.txt
└─ package.xml
```

---

## 3. 5개 인터페이스 매핑 (I1 ~ I5)

[docs/interfaces/INTERFACE_CONTRACTS.md](../docs/interfaces/INTERFACE_CONTRACTS.md) 의 I1~I5와 본 패키지의 메시지 매핑:

| ID | 인터페이스 | 본 패키지 매핑 | 비고 |
|:--:|---|---|---|
| **I1** | Terrain Asset | (없음, 파일 기반) | `isaac_sim/assets/generated_terrains/`. 풀 가이드 [I1_TERRAIN_ASSETS.md](../docs/interfaces/I1_TERRAIN_ASSETS.md) |
| **I2** | `/perception/detections` | `msg/PerceptionResult.msg` | `value_score` 필드 추가 협상 필요 |
| **I3** | `/mission/pick_request` | `action/ExecuteArmTask.action` (goal) | 팀 Action 사용 (더 좋음) |
| **I4** | `/mission/pick_response` | `action/ExecuteArmTask.action` (result/feedback) | 위와 통합 |
| **I5** | `/rover/estimated_pose` | (없음, ROS2 표준 사용) | `geometry_msgs/PoseWithCovarianceStamped` |

→ 자세한 메시지 정의는 [docs/interfaces/msg/](../docs/interfaces/msg/) (현재 작성 중인 .msg 정의, isaac_interfaces/msg/ 로 이관 예정).

---

## 4. 책임 분리

| 영역 | 위치 | 담당 |
|------|------|------|
| **계약/스펙 문서** | `docs/interfaces/` | 성선규 (T4 PM) — Day 1 합의 + freeze |
| **빌드 정의 (.msg 등)** | `isaac_interfaces/msg/` | 성선규 (T4) — colcon 빌드용으로 이관 |
| **메시지 사용** | 각 트랙 노드 | 각 트랙 owner |

---

## 5. 변경 정책

**Breaking change 금지**:
- 필드 삭제 ❌
- 필드 타입 변경 ❌
- 필드 의미 변경 ❌
- 토픽 이름 변경 ❌

**추가는 OK**:
- 새 optional 필드 ✅
- 새 status 값 (default fallback 보장 시) ✅

변경 시 [docs/pm_tools/DECISIONS.md](../docs/pm_tools/DECISIONS.md)에 기록 + [docs/interfaces/INTERFACE_CONTRACTS.md](../docs/interfaces/INTERFACE_CONTRACTS.md) CHANGELOG 갱신.

---

## 6. 빌드

```bash
cd ~/dev_ws/rover_ws
colcon build --packages-select isaac_interfaces
source install/setup.bash

# 확인
ros2 interface show isaac_interfaces/msg/PerceptionResult
```

---

## 7. 한 줄 요약

> **5개 트랙이 합의한 통신 규격의 single source of truth.** docs/interfaces/는 계약/스펙, isaac_interfaces/는 colcon 빌드 정의. Day 1 freeze, 변경은 PM 승인.
