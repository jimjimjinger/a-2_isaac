# isaac_drive

> **트랙 owner**: 이찬휘 (T3 — Driving, **Critical Path**)
> **책임**: Mission FSM + Coverage planner + A* + 휠 명령 실행 + PPO wrapper

이전 명칭이 `isaac_navigation`이었음. `isaac_drive`로 rename — 실제 콘텐츠가 "주행 실행"이고 navigation은 그 내부 서브 모듈이라 명료성을 위해. (자세한 사유: [docs/STUDY_AND_PLAN.md Part XI](../docs/STUDY_AND_PLAN.md))

---

## 1. 모듈 역할

`isaac_drive`는 화성 로버의 **주행 두뇌 + 실행기**를 모두 담는 패키지입니다.

- **상위 흐름 (Mission Brain)**: FSM(EXPLORE/APPROACH/PICK/RETURN) + Coverage planner + A* 경로 계획
- **하위 실행 (Executor)**: PPO 정책 wrapper로 waypoint → wheel command 변환

다른 트랙에서 모이는 모든 정보(이찬휘는 받는 쪽)가 여기서 결정됨 → 이 패키지가 멈추면 시스템 멈춤.

---

## 2. 폴더 구조 (현재 상태)

```text
isaac_drive/
├─ isaac_drive/
│  ├─ __init__.py
│  ├─ drive_manager_node.py             ⏳ stub — 자율/수동 라우팅
│  ├─ mobile_base_executor_node.py      ⏳ stub — /cmd_vel publish
│  ├─ navigation/
│  │  ├─ __init__.py
│  │  ├─ mission_fsm.py                 ⏳ stub — 4 phase 상태 머신
│  │  ├─ coverage_planner.py            ⏳ stub — Roomba 알고리즘
│  │  └─ path_planner.py                ⏳ stub — A* (pyastar2d 권장)
│  └─ primitives/
│     ├─ drive_to_target.py             ⏳ stub
│     ├─ avoid_obstacle.py              ⏳ stub
│     └─ stop_rover.py                  ⏳ stub
├─ package.xml
└─ setup.py
```

**현재 모든 .py가 stub (0 bytes 또는 골격만).** 이찬휘가 Day 1부터 채울 자리.

---

## 3. 노드 구조 (계획)

```
[drive_manager_node]
   ↑ subscribe:  /perception/detections (T2 최진우, I2)
   ↑ subscribe:  /rover/estimated_pose (T5 이지민, I5)
   ↑ subscribe:  /mission/* (T4 성선규)
   │
   │  내부 모듈:
   │   - mission_fsm.py        ← phase 전환
   │   - coverage_planner.py   ← 미방문 셀 결정
   │   - path_planner.py       ← A* (이벤트 기반)
   │   - ppo_wrapper.py        ← isaac_rl 정책 호출
   │
   ↓ publish:    /mission/pick_request (T2 최진우 매니퓰레이션 트리거, I3)
   ↓ publish:    /mission/status, /mission/minimap, /mission/path (T4 성선규 UI)
   ↓ publish:    cmd_vel → [mobile_base_executor_node] → Isaac Sim
```

PPO 정책(`isaac_rl/policies/driving_policy.pt`)은 별도 패키지(isaac_rl)에서 로드. drive_manager는 wrapper로만 사용.

---

## 4. Critical Path — Day별 우선순위

| Day | 이찬휘 목표 | 검증 게이트 |
|:---:|------------|------------|
| 1 | Coverage planner 단독 검증 (numpy + matplotlib, **Isaac Sim 띄우지 마라**) | 10×10 grid 100% 도달 영상 |
| 2 | Isaac Sim 첫 통합 (빈 영역 sweep) | 클론 PPO로 sweep 1회 완주 |
| 3 | A* + 장애물 회피 | T1 김현중 첫 5개 terrain에서 작동 |
| 4 | FSM + 이지민 T5 pose 통합 | PoseProvider source 전환 |
| 5 | PICK + RETURN phase ⭐ | **End-to-end 1회 성공** |
| 6+ | 안정화, 시연 시나리오 | demo-stable-v1 tag |

핵심 패턴: **Vacuum Cleaner First + PoseProvider abstraction** ([T3_BRIEF §3-4](../docs/tracks/T3_BRIEF.md#3--핵심-빌드업-전략--vacuum-cleaner-first)).

---

## 5. 데이터 의존성

| 입력 | 출처 | 사용 |
|------|------|------|
| `obstacle_grid.npy` | T1 김현중 (I1) | A* path planner 입력 ([I1_TERRAIN_ASSETS §5](../docs/interfaces/I1_TERRAIN_ASSETS.md#5-obstacle_gridnpy-상세)) |
| `meta.json.minimap` | T1 김현중 (I1) | Coverage planner 셀 초기화 |
| `meta.json.basecamp` | T1 김현중 (I1) | FSM "is rover home?" 판정 |
| `/perception/detections` | T2 최진우 (I2) | EXPLORE → APPROACH 전환 트리거 |
| `/rover/estimated_pose` | T5 이지민 (I5) | 모든 모듈에서 위치 |
| `/mission/pick_response` | T2 최진우 (I4) | PICK phase 결과 |

---

## 6. PPO 정책 wrapper

클론의 학습된 정책 `isaac_rl/policies/driving_policy.pt` 를 그대로 사용 (재학습 X). drive_manager가 waypoint 인터페이스로 wrapping:

```python
# pseudo
class PPODriver:
    def step(self, waypoint, pose_provider):
        self._set_command_target(waypoint, pose_provider)
        actions = self.agent.act(observations)
        return actions   # [steering, throttle] tensors
```

상세: [T3_BRIEF §8](../docs/tracks/T3_BRIEF.md#8-ppo-wrapper--클론-재사용)

---

## 7. 의존 패키지

```bash
pip install pyastar2d  # A* 빠른 구현
# numpy, matplotlib 이미 있음
```

---

## 8. 한 줄 요약

> **이찬휘의 Mission Brain + 휠 명령 실행기.** 다른 모든 트랙(T1 김현중/T2 최진우/T5 이지민)이 모이는 곳. critical path라 늦으면 발표 미스.
