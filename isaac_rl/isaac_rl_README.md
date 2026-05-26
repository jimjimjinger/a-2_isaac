# isaac_rl

> **트랙 owner**: 이찬휘 (T3 — Driving 트랙이 함께 다룸)
> **책임**: PPO 정책 inference (학습된 모델 그대로 사용) + 학습 환경 wrapper

---

## 1. 모듈 역할

- **inference**: 학습된 PPO 정책(`policies/driving_policy.pt`)을 ROS2 노드로 wrapping → drive_manager가 호출
- **environment**: 학습 시 재사용할 수 있는 Gym 호환 wrapper (재학습은 8일 스코프 아님)
- **training**: PPO trainer (선택적, 재학습 시)

8일에는 **클론의 학습된 정책을 그대로 inference**. 재학습 X. 자세한 결정: [docs/pm_tools/DECISIONS.md](../docs/pm_tools/DECISIONS.md).

---

## 2. 폴더 구조 (현재 상태)

```text
isaac_rl/
├─ isaac_rl/
│  ├─ __init__.py                  ⏳ stub
│  ├─ driving_policy_node.py       ⏳ stub — inference 노드
│  ├─ driving_policy.py            ⏳ stub — 정책 네트워크
│  ├─ policy_loader.py             ⏳ stub — .pt 로드
│  ├─ rl_environment.py            ⏳ stub — Gym wrapper
│  ├─ rl_trainer.py                ⏳ stub — PPO 학습 (선택)
│  ├─ ppo_wrapper.py               ⏳ stub — waypoint ↔ action 변환
│  └─ reward_function.py           ⏳ stub
├─ policies/
│  └─ driving_policy.pt            📦 학습된 weights (클론 재사용)
├─ package.xml
└─ setup.py
```

**모든 .py가 0 byte stub.** 이찬휘 작업 영역 (drive 트랙과 함께).

---

## 3. 작업 시작 가이드

| 자료 | 위치 |
|------|------|
| 트랙 onboarding | [docs/tracks/T3_BRIEF.md](../docs/tracks/T3_BRIEF.md) §8 PPO Wrapper |
| Claude Code context | [docs/tracks/T3_CLAUDE.md](../docs/tracks/T3_CLAUDE.md) |
| 클론 PPO 호출 예 | 클론의 `03_eval_ros2.py:298` (T3_BRIEF에 라인 명시) |

---

## 4. drive 트랙과의 관계

```
isaac_drive/drive_manager_node
   ↓ import + call
isaac_rl/ppo_wrapper (waypoint, pose) → torch tensor actions
   ↓ uses
isaac_rl/policies/driving_policy.pt  (학습된 weights)
```

**왜 별도 패키지인가**:
- 정책 weights 관리 (git LFS 후행 적용 시 분리 용이)
- 재학습 코드와 inference 코드 동거 (필요시)
- isaac_drive가 정책 implementation을 직접 import 안 하고 wrapper만 호출 → 정책 교체 시 isaac_drive 변경 없음

---

## 5. 한 줄 요약

> **클론의 PPO 정책을 그대로 inference. waypoint 인터페이스로 wrapping.** 이찬휘 T3가 drive 트랙과 함께 다룸. 8일에 재학습 안 함.
