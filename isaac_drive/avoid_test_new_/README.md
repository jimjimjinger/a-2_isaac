# avoid_test_new_ — Residual RL : 룰베이스 위에 강화학습 보정

`drive_test` 와 **완전히 동일한 환경**(terrain_00022 · vehicle_v1 · 하향
RayCaster · Ackermann 액션) 에서, `drive_test` 의 룰베이스 주행
컨트롤러(Goal-seek + 장애물 방향 회피) 를 **base controller** 로 깔아두고
그 위에 **RL 정책이 residual 보정만 학습** 하는 구조.

목표 = `drive_test` 의 룰베이스 수준의 거동을 RL 로 재현·미세 개선.

## 학습 전략 — Residual Policy Learning

```
RL 출력 (lin_res, ang_res) ──┐
                              ├─→ Ackermann 운동학 → 휠/조향 명령
룰베이스 base (lin_b, ang_b) ─┘
       │
       ├─ Goal-seek  : body-frame yaw_err P 제어
       └─ 장애물 방향 : raycaster prominence → 평균 lat 부호 → 회피 방향
                       (왼쪽 장애물 → 우회전 base, 오른쪽 → 좌회전 base)
                       정중앙·근접일수록 base 각속도↑
```

RL 정책의 마지막 layer 가중치를 0 으로 초기화하면 학습 시작 시 잔차 ≈ 0 →
차량은 룰베이스 그대로 거동한다.  학습이 진행되면서 정책이 base 위에 정밀한
보정(부드러움·옆구리 비킴·진입/탈출 타이밍)을 얹어간다.

## 폴더 구조 (자급자족)

```
avoid_test_new_/
├── README.md                     ← 본 파일
├── rover_vehicle.py              ← drive_test 에서 복사 (vehicle_v1, 27 DOF)
├── detector.py                   ← drive_test 에서 복사 (per-env obstacle 감지)
├── terrain_00022_new.usdc        ← drive_test 에서 복사 (지형+바위 병합 메시)
└── mdp/
    ├── __init__.py
    ├── ackermann_actions.py      ← drive_test 에서 복사 (운동학)
    ├── actions_cfg.py            ← drive_test 에서 복사 (AckermannActionCfg)
    └── residual_action.py        ← NEW : base + residual
```

아직 안 만든 것 (다음 단계):
  · `rover_env_cfg.py`     — ManagerBasedRLEnvCfg (씬·관측·보상·종료·이벤트)
  · `mdp/commands.py`      — random goal (xy) command term
  · `mdp/observations.py`  — body-frame goal, raycaster, prev action
  · `mdp/rewards.py`       — progress + collision + goal bonus + smoothness
  · `mdp/terminations.py`  — 도착 / 충돌 / timeout
  · `mdp/events.py`        — random spawn, random goal 샘플링
  · `agents/rsl_rl_ppo_cfg.py` — PPO 하이퍼파라미터
  · `train.py` / `play.py` — 학습·평가 진입점

## 핵심 디자인 결정

1. **무작위 goal + 무작위 spawn** — 일반화.  obstacle_grid 로 충돌·베이스캠프
   reject.  학습할 땐 매 에피소드 새 goal·spawn 샘플, 평가할 땐 mouse-click.
2. **Progress reward dense shaping** — `+α · (prev_dist - cur_dist)` 매 스텝.
   sparse goal-only 보상은 무작위 goal 에서 학습이 시작 안 됨.
3. **Body-frame 관측** — `(goal_dx_body, goal_dy_body)` + 레이캐스트 격자.
   월드 좌표로 주면 회전 invariance 까지 학습해야 해서 비효율.
4. **Residual + 0-init last layer** — 학습 시작 시점부터 룰베이스 거동.
   초기 탐색을 "뒤로 가기·빙빙 돌기" 단계 없이 건너뜀.

## RL 메클

`rsl_rl` + PPO.  `avoid_test/train.py` 구조 참고 (코드는 새로).
Isaac Lab 의 병렬 env 활용 — `num_envs=1024+` 로 학습 시간 단축.
