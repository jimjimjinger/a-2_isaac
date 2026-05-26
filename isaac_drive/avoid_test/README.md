# avoid_test — 6륜 로버 장애물 회피 강화학습

통합 차량 `vehicle_v1.usd` 의 **로버 부분**으로 *장애물을 피해 목표까지
주행* 하는 정책을 PPO(RSL-RL)로 학습하는 Isaac Lab manager-based RL 패키지.

m0609_lift_code_ver2 와 **같은 매니저 기반 구조**(씬·관측·액션·커맨드·이벤트·
보상을 선언형 Cfg 로 구성)를 그대로 따르며, 난이도를 단계적으로 올리는
**커리큘럼**(fixed → stage2 → avoid)으로 구성돼 있다.

> 자매 폴더 [`drive_test`](../drive_test/) 는 이 패키지의 레이캐스트 부착
> 방식·Ackermann 액션을 가져다 **WASD 수동 주행** 으로 만든 것이다.
> avoid_test 는 그 RL 버전(자동 회피 학습)에 해당한다.

---

## 1. 한 줄 개요

6륜 Mars Rover 가 정면의 장애물을 **레이캐스트로 감지**하고, **PPO 정책이
조향을 보정**해 부딪히지 않고 뒤쪽 목표까지 주행하는 법을 학습한다.
4096대 병렬 학습.

---

## 2. 디렉터리 구조

```
avoid_test/
├── README.md              ← 이 문서
├── STAGE1_BRIEFING.md      stage 1 학습 설정 브리핑 (초기 설계 스냅샷)
│
├── train.py               RSL-RL 학습 진입점
├── play.py                체크포인트 재생 + JIT/ONNX export
├── check_scene.py         씬 진단 — RL 없이 씬·센서·주행 점검
├── debug_fixed_obs.py     height-scan 이 큐브를 잡는지 점검
├── cli_args.py            RSL-RL 공용 CLI 인자 헬퍼
│
├── rover_avoid/           ★ 태스크 패키지 (gym 에 등록되는 본체)
│   ├── __init__.py        gym.register — 태스크 6종 등록
│   ├── rover.py           vehicle_v1 ArticulationCfg + 팔 HOME 고정 함수
│   ├── vehicle_env.py     VehicleAvoidEnv — 매 step 팔 HOME 고정
│   ├── avoid_env_cfg.py   기본 환경 Cfg (씬·관측·액션·보상·종료)
│   ├── fixed_obs_env_cfg.py   stage 1 — 고정 큐브 1개
│   ├── stage2_env_cfg.py      stage 2 — 랜덤 큐브 3개 + 먼 goal
│   ├── mdp/               커스텀 MDP 항목
│   │   ├── actions_cfg.py        Ackermann 액션 Cfg
│   │   ├── ackermann_actions.py  Ackermann 조향 모델 (6륜→휠 명령 변환)
│   │   ├── residual_action.py    잔차 Ackermann — RL 은 조향 보정만 출력
│   │   ├── observations.py       height-scan · 거리 · 방위각 관측
│   │   ├── rewards.py            보상 항목 9종
│   │   └── terminations.py       종료 조건 (도달/충돌/이탈)
│   └── agents/
│       └── rsl_rl_ppo_cfg.py     PPO 하이퍼파라미터
│
├── logs/rsl_rl/rover_avoid/   학습 로그·체크포인트 (run 별 폴더)
└── outputs/                   Hydra 실행 로그
```

---

## 3. 커리큘럼 — 태스크 6종

`rover_avoid/__init__.py` 가 gym 에 6개 태스크를 등록한다. 학습용(`-v0`)과
재생용(`-Play-v0`)이 쌍을 이룬다.

| Task ID | 단계 | 장애물 | goal | 비고 |
|---|---|---|---|---|
| `Isaac-Rover-FixedObs-v0` | **stage 1** | 고정 큐브 1개 (정면 3.5 m, 좌우 3 케이스) | 6.5 m 정면 고정 | 가장 쉬움 — 첫 단계 |
| `Isaac-Rover-Stage2-v0` | **stage 2** | 랜덤 큐브 3개 (경로상 분산) | 10~12 m | 연속 우회 학습 + progress 보상 |
| `Isaac-Rover-Avoid-v0` | (고난도) | 랜덤 박스 8개 (높이 0.8 m) | ±7 m 랜덤 | 못 넘는 큰 장애물 |

각 태스크에 `-Play-v0` 재생 변형이 있다 (`Isaac-Rover-FixedObs-Play-v0` 등).
재생 변형은 env 수를 줄이고 관측 노이즈를 끈다.

모든 태스크의 entry_point 는 `VehicleAvoidEnv` 를 가리킨다 — 학습/재생 중
m0609 팔을 HOME 으로 고정하기 위해서다.

---

## 4. 로봇 — vehicle_v1 (27 DOF)

`vehicle_v1.usd` = Mars Rover 베이스 + m0609 팔 + RG2-FT 그리퍼가 결합된
**단일 articulation**. 회피 학습에서는 로버 부분만 제어한다.

| 그룹 | 조인트 | 제어 |
|---|---|---|
| 조향 | `FL/FR/RL/RR_Steer_Revolute` (4) | position (stiffness 8000) |
| 구동 | 6개 휠 `*_Drive_Continuous` (6) | velocity (damping 4000) |
| 서스펜션 | `*_Rocker_Revolute`, `Differential_Revolute` (5) | 무동력 패시브 |
| m0609 팔 | `joint_1`~`joint_6` (6) | **HOME 자세 고정** |
| 그리퍼 | RG2-FT `*finger*`/`*knuckle*` (6) | 현 자세 고정 |

- 4륜 조향 + 6륜 구동 + 로커-보기(rocker-bogie) 서스펜션.
- **팔 HOME 고정**: m0609 팔은 회피 액션에 묶이지 않아 그냥 두면 물리
  시뮬레이션 중 흐트러진다(접힘→펴짐). `VehicleAvoidEnv` 가 매 step·reset
  직후 `keep_arm_folded()` 를 호출해 팔을 접힌 HOME 으로 직접 고정한다.
- 차량 에셋·Ackermann 기하 파라미터는 `rover_avoid/rover.py` 참조.

---

## 5. 센서 — 2개, 둘 다 로버 몸체(Body)에 부착

### ① height-scanner — 하향 RayCaster (장애물 감지용)

- 로버 몸체에 붙되 **몸체 위 10 m** 에 떠서 **수직 아래로** 광선을 쏜다 →
  위에서 내려다보는 *높이 스캐너*.
- **3×3 m 격자**, 해상도 0.2 m → **16×16 = 256개 광선**.
- 로버를 따라다니며 yaw 회전을 따라 같이 돈다 (롤·피치는 무시).
- 장애물은 별도 prim 이 아니라 **지형 메시의 '높이 돌출'** 로 포함된다 —
  RayCaster 는 메시 1개만 인식하므로 큐브를 height-field 안에 박아 넣는다.
- 평지는 ray 가 멀리 닿고(큰 값), 장애물 위는 가까이 닿는다(작은 값) →
  정책은 '작은 값 = 앞에 장애물' 을 학습한다.

### ② contact sensor — ContactSensor (충돌 감지용)

- 로버 몸체(Body)에 직접 부착. 몸체는 평소 공중에 떠 있으므로, net contact
  force 가 잡히면 = 장애물에 부딪힌 것.

> ⚠️ `STAGE1_BRIEFING.md` 는 초기 설계 스냅샷이라 4×4 m·441 ray·2D 액션으로
> 적혀 있다. **현재 코드는 3×3 m·256 ray·잔차 1D 액션** 이다 (아래 6·7절).

---

## 6. 관측 (259차원) — 정책 입력

| 항목 | 차원 | 함수 |
|---|---|---|
| 목표까지 거리 | 1 | `mdp.distance_to_goal` |
| 목표 방위각 | 1 | `mdp.angle_to_goal` |
| height-scan 격자 | 256 | `mdp.height_scan_grid` |
| 직전 행동 | 1 | `mdp.last_action` |

학습 시 관측 노이즈(corruption) ON, 재생(`-Play`) 시 OFF.

---

## 7. 행동 (1차원) — 잔차(Residual) Ackermann

이 패키지의 핵심 설계. 정책은 주행 전체가 아니라 **회피만** 학습한다.

```
베이스 컨트롤러(규칙기반)  →  goal 로 향하는 선속도·각속도
RL 정책(1차원 출력)        →  거기에 더할 조향 보정값
최종 각속도 = 베이스 각속도 + RL 조향 보정
최종 선속도 = 베이스 선속도 (고정 0.8 m/s)
```

- 장애물이 없으면 보정 0 → 베이스가 goal 로 직진.
- 장애물이 있으면 보정으로 우회.
- 'goal 찾아가기'는 베이스가 공짜로 해주므로, RL 은 '얼마나 비킬지' 만 배운다.
- 1D 보정값은 `ackermann()` 모델을 거쳐 6륜 구동 속도 + 4륜 조향각으로 변환된다.
- 구현: `rover_avoid/mdp/residual_action.py`, `ackermann_actions.py`.

---

## 8. 보상 — 9개 항목

`avoid_env_cfg.py` 의 `RewardsCfg` (stage 2 는 `progress` 추가 → 9개).

| 항목 | weight | 의미 |
|---|---|---|
| `goal_distance` | **+5.0** | 목표에 가까울수록 보상 (dense) |
| `goal_reached` | **+5.0** | 목표 도달 보너스 — 빨리 갈수록 큼 (sparse) |
| `heading` | **+3.0** | 목표를 바라볼수록 보상 |
| `angle_penalty` | −1.5 | 목표가 옆/뒤(\|angle\|>2rad)에 있으면 페널티 |
| `collision` | **−5.0** | 몸체 접촉센서 충돌 페널티 |
| `obstacle_proximity` | −1.5 | 장애물 0.9 m 이내 근접 페널티 (부딪히기 전 우회 유도) |
| `obstacle_hit` | **−5.0** | 레이캐스트로 잡은 충돌 (낮은 장애물 바퀴걸림 대응) |
| `oscillation` | −0.05 | 행동 급변 페널티 (부드러운 주행) |
| `steering_residual` | −0.1 | 조향 보정 크기 페널티 (장애물 없을 땐 보정 0 유도) |
| `progress` | +10.0 | **stage 2 전용** — 목표 방향 전진 속도 보상 (freeze 방지) |

설계 원칙: "목표로 가라(+) · 부딪히지·아슬아슬하지 마라(−)". 대부분 항목은
`max_episode_length` 로 나눠 per-step 크기를 작게 유지(에피소드 합 O(1) 스케일).

---

## 9. 종료 조건 — 5개

| 조건 | 결과 | 함수 |
|---|---|---|
| 시간 초과 (60초 = 300스텝) | 시간 초과 | `mdp.time_out` |
| 목표 0.5 m 이내 도달 | **성공** | `mdp.goal_reached` |
| 몸체 접촉센서 충돌 | 실패 | `mdp.collision` |
| 레이캐스트 충돌 (장애물 0.6 m 이내) | 실패 | `mdp.obstacle_hit` |
| 목표서 15 m 초과 이탈 | 실패 | `mdp.too_far_from_goal` |

---

## 10. 학습 설정 (PPO / RSL-RL)

`rover_avoid/agents/rsl_rl_ppo_cfg.py`.

- 신경망: MLP **[256, 128, 64]**, ELU 활성화 (actor·critic 동일 구조)
- gamma 0.99, lam 0.95, learning rate 1e-3 (adaptive, desired_kl 0.01)
- entropy coef 0.005, clip 0.2, num_steps_per_env 32, mini-batch 4
- max_iterations 1500, 체크포인트 저장 간격 25 iteration
- 제어 주기 5 Hz (sim 30 Hz ÷ decimation 6 → 0.2 s/스텝), 에피소드 60초 = 300스텝
- `experiment_name = "rover_avoid"` → 로그는 `logs/rsl_rl/rover_avoid/<날짜시각>/`

---

## 11. 실행

스크립트는 Isaac Lab 환경에서 실행한다 (`isaaclab` venv).

```bash
cd /home/rokey/dev_ws/rover_ws/src/a2_isaac/isaac_drive/avoid_test

# ── 학습 ──────────────────────────────────────────────
# stage 1 (고정 큐브 1개) — 4096대 병렬, 뷰어 없이
/home/rokey/dev_ws/IsaacLab/isaaclab.sh -p train.py \
    --task Isaac-Rover-FixedObs-v0 --num_envs 4096 --headless

# stage 2 (랜덤 큐브 3개)
/home/rokey/dev_ws/IsaacLab/isaaclab.sh -p train.py \
    --task Isaac-Rover-Stage2-v0 --num_envs 4096 --headless

# 과학습 collapse 방지 — 반복 수 제한 권장
#   --max_iterations 200

# ── 재생 (체크포인트 확인) ───────────────────────────
/home/rokey/dev_ws/IsaacLab/isaaclab.sh -p play.py \
    --task Isaac-Rover-FixedObs-Play-v0 \
    --checkpoint logs/rsl_rl/rover_avoid/<run>/model_<n>.pt

# ── 진단 (RL 없이 씬 점검) ───────────────────────────
/home/rokey/dev_ws/IsaacLab/isaaclab.sh -p check_scene.py --env fixed --num_envs 2
cat /tmp/avoid_scene_report.txt

# height-scan 이 큐브를 잡는지 점검
/home/rokey/dev_ws/IsaacLab/isaaclab.sh -p debug_fixed_obs.py --num_envs 2 --headless
cat /tmp/stage1_scan_check.txt
```

- `play.py` 는 재생과 동시에 정책을 `exported/policy.pt`(JIT)·`policy.onnx`
  로 내보낸다.
- 학습 진행은 TensorBoard 로 확인: `tensorboard --logdir logs/rsl_rl/rover_avoid`.

---

## 12. 주의 사항

### ⚠️ 큐브 타고넘기
fixed/stage2 의 큐브는 높이 **0.3 m** 로 낮아, 로커-보기 서스펜션 로버가
*타고 넘을* 수 있다. 재생에서 로봇이 큐브를 **우회** 하는지 꼭 확인할 것.
타고 넘으면 회피 학습이 안 된 것 — `cube_height` 를 0.6~0.8 m 로 키운다.
(그래서 레이캐스트 기반 `obstacle_hit` 종료·페널티로 바퀴 걸림을 잡는다.)

### ⚠️ 정책 collapse
정점을 찍은 뒤 과학습으로 무너질 수 있다. `--max_iterations` 로 제한하고,
지표가 정점일 때(목표 도달 최고 + 충돌 최저)의 체크포인트를 골라야 한다.

### 로봇이 제각각 움직이는 것은 정상
병렬 env 들이 같은 정책을 쓰지만 큐브 위치·목표·스폰이 모두 랜덤이라 경로가
다르다 — 동작을 외운 게 아니라 *상황을 보고 반응* 한다는(일반화) 좋은 신호.

---

## 13. 참고

- `STAGE1_BRIEFING.md` — stage 1 학습 설정 초기 브리핑. 일부 수치(4×4 m·
  441 ray·2D 액션)는 **초기 설계** 라 현재 코드와 다르다. 현재 코드 기준은
  이 README 의 6·7절.
- 차량 에셋은 `isaac_sim/assets/vehicle/vehicle_v1.usd` 를 경로 참조만 한다.
- 매니저 기반 구조는 m0609_lift_code_ver2 와 동일한 패턴을 따른다.
</content>
</invoke>
