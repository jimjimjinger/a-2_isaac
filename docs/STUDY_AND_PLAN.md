# 프로젝트 학습 노트 & 8일 실행 계획

> 이 문서는 팀 5명 공동 학습용입니다. 클론 프로젝트(`June2December/Rokey6-B1-Isaac-simulation-project`)를 이해하고, 그 위에 우리만의 차별화를 어떻게 얹을지까지 정리돼 있어요.
>
> Q&A 형식으로 학습 흐름을 그대로 보존했습니다. 앞에서부터 순서대로 읽으면 자연스럽게 결론에 도달합니다.

---

## 📑 목차

- **[Part I — 클론 프로젝트 이해](#part-i--클론-프로젝트-이해)**
  1. [전체 구조와 한 줄 요약](#1-전체-구조와-한-줄-요약)
  2. [자율주행은 어떻게 이뤄지는가](#2-자율주행은-어떻게-이뤄지는가)
  3. [강화학습은 어떻게 쓰였는가](#3-강화학습은-어떻게-쓰였는가)
- **[Part II — PPO와 Ackermann 기초](#part-ii--ppo와-ackermann-기초)**
  4. [PPO란?](#4-ppo란)
  5. [Ackermann 조향 모델이란?](#5-ackermann-조향-모델이란)
  6. [둘은 어떻게 연결되는가](#6-둘은-어떻게-연결되는가)
- **[Part III — PPO 학습 깊이 들여다보기](#part-iii--ppo-학습-깊이-들여다보기)**
  7. [강화학습의 4가지 빌딩블록](#7-강화학습의-4가지-빌딩블록)
  8. [학습 환경을 구성한다는 것](#8-학습-환경을-구성한다는-것)
  9. [Reward 설계 — RL의 가장 어려운 부분](#9-reward-설계--rl의-가장-어려운-부분)
  10. [PPO 학습 사이클 한 번 따라가기](#10-ppo-학습-사이클-한-번-따라가기)
  11. [이 프로젝트의 실제 학습 — 숫자로 보기](#11-이-프로젝트의-실제-학습--숫자로-보기)
- **[Part IV — 짧은 Q&A 묶음](#part-iv--짧은-qa-묶음)**
  12. [보상의 개수와 항목은 사용자가 설계하는가?](#12-보상의-개수와-항목은-사용자가-설계하는가)
  13. [Isaac Sim의 RL은 Isaac Lab으로?](#13-isaac-sim의-rl은-isaac-lab으로)
  14. [내 PC 사양으로 돌아갈까?](#14-내-pc-사양으로-돌아갈까)
  15. [초기 행동은 완전한 난수인가?](#15-초기-행동은-완전한-난수인가)
  16. [보상은 에피소드 끝에 한 번 적용?](#16-보상은-에피소드-끝에-한-번-적용)
- **[Part V — 클론 프로젝트의 솔직한 평가](#part-v--클론-프로젝트의-솔직한-평가)**
- **[Part VI — 우리 프로젝트의 방향](#part-vi--우리-프로젝트의-방향)**
  17. [skrl이 뭔지부터](#17-skrl이-뭔지부터)
  18. [Isaac Sim의 진짜 강점](#18-isaac-sim의-진짜-강점)
  19. [우주 주제 × Isaac Sim 강점 매칭 — 차별화 4가지](#19-우주-주제--isaac-sim-강점-매칭--차별화-4가지)
- **[Part VII — 제약 반영 첫 계획 (v1)](#part-vii--제약-반영-첫-계획-v1)**
- **[Part VIII — 화성 물성 미적용 확인 → 계획 갱신 (v2)](#part-viii--화성-물성-미적용-확인--계획-갱신-v2)**
- **[Part IX — 지형 random성 부분 적용 확인 → 최종 컨셉 (v3)](#part-ix--지형-random성-부분-적용-확인--최종-컨셉-v3)**
- **[Part X — 즉시 합의가 필요한 결정 항목](#part-x--즉시-합의가-필요한-결정-항목)**

---

# Part I — 클론 프로젝트 이해

## 1. 전체 구조와 한 줄 요약

> ❓ **이제 본격적으로 이 프로젝트에 관해서 이해를 도와줘. 먼저 트리구조를 이해하게 해줘서 개요를 잡으면 좋겠어. 그 다음 설명은 네가 생각하기에 최선의 방식대로 풀어가줘. 특히, 이 시뮬레이션에서 rover의 자율주행은 어떻게 이뤄지는지, 강화학습이 어떻게 쓰였는지도 꼭 알려줘. README가 너무 부족한 것 같아서, 필요하다면 그것도 수정/강화 부탁해.**

이 프로젝트의 큰 그림은 **"Isaac Sim/Isaac Lab 위에 강화학습으로 학습된 PPO 정책이, Ackermann 조향 모델을 통해 6륜 로버를 제어하면서 화성 지형에서 광물을 수집하고, 그 상태를 ROS2로 외부 모니터에 퍼블리시한다"** 입니다.

```
Rokey6-B1-Isaac-simulation-project/
├── readme.md, INSTALL.md           # 문서
├── setup_env.sh, verify_env.sh     # 환경변수 + 설치 검증
├── requirements.txt                # gymnasium, isaaclab, isaacsim, skrl, torch
│
└── rover/
    ├── sim/                        ⭐ 시뮬레이션 메인
    │   ├── run_ros2.sh             # ★ 진입점 ① (rviz + Isaac Sim 동시 실행)
    │   ├── scripts/
    │   │   ├── 03_eval_ros2.py     # ★ 진입점 ② 실제 미션 루프 (PPO 추론 + ROS2 pub)
    │   │   ├── 02_eval.py          # 학습된 정책 단독 평가 (ROS 없음)
    │   │   ├── 01_drive_test.py    # 단순 주행 테스트
    │   │   ├── mission_monitor.py  # 별도 터미널 텍스트 대시보드
    │   │   └── mission_viz_node.py # rviz용 Marker 퍼블리시 노드
    │   ├── mission/                # 미션 단위 헬퍼 (베이스캠프, 카메라, 텔레포트)
    │   │   ├── scene_utils.py      # 베이스캠프 마커 USD 스폰
    │   │   ├── camera_utils.py     # 듀얼 1인칭 뷰포트 설정
    │   │   ├── command_utils.py    # ★ 목표(광물/베이스) 강제 주입 로직
    │   │   └── robot_utils.py      # 로봇 텔레포트
    │   ├── config/mission_monitor.rviz
    │   │
    │   └── rover_envs/             ⭐⭐ 학습 환경 패키지 (Isaac Lab 표준 구조)
    │       ├── assets/             # USD 로봇·지형·텍스처
    │       ├── mdp/                # 공용 MDP 부품 (actions/, recorders/)
    │       ├── envs/navigation/    # 네비게이션 태스크 정의
    │       │   ├── rover_env_cfg.py    # ★ 환경 설정 (관측/보상/종료/scene)
    │       │   ├── entrypoints/        # gym.make 에 register되는 RoverEnv
    │       │   ├── mdp/                # ★ navigation 전용 MDP
    │       │   │   ├── observations.py
    │       │   │   ├── rewards.py      # ★ 8개 보상 함수
    │       │   │   ├── terminations.py
    │       │   │   └── randomizations.py
    │       │   ├── robots/aau_rover/
    │       │   │   ├── env_cfg.py
    │       │   │   ├── __init__.py       # ★ gym.register("AAURoverEnv-v0")
    │       │   │   └── policies/         # ★ best_agent_ppo.pt (학습된 정책)
    │       │   ├── learning/skrl/
    │       │   │   ├── models.py         # ★ Policy/Value 네트워크 (Conv+MLP)
    │       │   │   └── configs/          # PPO/TRPO/RPO/SAC/TD3 yaml
    │       │   └── utils/terrains/       # 지형 임포트, 명령 생성기
    │       ├── learning/agents/skrl.py   # PPO/TRPO/... 에이전트 팩토리
    │       └── utils/                    # 로깅, config, 모델 다운로드
    │
    └── analysis/                   # 학습/평가 결과 분석
        ├── analyze_models_*.py     # wandb csv 비교 차트
        ├── analysis_missions.py    # mission_log csv 대시보드
        ├── csv_logs/csv_PPO,TRPO/
        └── result_charts/          # 결과 PNG들
```

**중요 디렉터리 3개만 기억:**

| 위치 | 역할 | 비유 |
|------|------|------|
| `rover/sim/scripts/` | 진입점 스크립트 (사람이 실행) | "메인 함수" |
| `rover/sim/rover_envs/` | gym 환경 + RL 모델 정의 | "게임 룰 + 두뇌" |
| `rover/sim/mission/` | 미션 시나리오 헬퍼 | "각본/연출" |

---

## 2. 자율주행은 어떻게 이뤄지는가

이 시스템의 자율주행은 흔히 떠올리는 "센서 → 인지 → 경로 계획 → 제어" 파이프라인과는 다릅니다. **end-to-end 강화학습 정책 한 덩어리가 관측 → 행동을 직접 매핑**하고, 그 출력을 Ackermann 모델이 6륜 휠 명령으로 풀어냅니다.

### 2.1 데이터 흐름 (한 스텝)

```
[Isaac Sim 환경]
  ├ 로봇 root pose, quaternion (GT 사용)
  ├ height_scanner: RayCaster (5m × 5m, 0.05 해상도) → 10000개 높이 샘플
  ├ contact_sensor: 휠/바디가 obstacles(돌)에 접촉했는지
  └ command_manager: 목표 위치(target_pose) 4-vector
        │
        ▼
[Observation] (rover_envs/.../mdp/observations.py)
  · last_action (2) + distance (1) + heading (1) + angle_diff (1) + height_scan (10000)
  · 총 10005차원
        │
        ▼
[PPO 정책 — GaussianPolicyConv]
  · ConvHeightmapEncoder (8→16→32→64) → MLP [80,60]
  · proprio(5) ⊕ conv_out → MLP [256,160,128]
  · → Linear(2) → Tanh → μ ∈ [-1,1]²
  · + nn.Parameter log_std → Gaussian(μ,σ)
        │  action = [steer_norm, throttle_norm]
        ▼
[Ackermann 변환 — mdp/actions/ackermann_actions.py]
  · lin_vel, ang_vel = action × scale
  · turning_radius = |v|/|ω|
  · if R < 0.8·d_mw: point-turn 모드 (좌·우 휠 반대, 조향각 ±π/4)
  · else: 휠별 r_i 계산 → v_i = r_i·ω, θ_i = atan2(L/2±offset, r_i)
        │
        ▼
4 steer joint positions + 6 drive joint velocities
        │
        ▼
Isaac Sim PhysX (dt=1/30 × decimation 6 = 5Hz 의사결정)
        │
        └────► 다음 step의 [환경 상태]
```

### 2.2 "자율"이라고 부를 수 있는 이유

**(a) 정책 자체의 자율성** — PPO가 학습한 "관측 → 조향·가속" 매핑은 어떤 지형이 와도 같은 함수로 처리. heightmap을 읽고 돌을 피해서, 목표 방향으로 적절한 속도와 조향각을 자동으로 결정.

**(b) 미션 수준의 자율성** — `03_eval_ros2.py`의 `run_mission()` 루프가 "광물 5개 수집 → 베이스캠프 복귀 → 다음 라운드" 사이클을 자동 관리:

```python
# 03_eval_ros2.py:292 부근
if phase == "collect" and distance < 0.5 and abs(angle) < 0.2:
    collected += 1
    if collected >= 5:
        phase = "return"
        set_command_to_basecamp(...)
    else:
        resample_target(...)
elif phase == "return" and distance < 2.0:
    teleport_to_basecamp(...)
```

### 2.3 한 가지 중요한 단순화

이 시스템은 **인지(perception) 모듈이 없습니다**. 광물·베이스캠프의 좌표는 시뮬레이션이 직접 알려주는 ground truth고, 정책은 "목표까지의 상대 벡터"를 그냥 입력으로 받아요. 장애물 회피는 **heightmap(10000개 raycast)**가 담당합니다.

---

## 3. 강화학습은 어떻게 쓰였는가

### 3.1 RL 문제 정의 (MDP)

| 요소 | 정의 |
|------|------|
| **State (관측)** | 5 proprio + 10000 heightmap = 10005차원 |
| **Action** | 연속 2차원: `[steer ∈ [-1,1], throttle ∈ [-1,1]]` |
| **Reward** | 아래 8개 항의 가중합 |
| **Termination** | 충돌 / 도달 성공 / 타임아웃(150초) |
| **Episode 길이** | `episode_length_s = 150s`, dt=1/30, decimation=6 → 약 750 steps |
| **병렬 환경** | 학습 시 `num_envs=128` 동시 시뮬레이션 |

### 3.2 ⭐ 8개 보상 항 (README에 가장 중요한 부분)

[rover_env_cfg.py:120-160](rover/sim/rover_envs/envs/navigation/rover_env_cfg.py#L120-L160) 의 `RewardsCfg`:

| # | 이름 | 가중치 | 카테고리 | 핵심 수식·로직 |
|---|------|:--:|----------|----------------|
| 1 | `distance_to_target` | **+5.0** | Navigation | `1 / (1 + 0.11·d²)` — 가까울수록 + |
| 2 | `reached_target` | **+5.0** | Navigation | 목표 0.18m 이내 + 각도 0.1rad 이내 → 시간 남을수록 큰 + |
| 3 | `oscillation` | **-0.05** | Stability | 직전 action과 차이 제곱 → 떨림 페널티 |
| 4 | `angle_to_target` | **-1.5** | Alignment | 목표 각도 \|θ\|>2.0rad일 때 페널티 |
| 5 | `heading_soft_contraint` | **-0.5** | Alignment | throttle<0 (후진) → 페널티 |
| 6 | `collision` | **-3.0** | Stability | contact_sensor force>1N → 페널티 |
| 7 | `far_from_target` | **-2.0** | Navigation | target_distance+3m 초과 → 페널티 |
| 8 | `angle_diff` (=`angle_to_goal_reward`) | **+5.0** | Alignment | `1/(1+d) · 1/(1+|θ|)` |

**그룹별 의도:**
- **Navigation** (1·2·7): "목표로 다가가라 / 도달해라 / 너무 멀어지지 마라"
- **Alignment** (4·5·8): "목표 방향을 바라보고, 후진하지 마라"
- **Stability** (3·6): "떨지 마라, 부딪히지 마라"

### 3.3 학습 알고리즘 (PPO 기본)

`rover_ppo.yaml`의 핵심 하이퍼파라미터:

```yaml
rollouts: 60              # 60 step씩 모아서 한 번 학습
learning_epochs: 4
mini_batches: 60
discount_factor: 0.99     # γ
lambda: 0.95              # GAE
learning_rate: 1e-4
ratio_clip: 0.2           # PPO clip의 ε
kl_threshold: 0.008
trainer.timesteps: 1_000_000
```

**총 학습량**: 1M timesteps × 128 envs ≈ **1.28억 경험 샘플**.

### 3.4 학습 → 추론 흐름

```
학습 (이 레포 밖에서 수행됨, 추정 IsaacLab 표준 trainer)
  → num_envs=128 동시 rollout
  → skrl PPO가 60 step씩 모아 update
  → 1M steps 후 best_agent_ppo.pt 저장
  → wandb로 reward/loss 기록 (analysis/csv_logs/ 에 익스포트 보관)
       │
       ▼
추론 (03_eval_ros2.py)
  agent = create_agent("PPO", env, cfg)
  agent.load("best_agent_ppo.pt")
  agent.set_running_mode("eval")
  
  while running:
      actions = agent.act(states)
      actions[:, 0] = torch.clamp(actions[:, 0] * 2.0, -1.0, 1.0)  # ← 조향 강제 증폭
      env.step(actions)
```

### 3.5 비교 알고리즘

`learning/skrl/configs/`에는 **PPO 외에도 TRPO/RPO/SAC/TD3** yaml. `analysis/csv_logs/`에 PPO와 TRPO의 wandb 로그 둘 다 있음. 최종 선택은 PPO.

> 📝 위 내용을 더 잘 정리한 별도 README는 [`README.enhanced.md`](README.enhanced.md) 참조.

---

# Part II — PPO와 Ackermann 기초

> ❓ **PPO가 뭐고 Ackermann 변환기란?**

## 4. PPO란?

### 4.1 한 줄로
**"강화학습 알고리즘 중 하나로, 정책(policy)을 안전한 보폭으로 점진적으로 개선하는 방법"**

### 4.2 강화학습 기본 그림

```
    ┌─────────┐  action  ┌──────────────┐
    │  Agent  │ ───────▶ │ Environment  │
    │ (policy)│          │  (시뮬레이터)│
    └─────────┘          └──────┬───────┘
         ▲                      │
         │ reward, observation  │
         └──────────────────────┘
```
- **Policy(정책) π(a|s)**: "상태 s에서 어떤 행동 a를 할까"를 정하는 함수
- **목표**: 누적 보상의 기댓값을 최대화하는 정책 찾기

### 4.3 PPO의 핵심 아이디어

정책 업데이트를 한 번에 크게 하면 망가짐. **"한 번에 너무 멀리 가지 말자"**:

```
r(θ) = π_new(a|s) / π_old(a|s)         ← 새/옛 정책의 행동 확률 비율

L(θ) = E[ min( r(θ)·A,  clip(r(θ), 1-ε, 1+ε)·A ) ]
                                          │
                                  ε=0.2 (rover_ppo.yaml)
```
- `A` = Advantage (이 행동이 평균보다 얼마나 좋았는지)
- `r(θ)`가 1.2를 넘으면 → clip해서 보상 끊기 → "그만 더 크게 바꾸지 마"

"**Proximal**(가까운)" 이름의 유래.

### 4.4 PPO vs 다른 RL

| 알고리즘 | 핵심 차이 | 안정성 | 샘플 효율 |
|----------|----------|:------:|:---------:|
| REINFORCE | 가장 기본, 보폭 제한 없음 | ❌ | ❌ |
| TRPO | KL divergence를 엄격 제약, 수학 복잡 | ✅ | ✅ |
| **PPO** | clip으로 간단히 비슷한 효과 | ✅ | ✅✅ |
| SAC/TD3 | off-policy, 메모리 사용량 큼 | ✅ | ✅✅ |

**PPO가 인기**: TRPO만큼 안정적이면서 구현 간단하고 빠름. 로보틱스 시뮬의 사실상 표준.

---

## 5. Ackermann 조향 모델이란?

### 5.1 한 줄로
**"자동차가 곡선 도로를 매끄럽게 돌 때, 안쪽 바퀴와 바깥쪽 바퀴를 다르게 꺾는 기하학적 모델"**

### 5.2 왜 필요한가

곡선 회전 중 바퀴별 회전반경이 다름:
- 안쪽: 반경 작음 → **느린 속도 + 큰 조향각**
- 바깥쪽: 반경 큼 → **빠른 속도 + 작은 조향각**

같이 꺾으면 미끄러짐.

### 5.3 핵심 공식

```
R = L / tan(δ)        (L=wheelbase, δ=평균 조향각)
R_i = R ± d/2          (각 바퀴별 회전반경)
θ_i = atan2(L, R_i)    (각 바퀴별 조향각)
v_i = R_i · ω          (각 바퀴별 속도)
```

### 5.4 이 프로젝트의 6륜 + 4륜 조향

```
       FL ●─Steer  ●─Steer FR     ← 앞축
              │
        ML ●     ● MR             ← 중축 — 조향 X, 굴림만
              │
       RL ●─Steer  ●─Steer RR     ← 뒷축
```

[ackermann_actions.py:181-275](rover/sim/rover_envs/mdp/actions/ackermann_actions.py#L181-L275) 의 단순화:

```python
def ackermann(lin_vel, ang_vel, cfg, device):
    L  = cfg.wheelbase_length              # 0.849m
    d_mw = cfg.middle_wheel_distance       # 0.894m
    d_fr = cfg.rear_and_front_wheel_distance  # 0.77m
    
    R = |lin_vel| / |ang_vel|              # 회전 반경
    
    # 6개 휠 각각의 회전 반경
    r_ML = R - d_mw/2 · sign(ω)
    r_MR = R + d_mw/2 · sign(ω)
    r_FL = R - d_fr/2 · sign(ω)
    ...
    
    # 휠 속도 = 반경 × 각속도
    v_i = r_i · |ω| · direction
    
    # 조향각
    θ_FL = atan2(L/2 - offset, r_FL)
    ...
    
    # 너무 작은 반경 → point-turn
    if R < 0.8 · d_mw:
        # 좌우 휠 반대방향, 조향각 ±π/4 고정
```

### 5.5 Point-turn 모드 — 화성 로버 특수 기능

`R < 0.8 × d_mw`이면 자동 전환 → **제자리 회전(탱크처럼)**. 좁은 공간 탈출 시 유용.

---

## 6. 둘은 어떻게 연결되는가

```
관측 → 🧠 PPO 정책 → [steer, throttle] (의도) → 🔧 Ackermann → 6륜 명령 → PhysX
```

**왜 분리했나?** PPO에게 10차원(휠 6 + 조향 4)을 직접 학습시키면:

| 직접 학습 | 분리(현재) |
|----------|----------|
| Action 차원 = 10 | Action 차원 = 2 |
| 학습 매우 어려움 | 학습 빠르고 안정 |
| 정책이 차량 기하학을 재학습 | Ackermann이 알려진 수식 처리 |
| 다른 차체 재사용 불가 | 차체만 바꿔도 정책 재사용 |

→ **"action abstraction" / "hierarchical control"**. RL은 고차원 의사결정, Ackermann은 저수준 제어.

**비유**: PPO = 운전자 두뇌. Ackermann = 자동차 조향 메커니즘 + 차동기어.

---

# Part III — PPO 학습 깊이 들여다보기

> ❓ **PPO에 대해서 더 알아보고 싶고, 어떻게 학습시킨건지 궁금해. 나는 강화학습 지식이 부족해서, 어떻게 학습 환경을 구성해주고 어떻게 점수를 부여하는지 등이 궁금해.**

## 7. 강화학습의 4가지 빌딩블록

### ① 환경 (Environment)
- "행동을 받으면 다음 상태와 보상을 돌려주는 함수"
- 이 프로젝트 → Isaac Sim PhysX

```python
obs, info = env.reset()
obs, reward, terminated, truncated, info = env.step(action)
```

### ② 에이전트 (Agent)
- 정책 함수 π(action | observation) 보유. 보통 신경망
- 이 프로젝트 → `GaussianPolicyConv` + `ValueNetworkConv`

### ③ 에피소드 (Episode)
- reset → step ... → 종료까지가 1 에피소드
- 이 프로젝트 → 150초 ≈ **750 step**

### ④ 보상 (Reward)
- 매 step 환경이 돌려주는 스칼라
- 목표: 누적 보상 기댓값 최대화

> ✨ 학습이란 결국 **"누적 보상이 큰 행동의 확률을 높이고, 작은 행동의 확률을 낮추는 것"**.

---

## 8. 학습 환경을 구성한다는 것

[rover_env_cfg.py](rover/sim/rover_envs/envs/navigation/rover_env_cfg.py) 한 파일이 환경 구성의 전부:

```python
class RoverEnvCfg(ManagerBasedRLEnvCfg):
    scene: RoverSceneCfg = ...          # 1) 무대
    actions: ActionsCfg = ...           # 2) Action 공간 (= Ackermann)
    observations: ObservationCfg = ...   # 3) Observation 공간
    rewards: RewardsCfg = ...           # 4) ★ 보상 함수들
    terminations: TerminationsCfg = ... # 5) 종료 조건
    commands: CommandsCfg = ...         # 6) 목표 생성
    events: EventCfg = ...              # 7) reset 시 동작
```

### 8.1 Scene
```python
class RoverSceneCfg(MarsTerrainSceneCfg):
    dome_light    = ...
    sphere_light  = ...
    robot         = ...
    contact_sensor = ContactSensorCfg(...)
    height_scanner = RayCasterCfg(
        pattern_cfg=patterns.GridPatternCfg(resolution=0.05, size=[5.0, 5.0]),
        # 100×100 = 10000 ray
    )
```
→ **센서 종류·해상도가 관측의 차원을 결정**.

### 8.2 Action Space
```python
self.actions.actions = mdp.AckermannActionCfg(
    wheelbase_length=0.849,
    middle_wheel_distance=0.894,
    ...
)
```
`action_dim = 2` ← `[steer, throttle] ∈ [-1, 1]²`.

### 8.3 Observation Space
```python
class PolicyCfg(ObsGroup):
    actions     = ObsTerm(func=mdp.last_action)                    # 2
    distance    = ObsTerm(..., scale=0.11)                          # 1
    heading     = ObsTerm(..., scale=1/math.pi)                     # 1
    angle_diff  = ObsTerm(..., scale=1/math.pi)                     # 1
    height_scan = ObsTerm(..., scale=1)                             # 10000
```

**중요 포인트:**
- `scale` 파라미터 = 정규화 (NN 입력은 -1~1 근방이 best)
- `enable_corruption=True` → 학습 시 입력 노이즈 추가 (sim2real robust)
- `concatenate_terms=True` → 평탄한 10005-vector

### 8.4 Termination
```python
time_limit = DoneTerm(func=mdp.time_out, time_out=True)
is_success = DoneTerm(func=mdp.is_success, params={"threshold": 0.2})
collision  = DoneTerm(func=mdp.collision_with_obstacles, params={"threshold": 0.001})
```

`terminated`(나쁜 종료) vs `truncated`(시간 만료) 구분:
- `terminated=True` → 이후 보상 0
- `truncated=True` → bootstrap (value network 이용)

### 8.5 Commands
```python
target_pose = TerrainBasedPositionCommandCfg(
    resampling_time_range=(150.0, 150.0),
    ranges=...heading=(-math.pi, math.pi),
)
```
랜덤 목표 자동 생성 → 정책이 "어디서든 목표가 주어지면 가는" 일반화 학습.

### 8.6 Events (Reset)
[randomizations.py:11](rover/sim/rover_envs/envs/navigation/mdp/randomizations.py#L11):
```python
def reset_root_state_rover(env, env_ids, asset_cfg, z_offset=0.5):
    spawn_locations = terrain.get_spawn_locations()
    spawn_index = torch.randperm(...)[:len(env_ids)]
    angle = torch.rand(...) * 2 * torch.pi
    # 로봇 위치/방향 적용
```
**도메인 랜덤화**: 매 에피소드 시작 위치·방향 랜덤 → 특정 시작조건 과적합 방지.

---

## 9. Reward 설계 — RL의 가장 어려운 부분

### 9.1 Sparse vs Dense

| 종류 | 예 | 장단점 |
|------|-----|--------|
| Sparse | "목표 도달 시에만 +1" | 깔끔하지만 신호 드물어 학습 거의 불가 |
| Dense | "가까워질수록 매 step +α" | 학습은 잘 되지만 잘못된 행동 유도 위험 |

대부분 로보틱스 RL = **shaped dense reward**.

### 9.2 8개 보상 함수 코드 한 줄씩

**Navigation 그룹**

```python
# #1 distance_to_target (+5.0)
return (1.0 / (1.0 + (0.11 * distance * distance))) / env.max_episode_length
# 1/(1+0.11·d²) 종 모양 → 목표 근처에서 급격히 커짐
```
```python
# #2 reached_target (+5.0)
return torch.where(
    (distance < 0.18) & (torch.abs(angle) < 0.1), 
    2.0 * (time_steps_to_goal / env.max_episode_length), 0.0
)
# 빨리 도달할수록 큰 보상 + 자세도 정렬 요구
```
```python
# #7 far_from_target (-2.0)
threshold = env.scene.terrain.target_distance + 3
return torch.where(distance > threshold, 1.0, 0.0)
# 잘못된 방향 헤매기 방지
```

**Alignment 그룹**

```python
# #4 angle_to_target (-1.5)
return torch.where(torch.abs(angle) > 2.0,    # ~115° 이상만
                   torch.abs(angle) / env.max_episode_length, 0.0)
```
```python
# #5 heading_soft_contraint (-0.5)
return torch.where(env.action_manager.action[:, 0] < 0.0,
                   (1.0 / env.max_episode_length), 0.0)
# soft constraint — 정말 필요할 땐 후진 허용
```
```python
# #8 angle_to_goal_reward (+5.0)
angle_reward = (1 / (1 + distance)) * 1 / (1 + torch.abs(angle_b))
# 곱셈 = AND 조건 (거리 + 정렬 둘 다 만족 시 큼)
```

**Stability 그룹**

```python
# #3 oscillation (-0.05)
angular_penalty = torch.where(angular_diff*3 > 0.05, torch.square(angular_diff*3), 0.0)
angular_penalty = torch.pow(angular_penalty, 2)  # 4승!
# 작은 떨림 무시, 큰 떨림 폭발적 페널티
```
```python
# #6 collision (-3.0)
forces_active = torch.sum(normalized_forces, dim=-1) > 1
return torch.where(forces_active, 1.0, 0.0)
# 접촉력 1N 초과 → 페널티 (termination 동시 발동)
```

### 9.3 잘 짠 부분
1. **양수+음수 균형**: Navigation/Alignment 양수, Stability 음수
2. **Dense+Sparse 혼합**
3. **`/ env.max_episode_length` 정규화**: 에피소드 길이 무관 안정
4. **곱셈 조합 (#8)**: AND 조건 표현
5. **임계 기반 (#3, #4)**: 작은 변화 무시, 큰 변화만 처벌

### 9.4 Reward Hacking 방어

| 가능한 hack | 이 프로젝트의 방어 |
|-------------|-------------------|
| 목표 주변 빙빙 돌기 | #2 angle 조건 + #8 정렬 보상 |
| 천천히 가며 보상 누적 | #2 시간 효율 + episode time limit |
| 후진으로 우회 | #5 후진 페널티 |
| 떨면서 도달 | #3 떨림 페널티 |

---

## 10. PPO 학습 사이클 한 번 따라가기

### 10.1 큰 그림

```
┌─ 1 iteration ──────────────────────────────────────────────────┐
│ ① ROLLOUT: 현재 π_old로 60 step × 128 env = 7680 transition    │
│ ② ADVANTAGE: V(s) 추정 → GAE-λ로 A(s,a) 계산                   │
│ ③ POLICY UPDATE: 4 epoch × 60 mini-batch, clipped loss         │
│ ④ π_old ← π_new, ①로                                           │
└────────────────────────────────────────────────────────────────┘
1M timesteps / 60 ≈ 16,666 iteration 반복
```

### 10.2 Rollout 단계

128 환경이 **동시에** 시뮬레이션 (GPU 병렬화 핵심).

각 transition:
- `s` (observation): 10005-dim
- `a` (action): 2-dim
- `log_π_old(a|s)`: 그때 정책의 로그 확률
- `r` (reward), `V(s)`, `terminated`, `truncated`

### 10.3 Advantage 계산

**Advantage** A(s, a) = "행동 a를 했을 때 받은 누적 보상이 V(s) 예측보다 얼마나 좋았나"

**GAE-λ** (Generalized Advantage Estimation):
```
δ_t = r_t + γ·V(s_{t+1}) - V(s_t)    ← TD error
A_t = δ_t + (γλ)·δ_{t+1} + (γλ)²·δ_{t+2} + ...
```
- `λ=0` → 가까운 TD만 (bias↑, variance↓)
- `λ=1` → 전체 return (bias=0, variance↑)
- `λ=0.95` → **sweet spot**

### 10.4 PPO 핵심 수식

```
ratio = π_new(a|s) / π_old(a|s)

L_clip = -E[ min(
    ratio · A,
    clip(ratio, 1-ε, 1+ε) · A
)]

L_value = E[(V_new(s) - G)²]
L_total = L_clip + c_v · L_value - c_e · entropy
```

### Clip 작동

```
A > 0 (좋은 행동):                  A < 0 (나쁜 행동):
                                          
    L_clip                              L_clip
      ▲                                   ▲
      │   ▁▁▁▁▁                            │
      │  /                                 │  ▁▁▁▁
      │ /                                  │ /
      │/                                   │/
   ───┼──────────▶ ratio                ───┼───▶ ratio
      0    1   1.2                          0   0.8  1
```

`ratio > 1.2` → clip 발동 → 정책 변화 멈춤. **"보폭 제한"의 수학적 실체**.

`kl_threshold: 0.008` → KL이 넘으면 **early stop**. 이중 안전장치.

---

## 11. 이 프로젝트의 실제 학습 — 숫자로 보기

### 11.1 학습 규모

| 항목 | 값 |
|------|-----|
| `num_envs` | 128 |
| `rollouts` | 60 |
| 1 iteration 데이터 | 60 × 128 = **7,680 transitions** |
| `learning_epochs` | 4 |
| `mini_batches` | 60 |
| 1 iteration update | 4 × 60 = **240 gradient steps** |
| `trainer.timesteps` | 1,000,000 |
| 총 경험 샘플 | ≈ **128M sample** (env-step 환산) |

### 11.2 학습은 어디서 돌렸을까?

레포에 train 스크립트가 **없음**. 진입점은 eval 뿐 → IsaacLab 표준 trainer로 추정:
```bash
./isaaclab.sh -p source/standalone/workflows/skrl/train.py \
    --task AAURoverEnv-v0 --num_envs 128 --headless
```

증거: `learning/agents/skrl.py`의 `PPO_agent` 정의 + `wandb: True` + `analysis/csv_logs/csv_PPO/` 익스포트 존재.

---

# Part IV — 짧은 Q&A 묶음

## 12. 보상의 개수와 항목은 사용자가 설계하는가?

> ❓ **보상이 몇개인지, 그리고 어떤 항목인지는 사용자가 최초에 설계하는건가?**

**네, 전적으로 설계자의 선택입니다.** 정답이 정해진 게 아님.

- 개수: 1개일 수도, 20개일 수도
- 항목: 도메인 지식 + 시행착오
- 가중치: 튜닝 대상 (RL 실무의 큰 비중)

이 프로젝트가 8개로 정한 건 **navigation/alignment/stability 3그룹 커버**라는 설계 판단. 다른 개발자가 같은 미션을 풀면 5개나 12개일 수도. **"보상 설계(reward design / shaping)"는 RL에서 가장 어려운 부분**이고, 논문이 따로 나올 정도의 주제예요.

## 13. Isaac Sim의 RL은 Isaac Lab으로?

> ❓ **이런 강화학습은 아이작 심 관련해서는 아이작 랩에서 작동되는거고?**

**맞습니다.** 역할 분담:

| 도구 | 역할 |
|------|------|
| **Isaac Sim** | 물리 시뮬레이터 (PhysX, USD, 렌더링) — "무대" |
| **Isaac Lab** | Isaac Sim 위의 RL 학습 인프라 — "RL 도구" |
| **skrl** | RL 알고리즘 라이브러리 (PPO, SAC 등) |

## 14. 내 PC 사양으로 돌아갈까?

> ❓ **그 아이작 랩은 내 PC 사양에서 충분히 돌아갈까?**

[INSTALL.md](INSTALL.md) 기준 사용자 PC: **RTX 5070 Ti / 12GB VRAM**, Ubuntu 22.04, 31GB RAM

| 작업 | 사용자 PC | 비고 |
|------|----------|------|
| `02_eval.py` / `03_eval_ros2.py` 실행 | ✅ 잘 됨 | 학습된 .pt 사용 |
| 처음부터 학습 (`num_envs=128`) | ⚠️ OOM 가능 | 줄여서 시도 |
| 학습 (`num_envs=32~64`, headless) | ✅ 가능 | 시간 더 걸림 |

## 15. 초기 행동은 완전한 난수인가?

> ❓ **최초나 초기의 작동은 완전한 난수 값으로 시작하는거니?**

**거의 맞아요.** 정확히는:
- 정책 네트워크 가중치 = **작은 랜덤값으로 초기화**
- forward pass → 출력 평균 μ ≈ 0 (Tanh 이후라 0 근처)
- Gaussian policy `a ~ N(μ, σ)` → **0 주변 노이즈**
- `log_std=0` 시작 → σ=1 → 큰 노이즈

**결과**: 첫 몇 에피소드는 핸들을 마구 흔들면서 무의미하게 헤맴. **어쩌다 우연히 목표 가까이 가면** 보상을 받고, 그 행동 확률이 조금 올라가는 식.

> 💡 그래서 RL은 "운 좋게 한 번 성공해야 학습이 시작". dense reward를 잘 설계해야 하는 이유 — 목표 도달 없이도 "조금이라도 가까워졌어" 신호로 첫 학습 신호 만들기.

## 16. 보상은 에피소드 끝에 한 번 적용?

> ❓ **에피소드가 끝나는 기준이 있는데, 그 상태에서 보상의 기준들을 적용하여 그 에피소드의 점수를 저장하는거고?**

**아니에요. 보상은 매 step 적용됩니다:**

```
step 1: action → env.step() → reward_1
step 2: action → env.step() → reward_2
...
step 750: action → env.step() → reward_750 + (종료)
```

매 step마다 8개 reward 함수가 호출. 에피소드 점수 = `Σ reward_t`.

다만 **어떤 reward는 특정 시점에만 0이 아닌 값**:
| 종류 | 매 step 발동? | 예 |
|------|:------------:|-----|
| Dense | ✅ 항상 | `distance_to_target` |
| Sparse | ❌ 조건 충족 시만 | `reached_target` (도달 시점), `collision` (충돌 시점) |

sparse reward도 "에피소드 끝에 모아서"가 아니라 **그 사건이 발생한 step에 즉시** 부여.

**왜 중요한가**: 매 step 보상이 있어야 PPO가 **credit assignment**를 할 수 있음. 에피소드 끝 점수 하나만 받으면 750 step 중 어느 행동이 좋았는지 구분 불가 → 학습 거의 불가능. **GAE-λ가 이 일을 함**.

---

# Part V — 클론 프로젝트의 솔직한 평가

> ❓ **이 프로젝트에서 진행한 강화학습이 유의미하다고 생각하니? 학습 목적으로는 괜찮을지 몰라도, 실무적 시야에서는 수준이 낮다는 생각이야.**

**사용자의 평가가 대체로 맞습니다.** 학습용으로는 잘 짜였지만, 연구나 실무 RL 기준으론 "포트폴리오/학생 프로젝트" 수준.

### 약한 이유

1. **인지(perception) 완전 우회** — `distance`, `heading`은 `env.command_manager`가 직접 주는 GT. 실제 화성 로버의 가장 어려운 문제(광물 인식, SLAM)를 sim 내부 cheat로 처리. 학습 정책은 **"좌표 주어지면 가는 navigation policy"** 그 이상이 아님.

2. **Sim2real 고려 전무** — Mass/friction/inertia randomization 없음, motor delay·sensor noise 없음. 실 로봇 deploy 시 **거의 확실히 실패**.

3. **학습 규모 작음**

| 비교 | timesteps |
|------|-----------|
| 이 프로젝트 | 1M |
| ANYmal locomotion | ~100M |
| ANYmal parkour | ~1B |
| OpenAI Dactyl | ~10B |

1M은 **IsaacLab 튜토리얼 수준**.

4. **Task가 RL 관점 trivial** — "go-to-point with obstacle avoidance"는 RL 교과서 첫 번째 navigation 예제. 클래식 알고리즘(RRT*, MPC)으로도 풀림.

5. **Reward 설계의 hacky한 흔적**
```python
# 매직 넘버 다수 — 원리에서 유도된 게 아니라 눈으로 튜닝
oscillation: angular_diff * 3 > 0.05, ^4 페널티  # 왜 3? 왜 4승?
distance:    1/(1 + 0.11 * d²)                    # 왜 0.11?
```

6. **추론 시점의 hack** — [03_eval_ros2.py:246](rover/sim/scripts/03_eval_ros2.py#L246):
```python
actions[:, 0] = torch.clamp(actions[:, 0] * 2.0, -1.0, 1.0)
```
**조향 강제 2배 증폭** — 학습 부족/reward shaping 부족의 빨간 신호.

7. **"듀얼 로버"는 framing뿐** — 같은 정책을 두 번 인스턴스화. Multi-agent RL이나 협동 0.

8. **학습/평가 분리 부족** — 학습 스크립트 없음, 같은 terrain에서 학습/평가, holdout 평가 없음.

9. **ROS2 통합 비본질적** — publish만, subscribe로 정책에 들어오는 정보 0.

### 인정할 부분

1. **통합 작업 자체가 비쌈** — Isaac Sim 5.1 + IsaacLab 2.3 + skrl + ROS2를 한 데 묶음
2. **manager-based env 구조 깔끔**
3. **Ackermann과 RL 분리는 좋은 설계**

### 종합

| 평가 축 | 등급 |
|---------|------|
| Isaac Sim/Lab 학습 예제 | ⭐⭐⭐⭐ |
| 통합 엔지니어링 데모 | ⭐⭐⭐ |
| 학생 졸업작품 산출물 | ⭐⭐⭐⭐ |
| RL 연구 기여 | ⭐ |
| 실무 robotics 시스템 | ⭐ |
| 화성 로버 자율주행 솔루션 | ⭐ (그저 데모) |

**한 줄**: "통합 데모로는 합격, RL 연구나 실제 deployment 관점에선 starter project". 학습 목적으론 **딱 좋은 난이도**.

---

# Part VI — 우리 프로젝트의 방향

> ❓ **나도 외계 행성 탐사 로봇 주제로 Isaac SIM 프로젝트를 진행하려고 해. 시뮬레이터의 핵심은 '현실에서 불가한 작업이나 우주처럼 제한적 환경을 해소하는 것'이라고 판단. 다른 팀은 물류창고 4종 로봇 협업으로 병렬 시뮬을 최대 활용해서 멋있었어. 우리는 어떤 차별점을 줄 수 있을까?**

## 17. skrl이 뭔지부터

**skrl** ([github.com/Toni-SM/skrl](https://github.com/Toni-SM/skrl))은 **PyTorch 기반 RL 알고리즘 라이브러리**. PPO, SAC, TD3, TRPO 등 표준 알고리즘이 깔끔하게 구현돼 있고, **Isaac Lab과 공식 호환**. 비슷한 라이브러리: Stable-Baselines3, RL_games, RSL_RL. IsaacLab + skrl이 NVIDIA 공식 권장 페어 중 하나.

## 18. Isaac Sim의 진짜 강점

물류창고 팀이 멋있던 이유: Isaac Sim이 가장 잘하는 것을 정확히 활용. 차별화 포인트 7개:

| 강점 | 의미 | 누가 잘 활용? |
|------|------|--------------|
| **① 대규모 병렬 시뮬레이션** | 수천 env 동시 (GPU) | 물류창고 팀 ✓ |
| **② Photorealistic 렌더링** | Vision policy 학습용 합성 데이터 | Replicator 팀 |
| **③ Domain randomization** | 한 번에 수천 가지 변형 | sim2real 팀 |
| **④ 정밀 센서 시뮬** | LiDAR, RGBD, IMU, contact | 인지 연구 팀 |
| **⑤ 이종 로봇 통합** | 다른 동역학·센서 한 씬 | 물류창고 팀 ✓ |
| **⑥ USD/Replicator 합성 데이터** | 라벨링 학습 데이터 자동 | 데이터 부트스트랩 |
| **⑦ ROS2 + IsaacLab + RL 통합** | end-to-end 파이프라인 | (클론이 시도) |

클론 프로젝트는 **⑦만 살짝** 사용. 그래서 조악하게 느껴진 거. 물류창고 팀은 ①+⑤. 우리는 **다른 강점을 깊게 활용**해야 차별화.

## 19. 우주 주제 × Isaac Sim 강점 매칭 — 차별화 4가지

| 특성 | 물류창고 | 우주 탐사 |
|------|---------|----------|
| 환경 예측 가능성 | 높음 (정형) | 낮음 (랜덤 지형, 미지) |
| 통신 | 즉시 | **지연 4~24분** (Mars) |
| 중력/물리 | 표준 | **0.16g(달) / 0.38g(화성)** |
| 협업 형태 | 동종 다수 | **이종 소수** |
| 실패 비용 | 회수 가능 | **불가** → 극단적 robust |

### 🅰️ Heterogeneous Space Team (이종 협업)
- Rover + Drone + Lander 협업
- Isaac Sim 강점: ⑤ + ④
- 물류창고와 차별: 본질적으로 다른 동역학(2D vs 3D)·다른 센서·다른 역할
- 트레이드오프: 구현 복잡↑, multi-agent RL 난이도

### 🅱️ Mass Domain Randomization for Sim2Real
- 화성 지형 수천 가지 절차 생성 → 정책이 미지 지형에 robust
- 변동: 중력, 마찰, 암석 분포, 슬로프, 토양 침강, 모터 지연, 센서 노이즈
- 평가: holdout 지형 성공률
- Isaac Sim 강점: ① + ③
- 클론 대비 직접 개선: 하나의 terrain → 일반화 정량 입증
- 트레이드오프: 시각적 화려함은 떨어짐

### 🅲️ Vision-Based Autonomous Science
- 카메라만으로 "이 암석은 흥미로운가?" 판단 + 채취
- Replicator 합성 데이터로 분류기 학습 → RL과 결합
- Isaac Sim 강점: ② + ⑥ + ④
- 클론 대비 직접 개선: GT cheat → 진짜 perception
- 트레이드오프: 컴퓨터비전 작업↑

### 🅳️ Communication-Delayed Supervised Autonomy
- 지구-화성 통신 지연 명시적 반영
- 운영자: 고수준 명령만, 로봇: 자율 위험 회피
- Isaac Sim 강점: ⑦ + ROS2 본질적 사용
- 강점: "시뮬레이터의 필요성" 본질 최강 표현
- 트레이드오프: RL 비중 줄어듦, 시연 페이스 느림

**처음 추천**: 🅱️ + 🅰️ 조합 (구체화는 Part VII부터)

---

# Part VII — 제약 반영 첫 계획 (v1)

> ❓ **팀 규모 5명, 전원 클로드 코드 사용. RL이 핵심은 아님, 챌린지적 요소. 5060(8GB), 5070Ti(12GB, 본인), 5080×3(16GB). 다음주 목요일 발표, 수요일 오전까지 완성. 평일 9:30~22, 주말 10~18 가용. 70% 가중치 적용. 클론은 재사용 가능 판단.**

## 가용 시간 산정

```
화 5/19 ─┐
수 5/20  │
목 5/21  │  평일 6일 × 12.5h = 75h
금 5/22  │
토 5/23  ├─ 주말 2일 × 8h = 16h
일 5/24  │
월 5/25  │
화 5/26 ─┘
수 5/27 오전 ≈ 3h
                = 1인당 총 94h (raw)
```
**70% 가중치 → 1인당 ≈ 66h, 팀 5명 → ≈ 330 person-hours**

## 첫 추천 (v1)

> *"절차적으로 생성된 화성 지형에서, 클론의 6륜 로버 + 신규 정찰 드론이 협업하여 광물을 수집한다. 학습 안 한 지형에서도 동작함을 보인다."*

물류창고 팀과의 차별점:
| 그들 | 우리 |
|------|------|
| 동일 지면, 4종 로봇 | **수십 종 지형**, 2종 로봇 |
| 정형 환경 협업 | **이종 동역학(2D+3D) 협업** |
| 시각적 다양성 | **정량 일반화 평가** + 시각적 |

5인 트랙 분배 (v1):

| 역할 | VRAM | 산출물 |
|------|:----:|--------|
| A. Procedural Terrain Pipeline | 16GB | N개 지형 USD 자동 생성 |
| B. Drone Integration | 16GB | Isaac Sim 드론 + RGBD 검출 |
| C. RL Re-training & Eval | 16GB | rover PPO fine-tune, holdout success |
| D. ROS2 Orchestration & UI | 12GB | drone↔rover 통신, 대시보드 |
| E. Eval, Viz, Presentation | 8GB | 차트, 영상, 발표자료 |

**v1 문제**: 드론 통합과 화성 물성 둘 다 unknown → 동시 실패 리스크. 다음 단계에서 갱신.

---

# Part VIII — 화성 물성 미적용 확인 → 계획 갱신 (v2)

> ❓ **추가로 이 코드에서 Sim2Real 과정에서 화성의 물성치를 사용했는지 궁금함.**

## 결론: ❌ **사용하지 않았어요. "화성"은 비주얼만 화성**

### 1. 중력은 지구 표준값 (PhysX 기본)

[rover_env_cfg.py:234-252](rover/sim/rover_envs/envs/navigation/rover_env_cfg.py#L234-L252):
```python
sim: SimCfg = SimCfg(
    physx=PhysxCfg(...)
)
# gravity 파라미터 명시 안 됨 → IsaacLab 기본값 (0, 0, -9.81) 지구
```

### 2. 마찰 계수는 PhysX 디폴트
- `RigidBodyMaterialCfg`, `PhysicsMaterialCfg` **어디에도 없음** (0건)
- 로버 휠/지형 physics material 미할당 → PhysX 기본 μ≈0.5

### 3. 지형은 비주얼 자산만 화성
- USD 파일은 3D 메쉬 + 텍스처만
- 메쉬에 화성 토양 특유 물리속성 박혀있지 않음
- 텍스처 이름이 "Soil_Rocky"라 시각적으로만 화성

### 4. 빠진 화성 요소

| Mars-specific physics | 이 프로젝트 |
|----------------------|-------------|
| 중력 3.71 m/s² | ❌ 9.81 |
| Regolith 마찰 | ❌ PhysX 기본 |
| Soft soil sinkage | ❌ rigid contact만 |
| 먼지/dust 입자 | ❌ |
| 일조량/그림자 | ❌ |
| 온도 효과 | ❌ |

### 5. PhysxCfg의 튜닝 값들은?
`friction_correlation_distance=0.025` 등은 **PhysX 솔버 안정성 파라미터**. 화성 물성과 무관.

## 의미

이 시뮬은 화성을 **보이는 것처럼만** 모델링. **물리적으로는 지구 평지 위에 돌 흩뿌린 환경**과 동일. 정책을 화성으로 옮기면 동작 완전 달라짐.

## v1 계획을 v2로 갱신

**드론 트랙 드롭, 화성 물성 트랙 신설** 권장:

### 갱신 컨셉
> *"화성의 실제 중력·마찰·토양 물성을 적용한 절차생성 지형 1000개에서 학습된 화성 로버. 학습 안 한 지형 + 학습 안 한 물성 조건에서도 동작함을 정량 입증."*

### 갱신된 트랙 구성

| 트랙 | 담당 | VRAM | 산출물 |
|------|------|:----:|--------|
| A. Procedural Mars Terrain | 5080 #1 | 16GB | 절차생성 USD 지형 + randomization |
| **F. Mars Physics Module** ⭐신규 | 5080 #2 | 16GB | gravity 3.72, friction material, 휠 침강 |
| C. RL Fine-tune & Mars Eval | 5080 #3 | 16GB | retrain. Earth vs Mars 비교표 |
| D. ROS2 + Mission Control UI | 5070 Ti | 12GB | Earth/Mars 모드 토글 |
| E. Eval Charts + Presentation | 5060 | 8GB | 비교 차트, 영상, 발표 |

### 화성 물성 Tier 전략

```
Tier 1 (Day 1-2, MUST) 🟢
  ✓ gravity = (0, 0, -3.72)
  ✓ RigidBodyMaterialCfg(static_friction=0.4, dynamic_friction=0.35)
  → 학습 재개. "지구 vs 화성" 비교 가능

Tier 2 (Day 3-4, SHOULD) 🟡
  ✓ 휠 침강 근사: contact rest_offset + 휠별 마찰 randomize
  ✓ 지형 영역별 friction zone (모래/암반/자갈)

Tier 3 (Day 5+, COULD) 🔵
  ✓ Dust particle (시각용)
  ✓ Solar lighting 시뮬

Tier 4 (NOT IN SCOPE) 🔴
  ✗ Full PhysX deformable soil (FEM)
```

---

# Part IX — 지형 random성 부분 적용 확인 → 최종 컨셉 (v3)

> ❓ **클론한 팀은 지형 생성에는 Random성이 없던건가? 체크해주고, Random성을 우선 반영한다면 같이 들어가면 좋을 스코프/타겟도 추천해주라.**

## 지형 random성 점검 결과

**부분적으로 적용돼 있음.**

| 항목 | random 여부 | 위치 |
|------|:----------:|------|
| 로버 스폰 위치 | ✅ 2000개 사전 계산된 safe location 중 무작위 | [randomizations.py:11](rover/sim/rover_envs/envs/navigation/mdp/randomizations.py#L11) |
| 로버 스폰 방향 (yaw) | ✅ 0~2π 균등 | randomizations.py:31 |
| 목표(광물) 위치 | ✅ 반경 9m 원 위 무작위 + safety mask | [terrain_importer.py:139](rover/sim/rover_envs/envs/navigation/utils/terrains/terrain_importer.py#L139) |
| 관측 노이즈 | ✅ enable_corruption=True | rover_env_cfg.py:113 |
| **지형 메쉬 자체** | ❌ **단일 고정** terrain_only.usd | mars_terrains.py |
| **암석 배치** | ❌ **단일 고정** rocks_merged.usd | mars_terrains.py |
| 물리 (중력/마찰/질량) | ❌ 전혀 없음 | — |
| 조명/그림자 | ❌ 정적 | rover_env_cfg.py:39 |

**핵심**: "한 지형 안에서의 시작점·목표·방향"만 randomize. **지형 형상 자체는 항상 같은 USD 한 장**. 진정한 procedural generation은 0건 → **지형 randomization은 명백한 차별화 포인트**.

## 지형 random성과 함께 갈 보조 스코프 추천

| 보조 스코프 | 시너지 | 8일 내 난이도 |
|------------|--------|:---:|
| **🔴 화성 물성** | "다양한 지형 × 진짜 화성 물리" 임팩트 ×4. 둘 다 클론 결함 보완 | 🟢 쉬움 |
| **🟠 Holdout terrain 평가** | "보지 못한 지형 X개 중 Y개 성공" 한 줄이 발표 핵심 | 🟢 쉬움 |
| **🟡 Difficulty curriculum** | 절차생성 파라미터로 난이도 자동 → 학습 안정성↑ | 🟡 중 |
| 🟢 Sensor noise randomization | sim2real 스토리 강화 | 🟡 중 |
| 🔵 동적 조명 | sol 표현, 시각 wow | 🔵 시간 남으면 |

**추천 묶음**: 🔴 + 🟠 (필수) + 🟡 (권장). 🟢🔵은 시간 남을 때.

---

## ✨ 최종 컨셉 (v3)

> **"클론의 단일 화성 지형을 절차 생성된 다지형 + 진짜 화성 물성으로 확장. Difficulty curriculum으로 학습 안정성 확보. 학습에 보지 못한 지형 N개에서 정량 평가로 일반화 입증."**

### 트랙 구성 (최종)

| 트랙 | 담당 | VRAM | 산출물 핵심 KPI |
|------|------|:----:|----------------|
| **A. Procedural Mars Terrain** | 5080 #1 | 16GB | 절차생성 USD N≥50개, 난이도 파라미터화 |
| **F. Mars Physics Module** | 5080 #2 | 16GB | Tier 1+2 완료, friction zone 동작 |
| **C. RL Fine-tune w/ Curriculum** | 5080 #3 | 16GB | easy→hard 자동 진행, holdout 성공률 측정 |
| **D. ROS2 + Mission Control UI** | 5070 Ti (사용자) | 12GB | Earth/Mars 토글, 다지형 갤러리 view |
| **E. Eval Charts + Presentation** | 5060 | 8GB | 학습/holdout 비교 차트, RTX grid 영상, 발표 |

### 8일 일정 (최종)

```
┌─ Phase 1: Setup & Mars Physics Spike (Tue-Wed) ──────────────────┐
│ 화 5/19 ✦ 킥오프, 역할/git 합의                                  │
│        ✦ F: gravity·friction 즉시 적용 → eval 1회                 │
│        ✦ A: 클론 terrain 분석 → 절차생성 파라미터 후보            │
│ 수 5/20 ✦ F: Tier 1 완료 (지구 vs 화성 eval 차트)                 │
│        ✦ A: 5개 시드로 terrain 변형 PoC                          │
│        ✦ C: fine-tune 파이프라인 셋업, 화성 중력 학습 시작        │
│        ✦ D: UI 와이어프레임, Earth/Mars 토글 설계                 │
│        ✦ E: baseline eval (지구·단일지형) 차트                    │
│                                                                  │
│ ⚠️ 수 EOD 게이트: "Tier 1 적용 + eval 차트 1장"                   │
│    실패 시 Tier 2 포기                                            │
└──────────────────────────────────────────────────────────────────┘

┌─ Phase 2: Core Build (Thu-Sun) ──────────────────────────────────┐
│ 목 5/21 ✦ A: 50개 지형 batch 생성 시작                            │
│        ✦ F: Tier 2 (영역별 friction zone)                         │
│        ✦ C: 50지형 × 화성 물성 학습 (overnight)                   │
│        ✦ C: curriculum 1차 구현 (easy 시작)                       │
│        ✦ D: ROS2 토픽 확장, dual-mode live switch                 │
│ 금 5/22 ✦ A: 50→200 지형 확장, train/holdout split                │
│        ✦ F: Tier 2 검증, Tier 3 시작 (dust particle)              │
│        ✦ C: 학습 결과 분석, hyperparameter 조정                    │
│        ✦ E: Earth vs Mars 비교 차트 첫 버전                        │
│ 토 5/23 ✦ 전원: 통합 점검. End-to-end 시연 1차                     │
│        ✦ C: holdout 평가 시작                                     │
│ 일 5/24 ✦ ★ Demo dry-run #1                                       │
│        ✦ 시나리오: Earth → Mars 전환, 다지형 grid view             │
│                                                                  │
│ ⚠️ 일 EOD 게이트: "데모 1회 성공"                                 │
│    실패 시 신규 기능 동결                                         │
└──────────────────────────────────────────────────────────────────┘

┌─ Phase 3: Harden & Demo Prep (Mon-Wed AM) ───────────────────────┐
│ 월 5/25 ✦ 버그/edge case                                          │
│        ✦ E: 최종 차트 + 영상 컷                                    │
│        ✦ Tier 3 적용 여부 최종 결정                                │
│ 화 5/26 ✦ ★ Demo dry-run #2 (전체 리허설)                          │
│        ✦ 발표자료 완성, Q&A 준비                                  │
│        ✦ 백업 시나리오 2개 영상 녹화                              │
│ 수 5/27 AM ✦ 최종 점검, 라이브 환경 안정성 체크                    │
│ 목 5/28    ✦ 발표 🎤                                              │
└──────────────────────────────────────────────────────────────────┘
```

### 작업량 견적 (최종)

| 트랙 | person-hours |
|------|:---:|
| A. Procedural Terrain | 55h |
| F. Mars Physics | 45h |
| C. RL + Curriculum | 65h |
| D. ROS2 + UI | 55h |
| E. Eval + Presentation | 60h |
| **합계** | **280h** |
| 가용 (5명 × 66h) | **330h** |
| **버퍼** | **50h ≈ 15%** |

### 리스크 (최종)

| 리스크 | 영향 | 대응 |
|--------|:----:|------|
| Tier 2 (friction zone) PhysX 한계 | 🟡 | Tier 1만으로도 차별점 충분 |
| Curriculum 자동 진행 알고리즘 미수렴 | 🟡 | 수동 stage 진행으로 fallback |
| 50지형 batch 시간 | 🟡 | 5080 16GB × 3대 분산 |
| 학습 수렴 안 함 | 🔴 | 클론 best_agent_ppo.pt를 init으로 fine-tune (from-scratch 회피) |
| 5060 8GB 학습 부담 | ✅ | 학습 X, viz/문서만 |
| 5명 git 충돌 | 🟢 | feature branch + 일 1회 통합 |

### 발표 임팩트 헤드라인 3개 (미리 박아두기)

1. **"화성을 보이는 것처럼만 만든 시뮬과, 진짜 화성처럼 만든 시뮬"**
   - 좌: 클론 (지구 9.81) / 우: 우리 (화성 3.72)
   - 같은 정책 → 좌 성공, 우 휠스핀/슬립 → "물성이 시뮬의 본질"
2. **"50가지 절차생성 화성 지형 중 학습 안 한 N개에서 X% 성공"**
   - 정량 그래프 + RTX 멀티-env grid view 영상
3. **"Earth → Mars 모드 라이브 전환 데모"**
   - 1 클릭 토글, 즉시 성능 차이 시각화

---

# Part X — 즉시 합의가 필요한 결정 항목

오늘 EOD까지 팀 합의:

1. ✅/❌ **최종 컨셉 (v3 — 절차생성 + 화성 물성 + curriculum + holdout 평가)** 확정?
2. ✅/❌ **드론 트랙은 드롭** (스토리 단순화, 리스크 감소)
3. ✅/❌ **Tier 1+2까지 목표, Tier 3는 시간 남으면**
4. ✅/❌ **50지형 우선, 시간 남으면 200까지 확장**
5. ✅/❌ **학습은 클론 best_agent_ppo.pt에서 fine-tune** (from-scratch 회피)
6. ✅/❌ **트랙 담당** (VRAM 기준 배분 — Part IX 트랙 표)

결정되면 별도 산출물:
- 화요일 킥오프 자료 (역할표, git 브랜치 전략, 첫 spike 체크리스트)
- 각 트랙의 PR 템플릿
- 수요일 EOD 게이트 체크리스트

---

## 📚 부록: 참고 파일

- 클론 프로젝트 강화 README: [`README.enhanced.md`](README.enhanced.md)
- 클론 프로젝트 원본 README: [`readme.md`](readme.md)
- 설치 가이드: [`INSTALL.md`](INSTALL.md)
- 핵심 코드 진입점:
  - 환경: [rover_env_cfg.py](rover/sim/rover_envs/envs/navigation/rover_env_cfg.py)
  - 보상: [rewards.py](rover/sim/rover_envs/envs/navigation/mdp/rewards.py)
  - Ackermann: [ackermann_actions.py](rover/sim/rover_envs/mdp/actions/ackermann_actions.py)
  - 정책 네트워크: [models.py](rover/sim/rover_envs/envs/navigation/learning/skrl/models.py)
  - PPO 설정: [rover_ppo.yaml](rover/sim/rover_envs/envs/navigation/learning/skrl/configs/rover_ppo.yaml)
  - 미션 루프: [03_eval_ros2.py](rover/sim/scripts/03_eval_ros2.py)

---

# Part XI — 최종 확정 (Latest, 2026-05-19 기준)

> 이 섹션은 Part I~X 의 학습/탐색 과정을 거쳐 도달한 **최종 결정사항**입니다.
> 작업 진행 시 본 섹션을 1차 참조 — Part I~X는 historical context.

## XI-1. 최종 컨셉

> **"화성 광물 자율 수집 로버 시뮬레이션 — 절차생성 화성 환경 + Vision 기반 자율 탐사 + TRN 위치 추정 + M0609 매니퓰레이션"**

### 클론 대비 차별화
1. ❌ GT 좌표 cheat → ✅ Vision detection (HSV 색기반)
2. ❌ 단일 고정 지형 → ✅ 절차생성 N개 지형 (Perlin noise)
3. ❌ 시각만 화성 → ✅ 진짜 Mars 물리 (gravity 3.72, friction zone)
4. ❌ 매니퓰레이터 없음 → ✅ M0609 pick & place (Tier 1.5 scripted)
5. ❌ GT pose 사용 → ✅ TRN + Multi-sensor Fusion (Perseverance 기법)

### 발표 헤드라인 3개
1. "Procedural Mars terrain × 30개 holdout에서 X% 성공"
2. "GPS 없는 화성에서 TRN으로 위치 추정 (Perseverance 기법 시뮬)"
3. "Vision-driven 자율 탐사 + M0609 매니퓰레이션 (full autonomy stack)"

## XI-2. 5인 트랙 (최종)

| # | 트랙 | 담당 | GPU | 시간 | BRIEF |
|---|------|------|:---:|:---:|------|
| **T1** | 🗺️ Environment | 시니어 | 5060 (8GB) | 55-65h | [T1_BRIEF.md](tracks/T1_BRIEF.md) |
| **T2** | 👁️🦾 Perception + M0609 | 주니어 (스왑됨) | 5080 (16GB) | 70h | [T2_BRIEF.md](tracks/T2_BRIEF.md) |
| **T3** | 🚗 Driving (Mission Brain) | 시니어 (스왑됨) | 5080 (16GB) | 60h | [T3_BRIEF.md](tracks/T3_BRIEF.md) |
| **T4** | 🔌 Integration + PM | 사용자 본인 | 5070 Ti (12GB) | 75h | [T4_BRIEF.md](tracks/T4_BRIEF.md) |
| **T5** | 📍 Localization + Infra | 기존 통합 담당자 | 5080 (16GB) | 70h | [T5_BRIEF.md](tracks/T5_BRIEF.md) |

**핵심 변경 사항**:
- T2 ↔ T3 사람 스왑 (스킬-복잡도 매칭)
- 기존 통합 담당자 → T5 이동 (사용자가 통합 owner)
- Localization은 Mock → **TRN 기반 진짜 multi-sensor fusion**

## XI-3. 인터페이스 (Day 1 lock)

| ID | 토픽 / 파일 | Producer→Consumer | 종류 |
|:--:|------------|:-----------------:|:----:|
| **I1** | terrain_*.usd + meta.json | T1 → 모두 | 파일 |
| **I2** | `/perception/detections` | T2 → T3, T4 | ROS2 @10Hz |
| **I3** | `/mission/pick_request` | T3 → T2 | ROS2 event |
| **I4** | `/mission/pick_response` | T2 → T3 | ROS2 event |
| **I5** | `/rover/estimated_pose` | T5 → T3, T4 | ROS2 @30Hz |

상세: [interfaces/INTERFACE_CONTRACTS.md](interfaces/INTERFACE_CONTRACTS.md)
UI 관련 (I6~I10) deferred: [interfaces/deferred_interfaces.md](interfaces/deferred_interfaces.md)

## XI-4. 핵심 기술 결정

| 결정 | 내용 | 위치 |
|------|------|------|
| Localization | **TRN + EKF** (IMU/Wheel/Sun + heightmap correlation) | T5 |
| Vision | 색기반 HSV (CNN 아님) | T2 |
| M0609 | Tier 1.5 (scripted trajectory + 텔레포트) | T2 |
| Basecamp | Tier 1 (시각만, 충돌 X) | T1 |
| 카메라 | 마스트 (0, -0.2, 0.7), 30° down, 640×480 | T2 |
| Mars Physics | Tier 1 (gravity 3.72) by T1, Tier 2 (friction zones) by T5 | T1+T5 |
| Coverage | Greedy frontier 미니맵 25×25 (cell 2m) | T3 |
| RL | 클론 PPO 정책 그대로 재사용 (재학습 X) | T3 |

## XI-5. 8일 일정 (최종)

```
Day 1 (화 5/19) ★ Kickoff
  - 09:30 standup
  - 90분 회의 (KICKOFF_AGENDA.md)
  - 5개 인터페이스 사인
  - 각 트랙 Day 1 작업 시작
  ⚠️ EOD: 인터페이스 lock + T2 M0609 spike 결과

Day 2 (수 5/20)
  ⚠️ Gate: 각 트랙 hello-world 동작

Day 3 (목 5/21)
  - T3 A* + 장애물
  - T5 TRN 단독 검증
  - T2 M0609 통합

Day 4 (금 5/22) ★ First Integration
  - T3 + T5 통합 (pose source swap)
  - T3 + T2 통합 (pick_request/response)

Day 5 (토 5/23)
  ⚠️ Gate: End-to-end 미션 1회 성공

Day 6 (일 5/24) ★ Demo Stable
  - demo-stable-v1 git tag
  - 신규 기능 동결 시작

Day 7 (월 5/25)
  - 발표 자료
  - 데모 시나리오 A/B
  - 백업 영상

Day 8 (화 5/26)
  - Dry-run #2
  - 최종 점검

Day 9 정오 (수 5/27)
  - Final freeze

Day 10 (목 5/28) 🎤
  - 발표
```

## XI-6. 산출물 위치 (전체 인덱스)

```
프로젝트 루트/
├── readme.md                          # 클론 원본
├── README.enhanced.md                 # 클론 강화 README
├── INSTALL.md                         # 설치 가이드
├── STUDY_AND_PLAN.md                  # 본 문서 (학습 노트 + 본 Part XI)
│
├── system_flowchart.svg               # 런타임 데이터 흐름 (간단)
├── project_overview_flowchart.svg     # 팀 회의용 미션 시나리오
├── rover_steering_comparison.svg      # AAU vs NASA 조향 비교
├── system_architecture_full.svg       # 풀 아키텍처 (인터페이스 포함)
│
├── tracks/                            # 트랙별 onboarding
│   ├── T1_BRIEF.md, T1_CLAUDE.md
│   ├── T2_BRIEF.md, T2_CLAUDE.md
│   ├── T3_BRIEF.md, T3_CLAUDE.md
│   ├── T4_BRIEF.md, T4_CLAUDE.md     # PM(본인)용
│   └── T5_BRIEF.md, T5_CLAUDE.md
│
├── interfaces/                        # 인터페이스 명세
│   ├── INTERFACE_CONTRACTS.md         # 5개 인터페이스
│   ├── deferred_interfaces.md         # I6~I10 (Day 4+)
│   ├── terrain_meta_schema.json       # I1 JSON Schema
│   ├── example_terrain_meta.json      # 예시
│   └── msg/                           # ROS2 메시지 정의
│       ├── Detection.msg
│       ├── DetectionArray.msg
│       ├── PickRequest.msg
│       └── PickResponse.msg
│
└── pm_tools/                          # PM 운영 도구
    ├── README.md                      # 사용 가이드
    ├── KICKOFF_AGENDA.md              # Day 1 회의
    ├── DAILY_STATUS.md                # 매일 09:30
    ├── RISK_REGISTER.md               # 주 2회
    ├── DECISIONS.md                   # 이벤트 기반
    └── run_dist.sh                    # 매일 18:00
```

## XI-7. 다음 행동

PM(사용자)이 지금 해야 할 것:

```
1. ★ 킥오프 회의 진행
   - pm_tools/KICKOFF_AGENDA.md 따라
   - 5명 모이는 자리 확보
   
2. 각 트랙 담당자에게 사전 공유
   - 본인 트랙의 BRIEF.md 미리 읽기 요청 (회의 1시간 전)
   
3. T5 담당자와 1:1
   - 시간 가용성 확인
   - TRN scope 합의 (Day 3 EOD까지 TRN 단독 검증 가능?)
   
4. Day 1 후 작업
   - run_dist.sh 매일 실행
   - DAILY_STATUS 매일 09:30 갱신
   - RISK_REGISTER 주 2회 검토
```

## XI-8. 마지막 한 마디

8일 안에 만들 것은 **단 하나**:
> **"발표 데모가 한 번이라도 성공적으로 돌아가는 시스템"**

다른 모든 것 (정량 평가, 가치점수, TRN 정확도, 등)은 이 위에 얹는 보너스.

발표 데모를 위해 **인터페이스 lock + 매일 DIST + Day 6 demo-stable-v1 tag** 3가지가 핵심.

화이팅 🚀
