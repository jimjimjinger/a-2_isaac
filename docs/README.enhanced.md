# 🚀 Rokey6 — Mars Mineral Collection Rover

**Isaac Sim 기반, PPO 강화학습 + Ackermann 조향으로 자율주행하는 6륜 화성 탐사 로버 시뮬레이션**

> 두 대의 로버가 화성 지형에서 각자 광물 5개를 수집하고 베이스캠프로 복귀하는 미션을 자동 반복합니다.
> 주행 의사결정은 PPO로 학습된 정책 네트워크가, 휠 제어는 Ackermann 모델이 담당하며,
> 미션 상태는 ROS2 토픽으로 실시간 모니터링됩니다.

![Monitoring Dashboard](images/monitoring_capture.png)

---

## 📑 Table of Contents

1. [What this project does](#1-what-this-project-does)
2. [System architecture](#2-system-architecture)
3. [How the rover drives itself (자율주행 파이프라인)](#3-how-the-rover-drives-itself)
4. [Reinforcement learning details](#4-reinforcement-learning-details)
5. [Repository structure](#5-repository-structure)
6. [Environment & hardware](#6-environment--hardware)
7. [Installation](#7-installation)
8. [How to run](#8-how-to-run)
9. [Troubleshooting](#9-troubleshooting)

---

## 1. What this project does

### 핵심 한 줄
**"학습된 PPO 정책이 (관측 → [조향, 속도])을 출력 → Ackermann 변환기가 6륜 휠 명령으로 풀이 → Isaac Sim PhysX가 물리 진행 → ROS2로 상태 퍼블리시"** 한 사이클의 반복.

### 미션 시나리오
- 두 로버(`Robot0`, `Robot1`)가 화성 지형에 스폰됨
- 각자 무작위 위치의 광물 1개를 목표로 받음 → 도달하면 다음 광물 리샘플
- 광물 5개를 모두 수집하면 베이스캠프 좌표로 목표를 강제 전환 → 복귀
- 베이스캠프 도달 시 다음 라운드 시작 (즉시 새 위치로 텔레포트)
- 충돌·전복으로 에피소드 종료 시 자동 재배치 후 미션 재개

### 자율(autonomy)의 범위
| 자율인 부분 | 자동화되지 않은 부분 (= 추상화됨) |
|-------------|----------------------------------|
| 목표가 주어졌을 때 그곳까지 가는 주행 | 광물/베이스캠프의 **인지** (좌표는 GT로 주어짐) |
| 지형 굴곡과 돌 회피 (heightmap 사용) | 카메라 기반 객체 탐지·SLAM |
| 광물 수집/복귀 사이클 관리 | 미션 계획 (광물 개수, 순서 등은 하드코딩) |

> 즉 이 프로젝트의 자율주행은 **"navigation-only end-to-end RL"** 이며,
> SLAM·perception은 시뮬레이션 ground truth로 단순화돼 있습니다.

---

## 2. System architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                    Isaac Sim 5.1 + IsaacLab 2.3                      │
│                                                                      │
│   ┌─── Scene ─────────────────────────────────────────────────────┐  │
│   │   • Mars terrain (USD, 화성 지형 + 돌 mesh)                   │  │
│   │   • AAU 6-wheel rover (4 steer joints + 6 drive joints)        │  │
│   │   • Dome/Sphere lights, basecamp marker (USD)                  │  │
│   │   • Sensors: RayCaster(5×5m, 0.05 grid), ContactSensor          │  │
│   └────────────────────────────────────────────────────────────────┘  │
│                              ▲   │                                    │
│            steer/drive joint │   │ observations                       │
│                  targets     │   ▼                                    │
│   ┌──────────────────────────┴──────────────────────────────────────┐ │
│   │                  ManagerBasedRLEnv (gym)                          │ │
│   │   - ObservationManager: 5 + 10000 = 10005 dim                     │ │
│   │   - ActionManager     : Ackermann(2D) → 6 wheels                  │ │
│   │   - RewardManager     : 8 reward terms                            │ │
│   │   - TerminationManager: time_out / is_success / collision         │ │
│   │   - CommandManager    : target_pose (광물/베이스 위치)            │ │
│   └──────────────────────────┬──────────────────────────────────────┘  │
└──────────────────────────────┼─────────────────────────────────────────┘
                               │
                ┌──────────────┴────────────────┐
                │  skrl PPO Agent (GaussianPolicyConv)│
                │  loaded from best_agent_ppo.pt       │
                └──────────────┬────────────────┘
                               │  publishes via rclpy
        ┌──────────────────────┴────────────────────────┐
        ▼                                               ▼
  /robot{i}/pose, /mission/status, /robot{i}/image_raw
        │                                               │
        ▼                                               ▼
  mission_viz_node.py  ──▶  rviz2          mission_monitor.py
  (Marker text/arrow)        (3D view)     (terminal dashboard, CSV log)
                                                       │
                                                       ▼
                                        analysis/logs/mission_log_*.csv
```

### 3개의 프로세스가 동시에 떠 있어야 함
1. **Isaac Sim**: `run_ros2.sh`가 띄우는 메인 시뮬레이터 (PPO 추론 + ROS pub)
2. **mission_viz_node + rviz2**: 시각화 (run_ros2.sh가 함께 띄움)
3. **mission_monitor**: 별도 터미널에서 사용자가 실행하는 텍스트 대시보드

---

## 3. How the rover drives itself

### 3.1 한 step 안에서 일어나는 일

```
[Sim state] ──┐
              ▼
        ┌─ Observation (10005 dim) ─────────────────────┐
        │  • last_action               [2]              │
        │  • distance_to_target        [1]   ← scale 0.11│
        │  • heading_angle             [1]   ← scale 1/π│
        │  • angle_diff (heading err)  [1]   ← scale 1/π│
        │  • height_scan (5×5m grid)   [10000] = 100×100│
        └──────────────────┬────────────────────────────┘
                           ▼
        ┌─ PPO policy network ───────────────────────────┐
        │  ConvHeightmapEncoder (8→16→32→64) → MLP[80,60]│
        │  proprio(5) ⊕ conv_out  →  MLP[256,160,128]    │
        │  → Linear(2) → Tanh → μ ∈ [-1,1]²              │
        │  + nn.Parameter log_std → Gaussian(μ,σ)        │
        │  eval: sample = mean μ  (no exploration noise) │
        └──────────────────┬────────────────────────────┘
                           │  action = [steer_norm, throttle_norm]
                           ▼
        ┌─ Ackermann transform ──────────────────────────┐
        │  lin_vel, ang_vel = action × scale             │
        │  R = |v|/|ω|                                   │
        │  if R < 0.8·d_mw:                              │
        │      point-turn (좌·우 휠 반대방향, ±π/4)       │
        │  else:                                         │
        │      per-wheel radius r_i 계산                  │
        │      v_i  = r_i · ω   (안쪽/바깥 차등)          │
        │      θ_i  = atan2(L/2 ± offset, r_i)            │
        └──────────────────┬────────────────────────────┘
                           ▼
        4 steer joint positions + 6 drive joint velocities
                           ▼
                Isaac Sim PhysX (dt=1/30 × decimation 6)
                           │
                           └────► 다음 step의 [Sim state]
```

### 3.2 ⚙️ Ackermann이 왜 필요한가?

PPO가 출력하는 건 추상적인 `[조향 의도, 속도 의도]` 2차원입니다. 하지만 실제 로버는 **6개의 휠과 4개의 조향 관절**을 가진 차량이라서, 한 차원의 "조향 의도"를 각 휠의 적절한 회전반경/각도로 분배해야 합니다. Ackermann 모델은 이 변환의 고전적 해법으로, **안쪽 휠은 더 작은 반경·낮은 속도, 바깥 휠은 큰 반경·높은 속도**로 자동 분배합니다.

조향 반경이 너무 작으면(`< 0.8·d_mw`) **point-turn 모드**로 자동 전환되어 제자리에서 회전합니다.

### 3.3 미션 자동화 (run_mission loop)

`scripts/03_eval_ros2.py::run_mission()` 의 핵심 상태 머신:

```python
phase = "collect"   # 또는 "return"
while sim_running:
    actions = agent.act(obs)
    env.step(actions)

    if phase == "collect":
        if dist_to_mineral < 0.5 and abs(angle) < 0.2:
            collected += 1
            if collected >= 5:
                phase = "return"
                set_command_to_basecamp(...)   # 목표 좌표를 베이스로 교체
            else:
                resample_target(...)            # 다음 광물 리샘플

    elif phase == "return":
        set_command_to_basecamp(...)            # 매 스텝 베이스 방향 재주입
        if dist_to_base < 2.0:
            round += 1
            teleport_to_basecamp(...)           # 새 라운드 시작
            phase = "collect"

    if terminated or truncated:
        pending_teleport.add(i)                 # 충돌/타임아웃 → 재배치
```

**핵심 트릭**: Isaac Lab의 `command_manager`는 매 `env.step()` 내부에서 명령을 덮어쓰기 때문에, "베이스캠프로 가라"는 목표를 유지하려면 [command_utils.py::set_command_to_basecamp](rover/sim/mission/command_utils.py)를 **매 스텝 호출**하고 resample 타이머도 차단해야 합니다.

---

## 4. Reinforcement learning details

### 4.1 MDP 정의

| 요소 | 정의 |
|------|------|
| **Observation** | 10005차원 (proprio 5 + height_scan 10000) |
| **Action** | `Box([-1,-1], [1,1])` — `[steer_norm, throttle_norm]` |
| **Reward** | 아래 8개 가중합 |
| **Termination** | `time_limit (150s)` / `is_success (목표 0.2m 이내)` / `collision (contact force > 0.001N)` |
| **Episode length** | `episode_length_s=150`, `dt=1/30`, `decimation=6` → ≈ 750 steps |

### 4.2 ⭐ 8개 Reward 항 (RewardsCfg)

[rover_env_cfg.py:120-160](rover/sim/rover_envs/envs/navigation/rover_env_cfg.py#L120-L160) 와 [rewards.py](rover/sim/rover_envs/envs/navigation/mdp/rewards.py) 에서 정의됨:

| # | Name | Weight | Category | Formula / Trigger |
|---|------|:------:|----------|-------------------|
| 1 | `distance_to_target` | **+5.0** | Navigation | `1/(1 + 0.11·d²)` per step — 가까울수록 큰 보상 |
| 2 | `reached_target` | **+5.0** | Navigation | `d<0.18m & |θ|<0.1` 충족 시, 남은 시간 비례 보상 |
| 3 | `oscillation` | **-0.05** | Stability | 직전 액션과 차이의 제곱 — 떨리는 주행 페널티 |
| 4 | `angle_to_target` | **-1.5** | Alignment | `|θ| > 2.0 rad` (≈115°) 일 때 절댓값 페널티 |
| 5 | `heading_soft_contraint` | **-0.5** | Alignment | `throttle < 0` (후진) → 페널티 |
| 6 | `collision` | **-3.0** | Stability | contact_sensor force > 1N → -1.0 |
| 7 | `far_from_target` | **-2.0** | Navigation | `d > target_distance + 3m` 초과 시 -1.0 |
| 8 | `angle_to_goal_reward` | **+5.0** | Alignment | `1/(1+d) · 1/(1+|θ|)` — 가깝고 정렬되면 큼 |

**그룹별 의도**:
- **Navigation** (1·2·7): "다가가라 / 도달해라 / 너무 멀어지지 마라"
- **Alignment** (4·5·8): "목표를 바라보고 직진해라 / 후진 금지"
- **Stability** (3·6): "떨지 마라 / 부딪히지 마라"

### 4.3 정책 / Value 네트워크 (skrl, `GaussianPolicyConv`)

- **공유 인코더**: `ConvHeightmapEncoder` — heightmap을 100×100 이미지로 reshape 후 Conv→BN→Conv→BN→MaxPool 블록 4단 (`[8,16,32,64]`)
- **MLP head**: `[256, 160, 128]` LeakyReLU + Linear → Tanh
- **출력**: `Gaussian(μ, σ)`, `log_std`는 학습 파라미터
- **평가 시**: `agent.set_running_mode("eval")` → exploration noise 제거, mean μ 사용
- **추가 후처리**: `actions[:,0] = clamp(steer × 2.0, -1, 1)` — 추론 시 조향 반응성 강제 증폭

[models.py:91](rover/sim/rover_envs/envs/navigation/learning/skrl/models.py#L91) 의 `GaussianPolicyConv` 참고.

### 4.4 학습 하이퍼파라미터 ([rover_ppo.yaml](rover/sim/rover_envs/envs/navigation/learning/skrl/configs/rover_ppo.yaml))

```yaml
rollouts: 60                # 60 step 모아 한 번 update
learning_epochs: 4
mini_batches: 60
discount_factor: 0.99
lambda: 0.95                # GAE
learning_rate: 1.0e-4
ratio_clip: 0.2             # PPO clip ε
value_clip: 0.2
kl_threshold: 0.008         # KL early stop
trainer.timesteps: 1_000_000
```

학습 환경: `num_envs=128`로 동시 시뮬레이션 → 약 **128M 경험 샘플** 소비.

### 4.5 비교 알고리즘 (PPO vs TRPO ablation)

`learning/skrl/configs/`에 **PPO, TRPO, RPO, SAC, TD3** yaml이 있고, `analysis/csv_logs/`에 PPO/TRPO wandb 익스포트가 보관돼 있습니다. `analysis/result_charts/compare_*.png`로 비교 차트가 생성됐고, **최종 선택은 PPO** (`policies/best_agent_ppo.pt`).

| Algorithm | Config | Saved policy | 비고 |
|-----------|--------|--------------|------|
| **PPO** ✓ | `rover_ppo.yaml` (1M steps) | `best_agent_ppo.pt` | **기본 사용** |
| TRPO | `rover_trpo.yaml` (200k steps) | `best_agent_trpo.pt` | 비교용 |
| RPO/SAC/TD3 | 설정만 존재 | — | 실험 가능 |

---

## 5. Repository structure

```
.
├── rover/sim/                           # ⭐ 시뮬레이션 메인
│   ├── run_ros2.sh                      # 메인 진입점 (rviz + sim 동시 실행)
│   ├── scripts/
│   │   ├── 03_eval_ros2.py              # 실제 미션 루프 (ROS2 + PPO 추론)
│   │   ├── 02_eval.py                   # 정책 단독 평가 (ROS 없음)
│   │   ├── 01_drive_test.py             # 수동 주행 테스트
│   │   ├── mission_monitor.py           # 별도 터미널 텍스트 대시보드
│   │   └── mission_viz_node.py          # rviz용 Marker pub
│   ├── mission/                         # 미션 시나리오 헬퍼
│   │   ├── command_utils.py             # ★ 목표 강제 주입 (광물/베이스 전환)
│   │   ├── robot_utils.py               # 텔레포트
│   │   ├── camera_utils.py              # 듀얼 1인칭 뷰포트
│   │   └── scene_utils.py               # 베이스캠프 USD 마커
│   └── rover_envs/                      # ⭐⭐ gym 환경 패키지
│       ├── assets/                      # USD 로봇/지형/텍스처
│       ├── mdp/actions/ackermann_actions.py  # ★ Ackermann 변환
│       ├── envs/navigation/
│       │   ├── rover_env_cfg.py         # ★ scene/obs/reward/termination
│       │   ├── entrypoints/rover_env.py # ManagerBasedRLEnv subclass
│       │   ├── mdp/{observations,rewards,terminations,randomizations}.py
│       │   ├── robots/aau_rover/
│       │   │   ├── env_cfg.py           # AAURoverEnvCfg
│       │   │   ├── __init__.py          # gym.register("AAURoverEnv-v0")
│       │   │   └── policies/            # ★ best_agent_{ppo,trpo}.pt
│       │   ├── learning/skrl/
│       │   │   ├── models.py            # ★ GaussianPolicyConv / Value
│       │   │   └── configs/*.yaml       # PPO/TRPO/RPO/SAC/TD3
│       │   └── utils/terrains/          # TerrainImporter, command 생성
│       ├── learning/agents/skrl.py      # PPO/TRPO 팩토리
│       └── utils/                       # logging, config, downloader
│
└── rover/analysis/                      # 학습/평가 분석
    ├── analyze_models_*.py              # wandb csv 비교
    ├── analysis_missions.py             # mission_log csv 시각화
    ├── csv_logs/{csv_PPO,csv_TRPO}/
    └── result_charts/*.png
```

---

## 6. Environment & hardware

### Software
| Item | Version |
|------|---------|
| OS | Ubuntu 22.04 LTS |
| Middleware | ROS2 Humble |
| Python (system) | 3.10 |
| Python (Isaac Sim) | 3.11 |
| Isaac Sim | 5.1.0 |
| Isaac Lab | 2.3.2.post1 |
| skrl | 1.4.3 |
| gymnasium | 1.2.3 |
| torch | 번들 버전 사용 (수동 교체 X) |

### Hardware tested
- MSI Vector 16 / Intel Ultra 9 275HX (NPU 포함) / 64GB RAM
- NVIDIA RTX 5080 Laptop, 16GB GDDR7
- (대안) RTX 5070 Laptop, 12GB도 `num_envs=2`로 동작 확인

---

## 7. Installation

상세 절차는 [INSTALL.md](INSTALL.md) 참조. 핵심 5단계:

```bash
# 1) Isaac Sim 5.1 standalone 다운로드 → ~/isaacsim
# 2) IsaacLab v2.3.2 clone → ~/IsaacLab, 심볼릭 링크 + install
cd ~/IsaacLab && ln -s ~/isaacsim _isaac_sim
./isaaclab.sh --install
./isaaclab.sh --install skrl

# 3) 추가 패키지
./isaaclab.sh -p -m pip install gymnasium==1.2.3 skrl==1.4.3

# 4) Mars terrain asset 다운로드 (GitHub Releases)
#    rover/sim/rover_envs/assets/terrains/mars/ 에 압축 해제

# 5) 환경변수 설정
source ~/Rokey6-B1-Isaac-simulation-project/setup_env.sh
# (영구화는 ~/.bashrc 에 source 라인 추가)
```

학습된 정책 `best_agent_ppo.pt`는 이미 레포에 포함돼 있어서 별도 다운로드 불필요.

---

## 8. How to run

### 8.1 풀 미션 (rviz + sim + ROS2)
```bash
source ~/Rokey6-B1-Isaac-simulation-project/setup_env.sh
~/Rokey6-B1-Isaac-simulation-project/rover/sim/run_ros2.sh
```
실행되면:
1. `mission_viz_node.py` (Marker pub)
2. `rviz2` (config/mission_monitor.rviz)
3. Isaac Sim + IsaacLab + 03_eval_ros2.py (PPO 추론)

### 8.2 미션 모니터 (별도 터미널)
```bash
source /opt/ros/humble/setup.bash
python3 ~/Rokey6-B1-Isaac-simulation-project/rover/sim/scripts/mission_monitor.py
```
- 터미널 텍스트 대시보드 (라운드/수집/속도/거리/평균)
- `rover/analysis/logs/mission_log_*.csv`와 `mission_events_*.csv`로 자동 저장

### 8.3 정책 단독 평가 (ROS 없이)
```bash
cd ~/IsaacLab
./isaaclab.sh -p ~/Rokey6-B1-Isaac-simulation-project/rover/sim/scripts/02_eval.py \
  --task AAURoverEnv-v0 --num_envs 2
```

### 8.4 결과 분석
```bash
cd ~/Rokey6-B1-Isaac-simulation-project/rover/analysis
python3 analysis_missions.py        # CSV → 대시보드 PNG
python3 analyze_models_compare.py   # wandb csv → PPO vs TRPO 비교 차트
```

---

## 9. Troubleshooting

| 증상 | 원인 / 해결 |
|------|-------------|
| `./isaaclab.sh: not found` | `ISAACLAB_DIR` 미설정 → `source setup_env.sh` 재실행 |
| `Invalid ROS2_BRIDGE_DIR` | `~/isaacsim/exts/isaacsim.ros2.bridge/humble/{rclpy,lib}` 경로 비어있음 |
| `No module named 'rover_envs'` | `03_eval_ros2.py`를 직접 호출하지 말고 `run_ros2.sh` 경유 |
| RTX 50 시리즈 첫 실행 느림 | Blackwell shader cache 컴파일 (5-10분, 1회만) |
| GPU OOM | `--num_envs 1`로 축소 |
| rviz가 비어있음 | `/mission/markers` 토픽이 발행되는지 `ros2 topic list` 로 확인 |

---

## 📚 References

- [Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/)
- [skrl Documentation](https://skrl.readthedocs.io/)
- [AAU Mars Rover (원본 로버 모델 출처)](https://github.com/abmoRobotics/RLRoverLab)
- [PPO paper (Schulman et al., 2017)](https://arxiv.org/abs/1707.06347)

---

## ⚠️ Note on this fork/clone

This repository was developed by [June2December](https://github.com/June2December/Rokey6-B1-Isaac-simulation-project).
No LICENSE file is present at the time of this writing — see GitHub repository for usage permissions.
