# Rover Recovery RL

넘어진 Rover를 **M0609 팔 + 바퀴**를 동시에 사용해 스스로 일으켜 세우는 강화학습 환경.

---

## 구성 파일

| 파일 | 역할 |
|---|---|
| `recovery_env_cfg.py` | Isaac Lab 환경 설정 (씬, 관측, 행동, 보상, 종료) |
| `recovery_mdp.py` | 관측/보상/이벤트 함수 구현 |
| `train_recovery.py` | PPO 학습 진입점 |
| `recovery_node.py` | 학습된 정책을 ROS2로 실행하는 노드 |

---

## 환경 개요

### 씬

```
[Ground Plane]
    ├── Rover (vehicle_v3.usd)  — 넘어진 상태로 랜덤 초기화
    └── M0609 Arm               — Rover 옆 1.2m 고정 베이스
```

- 중력: **화성 중력 −3.72 m/s²**
- 물리 주기: 200 Hz / 정책 주기: 50 Hz (decimation=4)
- 에피소드 길이: **15초 (750 step)**

### 초기 상태

| 항목 | 값 |
|---|---|
| Rover roll | 60° ~ 120° (완전히 옆으로 넘어짐) |
| Rover pitch | −30° ~ +30° (랜덤) |
| Rover 위치 xy | ±0.5 m 랜덤 |
| Rover 바퀴/스티어 | 0 rad, 0 rad/s |
| M0609 자세 | 홈 자세 |

---

## 행동 공간 (dim=12)

| 인덱스 | 관절 | 제어 방식 | 스케일 |
|---|---|---|---|
| 0–5 | M0609 joint_1 ~ joint_6 | Position target | ±0.5 rad/step |
| 6–11 | FL/FR/CL/CR/RL/RR\_Drive\_Continuous | Velocity target | ±15 rad/s |

> 스티어 관절(FL/FR/RL/RR\_Steer\_Revolute)은 관측에만 포함, 행동 제어 없음.

---

## 관측 공간 (dim=32)

| 항목 | dim | 설명 |
|---|---|---|
| rover_roll, pitch, yaw | 3 | 오일러각 (rad) |
| rover_pos_z | 1 | 지면 대비 높이 (m) |
| rover_lin_vel | 3 | 선속도 (m/s) |
| rover_ang_vel | 3 | 각속도 (rad/s) |
| arm_joint_pos | 6 | M0609 관절 위치 (rad) |
| arm_joint_vel | 6 | M0609 관절 속도 (rad/s) |
| rover_drive_vel | 6 | 드라이브 바퀴 각속도 (rad/s) |
| rover_steer_pos | 4 | 스티어 관절 위치 (rad) |

---

## 보상 설계

| 보상 항목 | weight | 수식 / 조건 |
|---|---|---|
| `upright_cosine` | **+10.0** | cos(roll) × cos(pitch) — upright=1.0, 90°옆=0.0 |
| `height_reward` | **+5.0** | clamp((z − 0.30) / 0.30, 0, 1) — 몸체가 올라올수록 |
| `success_bonus` | **+500.0** | |roll|<15° AND |pitch|<15° 달성 시 1.0 (sparse) |
| `fallen_penalty` | **−3.0** | tilt > 75° 상태 지속 시 1.0 — 정체 방지 |
| `time_penalty` | **−0.2** | 매 스텝 1.0 — 빠른 기립 압박 |
| `wheel_drive_bonus` | **+2.0** | tilt>45° 상태에서 바퀴 회전속도 clamp(rms/10, 0, 1) |
| `arm_vel_penalty` | −0.005 | Σ(arm\_joint\_vel²) — 팔 부드럽게 |
| `joint_limit_penalty` | −2.0 | 팔 관절 소프트 한계 초과량 합 |

**보상 전략 요약:**
- 기립에 가까울수록 `upright_cosine`과 `height_reward`가 지속적으로 큰 양의 신호 제공
- 성공 시 `success_bonus` 500으로 강한 목표 유도
- `fallen_penalty` + `time_penalty`로 넘어진 채 버티는 lazy 정책 방지
- `wheel_drive_bonus`로 팔만 쓰는 것이 아니라 바퀴 회전을 통한 모멘텀 활용 유도

---

## 종료 조건

| 조건 | 설명 |
|---|---|
| `rover_upright` | |roll|<15° AND |pitch|<15° → **성공 종료** |
| `time_out` | 15초 초과 → 실패 종료 |

---

## 학습 실행

```bash
cd ~/dev_ws/rover_ws/src/a2_isaac/isaac_rl/isaac_rl/recovery

# 신규 학습
/mnt/data/isaac_sim/IsaacLab/isaaclab.sh -p train_recovery.py \
    --num_envs 64 \
    --max_iterations 5000 \
    --headless

# 체크포인트에서 재개
/mnt/data/isaac_sim/IsaacLab/isaaclab.sh -p train_recovery.py \
    --num_envs 64 \
    --max_iterations 5000 \
    --headless \
    --checkpoint <path/to/model_XXXX.pt>
```

### TensorBoard 모니터링

```bash
tensorboard --logdir ~/dev_ws/rover_ws/src/a2_isaac/logs/recovery --port 6006
# 브라우저: http://localhost:6006
```

---

## PPO 하이퍼파라미터

| 항목 | 값 |
|---|---|
| num_steps_per_env | 24 |
| num_learning_epochs | 5 |
| num_mini_batches | 4 |
| learning_rate | 1e-3 (adaptive) |
| clip_param | 0.2 |
| gamma / lambda | 0.99 / 0.95 |
| desired_kl | 0.01 |
| entropy_coef | 0.005 |
| network | [256, 128, 64] ELU |

---

## 출력 파일

```
logs/recovery/<YYYYMMDD_HHMMSS>/
    ├── model_200.pt, model_400.pt, ...   # 200 iter 마다 저장
    └── events.out.tfevents.*             # TensorBoard 로그

policies/
    └── recovery_policy.pt               # 학습 완료 후 최종 정책
```

---

## 주요 관절명 (vehicle_v3.usd)

| 종류 | 관절명 |
|---|---|
| Drive (6) | `FL_Drive_Continuous`, `FR_Drive_Continuous`, `CL_Drive_Continuous`, `CR_Drive_Continuous`, `RL_Drive_Continuous`, `RR_Drive_Continuous` |
| Steer (4) | `FL_Steer_Revolute`, `FR_Steer_Revolute`, `RL_Steer_Revolute`, `RR_Steer_Revolute` |
| Arm (6) | `joint_1` ~ `joint_6` |
