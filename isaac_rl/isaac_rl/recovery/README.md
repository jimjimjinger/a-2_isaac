# Rover Recovery RL

화성 로버 자력복구 강화학습 — Isaac Lab PPO 기반

---

## 구성 파일

| 파일 | 역할 |
|---|---|
| `recovery_env_cfg.py` | 환경 설정 (씬, 관측, 행동, 보상, 종료) |
| `recovery_mdp.py` | 관측/보상/이벤트 함수 구현 |
| `train_recovery.py` | PPO 학습 진입점 |
| `play_recovery.py` | 학습된 정책 Isaac Sim 시각화 |
| `run_curriculum.sh` | 1000 iter 단위 자동 분할 학습 스크립트 |

---

## 학습

### 처음 시작
```bash
cd ~/dev_ws/rover_ws/src/a2_isaac/isaac_rl/isaac_rl/recovery
./run_curriculum.sh
```

### 체크포인트에서 이어서
```bash
cd ~/dev_ws/rover_ws/src/a2_isaac/isaac_rl/isaac_rl/recovery
./run_curriculum.sh /home/kimi/dev_ws/rover_ws/src/a2_isaac/logs/recovery/20260527_013331/model_2997.pt
```

학습 로그: `/home/kimi/dev_ws/rover_ws/src/a2_isaac/logs/recovery/날짜_시간/`

---

## 시각화 (Isaac Sim GUI)

```bash
cd ~/dev_ws/rover_ws/src/a2_isaac/isaac_rl/isaac_rl/recovery
/mnt/data/isaac_sim/IsaacLab/isaaclab.sh -p play_recovery.py \
    --checkpoint /home/kimi/dev_ws/rover_ws/src/a2_isaac/logs/recovery/20260527_013331/model_2997.pt \
    --num_envs 4
```

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--checkpoint` | (필수) | 로드할 `.pt` 경로 |
| `--num_envs` | 4 | 동시 시각화 환경 수 |
| `--num_steps` | 10000 | 최대 실행 스텝 수 |

GUI 창을 닫으면 자동 종료됩니다.

---

## 현재 베스트 모델

| 체크포인트 | 버전 | 비고 |
|---|---|---|
| `20260527_013331/model_2997.pt` | **v3** | 최신 — 3000 iter 완료 (2026-05-27) |
| `20260526_173647/model_800.pt` | v2 | 성공률 61.4%, 피크 68.5% @ iter 578 |

---

## 학습 설정 (v3)

| 항목 | 값 |
|---|---|
| 환경 수 | 128 |
| Chunk | 1000 iter |
| 총 목표 | 3000 iter |
| 정책 주기 | 50 Hz (decimation=2) |
| 에피소드 길이 | 15초 |
| 중력 | −3.72 m/s² (화성) |
| GPU | RTX 5060 8GB |
| 행동 차원 | 6 (M0609 arm only) |
| 관측 차원 | 31 |

---

## 초기 상태 분포 (v3)

| 모드 | 비율 | roll | pitch |
|---|---|---|---|
| 옆으로 넘어짐 | 40% | ±60°~120° | < 20° |
| 뒤집힘 | 30% | ±140°~180° | < 20° |
| 비스듬히 | 30% | ±30°~70° | ±20°~50° |

---

## 보상 구조 (v3)

| 항목 | weight | 설명 |
|---|---|---|
| `upright_cosine` | +10.0 | (cos(roll)·cos(pitch)+1)/2 — 항상 [0,1] |
| `near_success` | +20.0 | 기립 근접 가우시안 신호 |
| `stable_upright` | +20.0 | 연속 upright 프레임 비율 |
| `wheel_contact` | +15.0 | 바퀴 지면 접촉 비율 |
| `arm_recovery` | +8.0 | 팔로 땅 짚고 일어나기 |
| `success_bonus` | +200.0 | 8프레임 안정 기립 달성 시 |
| `height_reward` | +5.0 | 차체 높이 상승 |
| `recovery_ang_vel` | +5.0 | 기립 방향 각속도 |
| `fallen_penalty` | −1.0 | 넘어진 상태 지속 |
| `time_penalty` | −0.1 | 매 스텝 소량 감점 |
| `joint_limit` | −2.0 | arm 관절 한계 초과 |
| `arm_vel` | −0.005 | 기립 후 arm 과속 |
| `ang_vel` | −0.02 | 폭발적 회전 억제 |
| `action_rate` | −0.01 | 급격한 동작 변화 |

---

## TensorBoard 모니터링

```bash
tensorboard --logdir /home/kimi/dev_ws/rover_ws/src/a2_isaac/logs/recovery --port 6006
# 브라우저: http://localhost:6006
```
