# list_to_fix.md — 수정 대기 목록

작동에는 지장 없지만 정리가 필요한 항목들. 발견 시 추가하고, 고치면 체크.

---

## 🎯 시연 후 졸업 작업 (cheat 청산)

### [ ] T5 Localization 정확도 향상 (cheat 청산 필수)

**현재 상태** — MVP 는 `odom_to_estimated_pose` 어댑터로 `/ground_truth/odom`을 `/rover/estimated_pose`로 forwarding (GT cheat). T5 EKF stack (`localization.launch.py`)은 구현됐으나 정확도 부족으로 시연용 미채택.

**문제**:
- `sun_yaw` measurement 의 innovation 이 EKF 예측과 자주 mismatch → 절대 yaw 보정 안 됨 (`world_sun_yaw` / `camera_yaw_offset` tune 필요)
- `trn_node` (TRN — heightmap 매칭) 정확도 검증 미완. 현재 ekf_fusion 도 trn_pose 무시 가능성.
- wheel slip → wheel_odom drift (특히 등반 시)

**졸업 경로**:
1. sun_yaw 노드의 camera_yaw_offset 캘리브레이션
2. trn_node 의 heightmap 매칭 정확도 검증 + EKF 통합
3. wheel friction 강화 (휠 prim 식별 + PhysicsMaterial binding) 로 slip 감소
4. 검증 PASS 시 mvp.launch.py 에서 `odom_to_estimated_pose` 제거 + `arm_executor`/`supervisor` 의 `odom_topic` 을 `/rover/estimated_odom` 으로 swap

**fallback** — 정확도 못 올리면 GT cheat 영구 유지 (현재 시연용 상태). 그러나 발표 시 "정공법" 주장 약화.

---

### [ ] T2 Perception z bias 해결 (arm cheat 청산)

**증상** — perception 의 mineral `world_position.z` 가 GT 보다 일관되게 **+40~56cm 위**로 추정. 평균 +47cm.

| target | dz (target - GT) |
|---|---|
| (-6.95, 13.04, **1.33**) | +0.48m |
| (9.02, 11.90, **0.52**) | +0.56m |
| (21.27, 8.52, **1.16**) | +0.39m |
| (20.53, 7.83, **1.21**) | +0.44m |

**원인 추정** — YOLO bbox center 가 30cm 광물 mesh 의 *상단* 표면 픽셀 → depth backproject 가 top 표면 z 잡음.

**임시 보정** — `arm_executor` 의 `ik_descend_dz=-0.40` 으로 IK target 을 perception 추정 위치 -40cm 으로 (mvp.launch.py default). 정공법 아님.

**졸업** — depth backproject 시 bbox 의 *bottom* 또는 *center* 픽셀 사용 + 광물 mesh 의 alignment 보정. 해결되면 mvp.launch.py 의 `ik_descend_dz` 제거 (default `+0.05` 복귀).

---

### [ ] 미네랄 충돌체 — 동적 강체에 삼각형 메시 (PhysX 경고)

**증상** — Isaac Sim 기동 시 `omni.physx` 에러 로그 다수:

> `triangle mesh collision (approximation None) cannot be a part of a dynamic body, falling back to convexHull approximation: .../Minerals/blue_mineral_XXXX/Reference/Cube`

**원인** — 미네랄은 그리퍼로 집을 수 있게 동적 RigidBody인데, 마커 자산 내부
`/scene/Cube` 충돌체가 `physics:approximation = none`(정확한 삼각형 메시)으로
저작돼 있음. PhysX는 동적 강체에 삼각형 메시 충돌을 허용하지 않음.

- `isaac_sim/assets/markers/tier2_mineral/blue_mineral.usd` — 해당
- `isaac_sim/assets/markers/tier2_mineral/yellow_mineral.usd` — 해당
- `green_gas.usd` — 충돌 Cube 없음 (에러 안 남)

**영향** — 없음. PhysX가 convexHull로 자동 대체 → 시뮬레이션 정상 동작.
기동 로그만 시끄러움.

**수정** — 위 두 자산의 `/scene/Cube` `physics:approximation`을
`none → convexHull`로 변경. world USD가 자산을 참조하므로 terrain 재생성 불필요.

**발견** — 2026-05-21, terrain_00022 시뮬레이션 중 (커밋 208e51c 시점)

---

## 🚀 기능 확장 (시연 후)

### [ ] 머신러닝 활용 (PPO RL 통합)

**현재** — `isaac_rl/driving_policy_node.py` 가 PPO inference 골격만 (stub). 학습된 정책 없음. 시연에선 비활성.

**계획**:
1. `isaac_rl/rl_environment.py` 구현 — Isaac Sim ↔ gym-style Env wrapper (observation = lidar/카메라/IMU, action = cmd_vel)
2. `reward_function.py` — coverage 효율 + mineral 수집 + 에너지/충돌 penalty
3. `rl_trainer.py` — PPO 학습 (stable-baselines3 또는 RLLib)
4. `policy_loader.py` — 학습 정책 load + `driving_policy_node` 가 inference
5. 통합: supervisor 가 EXPLORE 시 RL policy 호출 vs coverage BCD (param 으로 분기)

**용도** — 단순 coverage BCD 대비 동적 obstacle 회피 + 효율적 mineral 우선순위 학습 가능.

---

### [ ] UI 개발 (mission control dashboard)

**현재** — `minimap_publisher` 가 단순 2D map + rover 위치 + path 표시. 미션 진행 상황 모니터링 부족.

**필요 기능**:
- Mission phase 표시 (EXPLORE/APPROACH/PICK/CARGO)
- Cargo count / value score 누적
- mineral detection 실시간 list (class + 좌표 + confidence)
- arm action progress + grip 상태
- localization 정확도 (covariance 시각)
- 에너지/배터리 (battery_monitor 통합)

**구현 옵션**:
- (A) rqt 플러그인 추가 (rqt_plot + rqt_image_view 조합)
- (B) Streamlit / Plotly Dash 별도 ROS bridge
- (C) rviz2 panel (custom plugin)

**우선순위** — 발표 시각 자료 위해 (A) 또는 (C) 권장.

---

### [ ] Rover obstacle 회피 — 충돌 + 과민감 trade-off

**증상** — vehicle_v3 (실제 차체) 와 coverage A*/BCD 의 obstacle 회피 layer 간 mismatch.

- (a) **충돌 이슈** — 명목 footprint 와 USD 콜라이더 영역이 어긋나 일부 obstacle 에 차체가 부딪힘 (특히 epic obstacle 같은 큰 자산 추가 시).
- (b) **과민감 회피** — (a) 회피용으로 `robot_radius` (또는 keepout/inflation) 를 키우면, 통과 가능한 좁은 통로조차 가지 못하고 우회 (e.g., terrain 베이스캠프 근처 obstacle 의 1.5m 거리 지점도 회피).

**원인 추정**:
- vehicle_v3 콜라이더 (휠 포함) 의 실제 외접 반경 측정 미완 → coverage 의 `robot_radius` 가 실측 기반이 아님.
- inflation 이 uniform (한 값) 이라 큰 obstacle 우회와 좁은 통로 통과를 동시에 만족 못 함.

**졸업 방향**:
- (1) vehicle_v3 USD bbox 측정 후 `robot_radius` 정합 (베이스 + 휠 외접 + 안전 margin 분리).
- (2) obstacle_grid 에 obstacle 별 inflation 등급 (epic = 더 키움, 작은 rock = 작게) — terrain generator 단계에서 marking.
- (3) coverage 의 keepout 과 A* 의 inflation 을 분리 (현 v2 단일 generator 는 keepout=6 으로 통합).

**발견** — 2026-05-26, PR #11 (terrain_00023 epic obstacle 도입) 검토 시점.

---

## 🛠️ 인프라 / 정리

### [ ] outdated launch 파일 정리

`isaac_bringup/launch/` 의 다음 launch 들이 mvp.launch.py 와 mismatch:
- `supervisor.launch.py` — `battery_monitor_node` 띄움. 실제 mission 노드는 `mission_manager_node` 라 잘못된 default.
- `perception.launch.py` — `perception_node` (stub) 띄움. 실제는 `yolo_perception_node`.
- `drive.launch.py` — coverage_node 만, param 없음 (mvp 와 비교 시 cmd_vel remap + robot_radius 빠짐).
- `manipulation.launch.py` — arm_executor 만, ik_descend_dz 등 default 외 없음.
- `full_system.launch.py` — 위 outdated launch 들 include. 실 동작 검증 안 됨.

**수정** — 각 launch 파일을 mvp.launch.py 와 동일 패턴으로 갱신 + full_system 에 T5 localization include 옵션 추가.

---

### [ ] 단위 테스트 환경 — grip 외 확장

`isaac_sim/scripts/test_grip_unit.py` 가 grip 단위 테스트로 PASS 검증. 단 다른 노드는 단위 테스트 없음.

**확장 후보**:
- coverage_node — terrain + GT odom mock 으로 A* / BCD 동작 검증
- perception YOLO — synthetic mineral 이미지 → world_position 정확도 측정 (z bias 진단 자동화)
- arm_executor IK — DLS-IK 의 다양한 mineral 위치 reach 검증 (이미 단위 테스트 일부 있음)

**가치** — CI 통합 시 회귀 방지. 다만 Isaac Sim 통합 테스트는 GPU + GUI 라 CI 부담.

---

### [ ] README.enhanced.md archive 결정

`docs/README.enhanced.md` 가 README.md 와 중복 정보. archive 디렉토리 이동 또는 삭제.

---

## 💡 프로젝트 측면 추가 고려사항

### [ ] 발표 자료 / docs/system_design 갱신

- `docs/system_design/ARCHITECTURE_EVAL_2026_05_25.md` (현 시점 시스템 스냅샷) → 2026-05-26 시연 후 후속 평가 추가
- Demo video / GIF 캡쳐 (rover 자율 mineral 수집 loop)
- 시연 talking points: cheat 명시 + 졸업 경로 (정공법으로 어떻게 가는지)

### [ ] terrain 다양성 + 난이도 mode

- 현재 terrain_00001~00022 모두 비슷한 난이도. 발표용 "easy/medium/hard" 셋 분류
- 평지 위주 (easy) vs 언덕 많음 (hard) — 등반 시 wheel gain trade-off 시각 비교

### [ ] 협업 / 인수인계 가이드

- 각 트랙별 README (`isaac_*/0X_*_README.md`) 가 현재 상태와 동기화 안 됨. 트랙 owner 가 본인 노드 + 인터페이스 + tune param 정리
- `docs/tracks/T*_BRIEF.md` 도 시연 후 업데이트

### [ ] 회복 시나리오 — rover stuck / arm fail

- mission_manager 의 timeout/abort 흐름 검증 (`approach_lock_timeout_sec`, `explore_resume_delay_sec`)
- arm action abort 시 cargo basket 까지 못 가는 mineral 의 release 처리
- battery low 시 base 복귀 로직 (`battery_monitor_node` 활성화)

### [ ] 에너지 / 자원 모델 정공법

- 현재 mass 변화 (cargo 적재) 만 시뮬. 배터리/전력 소비 모델 없음.
- mission planner 가 에너지 예산 고려해 mineral 우선순위 결정 가능 (RL reward 와 결합)
