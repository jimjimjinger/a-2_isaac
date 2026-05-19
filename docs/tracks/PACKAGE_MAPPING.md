# 🗺️ 트랙 ↔ 패키지 매핑

> 우리 T1~T5 트랙이 팀 레포의 9개 패키지 어디에서 작업하는지.
> 각 BRIEF의 "작업 영역" 섹션과 함께 참조.

## 전체 매핑

```
T1 Environment (5060, 시니어)
   └─ isaac_sim/                     ← 메인 작업 영역
      ├─ worlds/                       USD 월드 파일
      ├─ assets/generated_terrains/    절차생성 batch 출력
      └─ scripts/                      ⭐ 우리가 채워야 함
         ├─ procedural_terrain_generator.py
         ├─ basecamp_visual_builder.py
         └─ mars_physics_config.py
   
   └─ isaac_perception/models/         학습 weights (Replicator 시 사용)


T2 Perception + M0609 (5080, 주니어)
   ├─ isaac_perception/                ← Vision 작업 영역
   │  └─ isaac_perception/
   │     ├─ perception_node.py          메인 publisher (이미 존재, 채워야 함)
   │     ├─ vision/                     ⭐ 우리 핵심
   │     │  ├─ mineral_detector.py       HSV 색기반 (이미 존재, 채워야 함)
   │     │  ├─ obstacle_detector.py
   │     │  ├─ terrain_analyzer.py
   │     │  └─ value_scorer.py          ⭐ 우리 추가 (광물 가치 점수)
   │     ├─ depth/depth_estimator.py
   │     └─ lidar/                      미구현 (확장 여지)
   │
   └─ isaac_manipulation/              ← M0609 작업 영역
      └─ isaac_manipulation/
         ├─ arm_executor_node.py        M0609 동작 실행
         └─ primitives/
            ├─ pick_mineral.py
            ├─ place_to_cargo.py
            ├─ unload_to_base.py
            └─ deploy_solar_panel.py


T3 Driving (5080, 시니어) — Critical Path
   ├─ isaac_drive/                     ← 주행 메인 영역
   │  └─ isaac_drive/
   │     ├─ drive_manager_node.py       자율/수동 흐름 (rename: nav_manager)
   │     ├─ mobile_base_executor_node.py 휠 명령 실행
   │     ├─ navigation/                 ⭐ 우리 추가 (planning)
   │     │  ├─ coverage_planner.py       Roomba 알고리즘
   │     │  ├─ path_planner.py           A*
   │     │  └─ mission_fsm.py            FSM
   │     └─ primitives/
   │        ├─ drive_to_target.py
   │        ├─ avoid_obstacle.py
   │        └─ stop_rover.py
   │
   └─ isaac_rl/                        ← PPO 영역 (T3가 함께 다룸)
      └─ isaac_rl/
         ├─ driving_policy_node.py       RL inference 노드
         ├─ policy_loader.py             best_agent_ppo.pt 로드
         ├─ rl_environment.py            학습 환경 wrapper
         ├─ reward_function.py
         ├─ rl_trainer.py                재학습 시 사용
         └─ ppo_wrapper.py              ⭐ 우리 추가 (waypoint → action)


T4 Integration + PM (사용자, 5070 Ti)
   ├─ isaac_bringup/                   ← launch 통합
   │  └─ launch/                        8개 launch 파일
   │     ├─ full_system.launch.py
   │     ├─ sim.launch.py
   │     ├─ perception.launch.py
   │     ├─ rl.launch.py
   │     ├─ drive.launch.py
   │     ├─ supervisor.launch.py
   │     ├─ manipulation.launch.py
   │     └─ localization.launch.py
   │
   ├─ isaac_supervisor/                ← mission orchestration
   │  └─ isaac_supervisor/
   │     ├─ mission_manager_node.py     top-level 감독
   │     └─ battery_monitor_node.py
   │
   └─ docs/pm_tools/                   PM 운영 도구 (DAILY_STATUS 등)


T5 Localization + Infra (5080)
   └─ isaac_localization/              ⭐ 신규 전체
      └─ isaac_localization/
         ├─ localization_node.py        /rover/estimated_pose publisher
         ├─ ekf_fusion.py               다중 센서 EKF
         ├─ trn.py                      ⭐ TRN 핵심
         └─ sensors/
            ├─ wheel_odom.py
            ├─ imu_integrator.py
            └─ sun_yaw.py
```

## 인터페이스 매핑 (우리 I1~I5 ↔ 팀 메시지)

| 우리 I | 팀 메시지 | 위치 | 비고 |
|:------:|----------|------|------|
| **I1** | (없음, 파일 기반) | `isaac_sim/assets/generated_terrains/` | 우리 schema 그대로 |
| **I2** | `PerceptionResult.msg` | `isaac_interfaces/msg/` | 팀 정의 사용. value_score 추가 협상 필요 |
| **I3** | `ExecuteArmTask.action` | `isaac_interfaces/action/` | 팀 Action 사용 (더 좋음) |
| **I4** | `ExecuteArmTask.action` result | (same) | Action의 feedback/result로 통합 |
| **I5** | `PoseWithCovarianceStamped` (ROS2 표준) | (없음, 표준 사용) | T5가 publish, 표준 메시지 |

## Day 1 회의에서 합의 필요

```
□ PerceptionResult.msg 필드 확인 (value_score 추가?)
□ ExecuteArmTask.action vs 우리 I3/I4 흐름 비교 → 팀 Action 채택
□ /rover/estimated_pose 추가 (PoseWithCovarianceStamped)
□ BatteryState 처리 → 우리 미션 scope 확장 (배터리 관리)
□ SaveExplorationMap.srv → minimap 저장 기능
```
