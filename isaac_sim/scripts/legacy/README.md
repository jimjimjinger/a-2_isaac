# isaac_sim/scripts/legacy — 보존된 옛 도구

vehicle_v3 / mars_terrain_generator_v3 정착 이전의 도구들. 시연 main path 에는 사용 안 함. 향후 비교·검증·재참고용으로 보존 (2026-05-26 cleanup 시점).

| 파일 | 역할 | 대체 (active) |
|---|---|---|
| `build_integrated_vehicle.py` | v1 시기 통합 vehicle 빌더 (T2 vehicle_origin_T2.usd 와 m0609 + RG2 결합) | `../build_vehicle_v3.py` |
| `build_vehicle_v2.py` | 밸러스트 차량 v2 빌더 (외형/물리/관절). v3 빌더의 입력 (`vehicle_v2.usd`) 생성용 | `../build_vehicle_v3.py` (v2 위에 action graph + sensors + arm 그래프 bake) |
| `run_vehicle_v2_scene.py` | v2 씬 시각 검증 런처 (action graph 없음) | `../run_vehicle_v3.py` |
| `mars_terrain_generator_v2.py` | v2 generator — epic obstacle 없음, 22 terrain 생성한 옛 정착 | `../mars_terrain_generator_v3.py` (epic obstacle 4종 통일) |
| `localize_d455.py` | D455 카메라 calibration helper (intrinsic/extrinsic 측정) | (v3 USD 안에 baked) |
| `teleop_rover_keyboard.py` | 키보드 teleop (cmd_vel publish) | mvp.launch.py 의 mission_manager 가 직접 발행 |

**언제 다시 꺼낼 일이 있는지**:
- v3 generator 가 만든 terrain 과 v2 의 결과 정량 비교 (passable_ratio / difficulty score 등)
- vehicle_v3.usd 가 깨지는 회귀 발견 시 v2 빌더로 step-by-step 재현
- 새 카메라 도입 시 localize_d455 의 calibration 패턴 참고
