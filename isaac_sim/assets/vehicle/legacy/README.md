# isaac_sim/assets/vehicle/legacy — 보존된 옛 vehicle 자산

vehicle_v3.usd 정착 이전의 단계별 자산 + AAU Mars Rover 원본. 시연 main path 에는 사용 안 함. 발표 talking point (단계별 진화) + 회귀 진단용으로 보존 (2026-05-26 cleanup 시점).

| 파일 | 역할 |
|---|---|
| `vehicle_v1.usd` | T3 coverage 검증 끝난 첫 통합 vehicle (m0609 + RG2 + AAU rover). vehicle_origin_T2 의 후속 |
| `vehicle_origin_T2.usd` | T2 (최진우) 의 원본. vehicle_v1 의 base. `build_integrated_vehicle.py` 의 입력이었음 |
| `vehicle_v2.usd` | 밸러스트 차량 v2. `build_vehicle_v2.py` 산출물. action graph 없는 정적 USD |
| `vehicle_v2_scene.usd` | v2 의 씬 시각 검증용 wrapper |
| `rover/Mars_Rover.usd` | AAU Mars rover 원본 자산 + SubUSDs/ (vehicle_v1 의 base 차체). `isaac_manipulation/scripts/` 의 standalone 데모 (pickup_demo, find_home_pose, view_wrist_cam 등) 가 직접 참조 |

**active 자산** (한 단계 위 `../vehicle_v3.usd`): action graph (센서 + 주행 + 팔 + GT odom) 모두 baked 된 자립 standalone USD. terrain 에 reference + play 만 하면 ROS2 토픽 발행 + 주행 + 팔제어 + odom 발행 자동.

**진화 흐름**:
```
AAU rover (Mars_Rover.usd)
    ↓ build_integrated_vehicle.py + T2 m0609/RG2 결합
vehicle_origin_T2.usd
    ↓ T3 coverage 검증 통합
vehicle_v1.usd
    ↓ 밸러스트 추가 + 외형 정리
vehicle_v2.usd  (build_vehicle_v2.py)
    ↓ action graph + 센서 + 팔 그래프 bake + flatten
vehicle_v3.usd  (build_vehicle_v3.py) ← 현재 시연 사용
```
