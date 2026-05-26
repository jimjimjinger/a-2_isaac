# 통합 로버 차량 (Integrated Vehicle)

팀별로 제각각이던 rover 정의를 **하나의 범용 차량 모델**로 통합한 자산.

## 현재 active 자산

| 파일 | 설명 |
|---|---|
| `vehicle_v3.usd` | ⭐ **현재 시연 사용** — 액션그래프 내장 자립 standalone USD (센서 + 주행 + 팔 그래프 + GT odom 모두 baked). terrain 에 reference + play 만 하면 ROS2 토픽 발행 + /cmd_vel 주행 + /arm/joint_command 팔제어 + /ground_truth/odom 발행 자동 |

### 사용 (시연)
```bash
tools/isaac-pypi isaac_sim/scripts/run_vehicle_v3.py --terrain terrain_00023
```

### 빌드 (USD 수정 시만)
```bash
python3 isaac_sim/scripts/build_vehicle_v3.py
```
v2 위에 action graph + 센서 + 팔 그래프 bake + flatten → v3 standalone USD 생성.

## legacy/ (보존)

vehicle_v3 정착 이전의 단계별 자산은 `legacy/` 폴더로 분리. 자세히는 [legacy/README.md](legacy/README.md).

## 진화 흐름
```
AAU rover (legacy/rover/Mars_Rover.usd)
    ↓ build_integrated_vehicle.py (legacy/scripts) + T2 m0609/RG2 결합
legacy/vehicle_origin_T2.usd
    ↓ T3 coverage 검증 통합 + 후방 바스켓
legacy/vehicle_v1.usd
    ↓ 밸러스트 차량 v2 (build_vehicle_v2.py)
legacy/vehicle_v2.usd
    ↓ action graph + 센서 + 팔 그래프 bake + flatten (build_vehicle_v3.py)
vehicle_v3.usd ← 현재 시연 사용
```

## 설계 원칙

1. **단일 범용 Vehicle USD** — 주행 / manipulation / localization 어디서나 베이스로 로드
2. **USD 는 nominal default(주행 가능 상태)를 담는다** — 그냥 로드하면 주행 가능
3. **정적·default 자산은 USD 에 굳힘** — 매 실행 조립 제거
4. **action graph 도 USD 에 baked** (v3 단계) — 런타임 그래프 빌드 제거

## 내정 결정사항

- 그리퍼 = **RG2-FT** (T2 의도 존중, F/T 센서 활용 여지)
- 후방 바스켓 = **포함, visual-only**
- 베이스 로버 = `legacy/rover/Mars_Rover.usd` (AAU Mars rover)
