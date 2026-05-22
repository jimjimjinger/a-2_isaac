# 통합 로버 차량 (Integrated Vehicle)

팀별로 제각각이던 rover 정의를 **하나의 범용 차량 모델**로 통합한다.

## 파일

| 파일 | 설명 |
|---|---|
| `vehicle_origin_T2.usd` | T2(최진우) 원본 — rover + M0609 + RG2-FT 단일 articulation (DOF 27). **보존용, 수정 금지** |
| `vehicle_v1.usd` | **통합본 v1** — 위 베이스 + 후방 바스켓 |
| `../../scripts/build_integrated_vehicle.py` | v1 빌드 스크립트 (순수 `pxr`, Isaac Sim 불필요) |

> T5(이지민) 원본(`rover_m0609_localization.usd`)은 절대경로 의존 + 환경 혼재라
> 베이스에서 제외. T5 정의는 `isaac_manipulation/scripts/build_rover_m0609_scene.py`
> (코드)로 보존돼 있다.

## 설계 원칙

1. **단일 범용 Vehicle USD** — 주행 / manipulation / localization 어디서나 베이스로 로드
2. **모드 의존 설정은 USD 에 박지 않음** — 휠 freeze 등은 런타임 모드 레이어
3. **정적·모드무관 자산은 USD 에 굳힘** — 매 실행 조립 제거

## vehicle_v1.usd 구성

**USD 에 포함 (정적):**
- `Mars_Rover.usd` 베이스 + M0609 로봇팔 + RG2-FT 그리퍼 (단일 articulation, DOF 27)
- rover Body 카메라, IMU 센서
- gripper drive 게인, finger 물리 마찰
- 외형 dark 색칠 (`T2DarkBody` — T2 원본에 이미 적용됨)
- **후방 바스켓 (visual-only)** ← v1 에서 추가

**런타임 (USD 밖, 모드 레이어가 담당):**
- 휠 freeze, RoverAnchor FixedJoint — manipulation 모드 전용
- 제어 로직 (coverage 주행 / IK pick&place), 센서 발행, ROS2 그래프
- terrain collision·rock 제거 — 환경(world) 처리

## 내정 결정사항

- 그리퍼 = **RG2-FT** (T2 의도 존중, 향후 F/T 센서 활용)
- 후방 바스켓 = **포함, visual-only**
- 베이스 로버 = `Mars_Rover.usd` (3개 작업본 이미 동일)

## 남은 작업

- [ ] `vehicle_v1.usd` 시각 검증 — Isaac Sim 에서 열어 바스켓 위치·외형 확인
- [ ] D455 wrist 카메라 추가 (v1.1) — Nucleus 자산을 `isaac_sim/assets/d455/` 로 로컬화 후
- [ ] 경로 하드코딩 정리 (`/home/rokey/...` → repo 상대경로)
- [ ] 런타임 모드 레이어 — 주행 / manipulation 제어 모듈

## 빌드

```bash
python3 isaac_sim/scripts/build_integrated_vehicle.py
```

`vehicle_origin_T2.usd` 를 복사한 뒤 후방 바스켓을 추가해 `vehicle_v1.usd` 를 생성한다.
