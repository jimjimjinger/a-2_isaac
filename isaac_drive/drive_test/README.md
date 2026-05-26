# drive_test — WASD 주행 + 레이캐스트 장애물 인식

통합 차량 `vehicle_v1.usd` 를 키보드로 조작하며, 하향 RayCaster 로 정면
장애물을 인식해 터미널에 출력하는 테스트.

avoid_test 의 **레이캐스트 부착 방식·Ackermann 액션을 그대로 가져와** 통합
차량에 붙였다.  drive_test 폴더 안에서 자급자족(self-contained)으로 동작하며,
외부 자산(`isaac_sim/assets/vehicle/vehicle_v1.usd`)은 경로 참조만 한다.

## 구성

| 파일 | 설명 |
|---|---|
| `play_wasd.py` | **메인** — WASD 수동 주행 + 장애물 인식 출력 (GUI) |
| `check_drive.py` | 자체검증 — 키보드 없이 자동 전진하며 씬 점검 (headless 가능) |
| `drive_env_cfg.py` | 씬·액션·이벤트 환경 설정 (ManagerBasedEnv) |
| `rover_vehicle.py` | `vehicle_v1.usd` 의 ArticulationCfg (27 DOF) |
| `obstacle_terrain.py` | 평지 + 큐브 장애물 1개 height-field 지형 |
| `detector.py` | 레이캐스트 히트 → 장애물 판정 헬퍼 |
| `mdp/` | avoid_test 에서 복사한 Ackermann 액션 |
| `_inspect_vehicle.py` | (일회성) vehicle_v1.usd 구조 점검 도구 |

## 실행

```bash
cd isaac_drive/drive_test

# 메인 — WASD 주행 (Isaac Sim 뷰어 창)
/home/rokey/dev_ws/venv/isaaclab/bin/python play_wasd.py

# 자체검증 — 키보드 없이 자동 전진 점검
/home/rokey/dev_ws/venv/isaaclab/bin/python check_drive.py --headless
cat /tmp/drive_check_report.txt
```

## 조작 (`play_wasd.py`)

| 키 | 동작 |
|---|---|
| `W` / `S` | 전진 / 후진 |
| `A` / `D` | 좌회전 / 우회전 |
| `R` | 차량 리셋 (스폰 위치·정면으로) |
| `ESC` | 종료 |

- 키 입력은 **Isaac Sim 뷰어 창이 포커스일 때만** 동작한다.
- 뷰어 카메라 fly 모드(마우스 우클릭 드래그)와 WASD 가 겹치므로, 주행 중에는
  우클릭을 누르지 않는다.

## 씬 / 장애물

- 평지(20×20m) 정면 **3.5m** 앞에 큐브 장애물 1개.
- 큐브 크기 **0.5 × 0.5 × 0.3 m** — avoid_test 와 동일.
- RayCaster 는 메시 1개만 인식하므로, 장애물은 별도 prim 이 아니라 지형
  메시의 '높이 돌출' 로 포함된다 (avoid_test 와 동일 방식).

## 레이캐스트 / 장애물 인식

- 하향 RayCaster — 차량 위 10m 에서 아래로 3×3m 격자 ray (해상도 0.2m →
  16×16 = 256 ray).  차량 yaw 만 따라 회전.
- 격자 안에서 지면보다 0.15m 이상 솟은 셀이 잡히면 **장애물 인식** →
  터미널에 `🚧 장애물 인식!` 출력.  격자에서 벗어나면 `장애물 벗어남` 출력.
- 격자 반경이 1.5m 이므로, 정면 3.5m 의 장애물은 차량이 약 2m 전진하면
  인식된다.

## 참고

- 차량은 rover + M0609 팔 + RG2-FT 그리퍼가 결합된 단일 articulation
  (27 DOF).  주행만 테스트하며 팔은 움직이지 않는다.
- m0609 팔은 어떤 액션에도 안 묶여 있어 그냥 두면 물리 시뮬레이션 중
  흐트러진다(접힌 HOME → 펴짐).  `rover_vehicle.py` 의 `keep_arm_folded()`
  를 매 step 호출해 팔을 접힌 HOME 자세로 고정한다.
- Ackermann 모델 특성상 직진(ang=0) 시 좌우로 약간 드리프트가 있다 —
  주행 중 `A`/`D` 로 보정한다.
