# T5 Localization + Infra — Claude Code Context

> 이 파일은 Claude Code가 자동 로드하는 트랙 컨텍스트입니다.

## 너의 정체성
**T5 트랙 owner — TRN 기반 GPS-less 위치 추정 + ROS2 인프라 + Eval + Mars Tier 2**

GPU: 5080 (16GB)

## 작업 시작 전 필독
1. [T5_BRIEF.md](T5_BRIEF.md) — TRN 알고리즘 풀 구현 포함
2. [interfaces/INTERFACE_CONTRACTS.md](../interfaces/INTERFACE_CONTRACTS.md) — I5 섹션 (TRN 명세)
3. [interfaces/terrain_meta_schema.json](../interfaces/terrain_meta_schema.json) — heightmap.npy 형식

## 내가 만드는 4개 모듈 (70h)

```
1. Localization (TRN + EKF Fusion)     (40h)
   - Wheel/IMU/Sun + TRN → EKF → estimated_pose
   - publish /rover/estimated_pose @30Hz
   
2. ROS2 Infrastructure                  (10h)
   - launch 파일, 토픽 schema, rosbag
   
3. Eval Pipeline                         (10h)
   - 미션 데이터 자동 수집 + 차트
   
4. Mars Physics Tier 2                   (10h)
   - friction zone 정밀 튜닝
```

## ⭐ 핵심 컨셉

### TRN (Terrain Relative Navigation)
실제 Perseverance가 사용한 기법:
- RayCaster의 5m×5m 로컬 heightmap 측정
- T1의 global heightmap.npy와 cross-correlation 매칭
- EKF 누적 드리프트 보정

### GT는 어디서도 직접 사용 X
시뮬레이터 내부 검증용으로만. 정책 입력엔 안 들어감.

## 핵심 작업 영역

```
tracks/T5/
  ├ localization/
  │   ├ wheel_odom.py
  │   ├ imu_integration.py
  │   ├ sun_yaw.py
  │   ├ trn.py              # ⭐ 핵심
  │   └ ekf.py
  ├ launch/                  # ROS2 launch
  ├ eval/                    # 데이터 수집 + 차트
  └ mars_physics/            # Tier 2 friction zones

rover/sim/rover_envs/envs/navigation/rover_env_cfg.py   ← gravity (T1과 분담)
```

## 절대 손대지 마라
- T1, T2, T3, T4 코드
- 클론의 PPO 정책
- 인터페이스 schema (PM 승인 필수)
- **GT를 정책에 직접 주입** ❌ (시뮬 내부 검증용만)

## 핵심 의존성
- **T1 heightmap.npy** (TRN에 사용) — 좌표계 합의 필수
- **Isaac Sim sensors** (IMU, joint_vel, sphere_light pos, RayCaster)

## 도구
```bash
pip install numpy scipy  # EKF + correlation
```

## 단계별 구현

| Day | 구현 수준 |
|:---:|----------|
| 1 | Stub: GT + 가우시안 노이즈 (T3 사용 가능) |
| 2 | Wheel/IMU/Sun 적분 모듈 |
| 3 ⭐ | TRN 단독 검증 (합성 데이터로) |
| 4 | EKF 융합 + T3와 통합 |
| 5 | 노이즈 sweep, Mars Tier 2 |
| 6+ | Eval pipeline, 폴리싱 |

## TRN 핵심 코드 골격
```python
class TerrainRelativeNav:
    def __init__(self, global_heightmap_path):
        self.global = np.load(global_heightmap_path)
        self.resolution = 0.05
    
    def localize(self, local_heightmap, prior_pos, search_radius_m=3.0):
        # 5m × 5m local heightmap을 global의 prior 주변에서 검색
        # cross-correlation 최대값 위치 반환
        return estimated_pos, confidence
```

## 트러블슈팅
1. TRN correlation 모든 위치 비슷 → 평탄 지역, EKF가 알아서 처리
2. TRN 매칭이 GT랑 차이 큼 → 좌표계 (origin, resolution) T1과 재합의
3. EKF 발산 → Q, R 보수적 (작은 값)으로 시작
4. /rover/estimated_pose 발행 멈춤 → watchdog, 항상 publish (stub fallback)
