# 📍 T5 (이지민) Localization + Infra — 담당자 브리프

> **TRN 기반 위치 추정 + ROS2 인프라 + Eval + Mars Physics Tier 2**
> 학술적 깊이가 큰 트랙. "GPS 없는 화성에서 어떻게 위치를?" 의 답.

---

> 📦 **이 트랙이 작업하는 패키지 위치**: [PACKAGE_MAPPING.md](PACKAGE_MAPPING.md) 참조 (팀 레포 9개 패키지 중 어디서 코딩하는지 명시).


## 📑 목차

1. [왜 이 트랙이 학술적으로 깊은가](#1-왜-이-트랙이-학술적으로-깊은가)
2. [당신이 만들 4개 모듈](#2-당신이-만들-4개-모듈)
3. [⭐ TRN (Terrain Relative Navigation)](#3--trn-terrain-relative-navigation)
4. [Multi-sensor Fusion (EKF)](#4-multi-sensor-fusion-ekf)
5. [ROS2 Infrastructure](#5-ros2-infrastructure)
6. [Eval Pipeline](#6-eval-pipeline)
7. [Mars Physics Tier 2](#7-mars-physics-tier-2)
8. [인터페이스](#8-인터페이스)
9. [일정과 마일스톤](#9-일정과-마일스톤)
10. [흔한 함정](#10-흔한-함정)
11. [DoD](#11-dod)

---

## 1. 왜 이 트랙이 학술적으로 깊은가

### 핵심 질문

> **"화성엔 GPS가 없다. 그럼 로봇은 자기 위치를 어떻게 아는가?"**

이 질문에 답하는 게 T5 (이지민)의 일. 실제 NASA Curiosity, Perseverance가 푸는 문제와 동일.

### 우리의 답 (가짜 아님)

```
GT cheat ❌    →   IMU + Wheel + Sun + TRN 융합 ✅
```

**TRN (Terrain Relative Navigation)**:
- Perseverance가 화성 착륙 시 사용한 실제 기법
- 로컬 heightmap을 전역 heightmap에 매칭
- 우리 시뮬레이션에서 그대로 구현

### 클론과의 결정적 차이

| | 클론 | 우리 |
|---|------|------|
| 위치 추정 | GT 직접 사용 (cheat) | IMU+Wheel+Sun+TRN 융합 |
| GT 사용? | 정책 입력에 직접 | 시뮬 내부 검증용만 |
| 학술 키워드 | 없음 | "Multi-sensor fusion + TRN" |

### 발표 임팩트

> *"실제 화성 로버와 동일한 기법: 다중 센서 융합 + 지형 상관 매칭 (TRN). GT 사용 없이 GPS-less 환경에서 위치 추정."*

→ 학술적 정직성 + 진짜 화성 모방.

---

## 2. 당신이 만들 4개 모듈

```
T5 (이지민) = Localization + Infra + Eval + Mars
   │
   ├ 1. Localization (TRN + EKF Fusion)     (40h)
   │    └ Wheel/IMU/Sun + TRN → EKF → estimated_pose
   │
   ├ 2. ROS2 Infrastructure                  (10h)
   │    └ launch 파일, 토픽 schema, rosbag
   │
   ├ 3. Eval Pipeline                         (10h)
   │    └ 미션 데이터 자동 수집 + 차트 생성
   │
   └ 4. Mars Physics Tier 2                   (10h)
        └ friction zone 정밀 튜닝
        
        총 70h (살짝 over, 핵심은 1번)
```

---

## 3. ⭐ TRN (Terrain Relative Navigation)

### 원리

```
[로버 주변 5m × 5m 로컬 heightmap]    [T1 (김현중) 전역 heightmap]
                                       
   ◾ ◾ ◽ ◽                              ◽ ◽ ◾ ◾ ◽◽ ◾
   ◾ ◾ ◽ ◽◽          ──── ?? ────→     ◽ ◽◽ ◾ ◾ ◽◽
   ◾ ◾ ◽                                 ◽ ◾ ◽ ◾ ◾  ← 여기 일치!
                                          ◽ ◾ ◽◽ ◾
   ↑                                      
   RayCaster로 측정                       1000 × 1000 heightmap.npy
                                          (T1 (김현중) 산출물)
```

**알고리즘**: 로컬 패턴을 전역 지도에서 cross-correlation으로 검색.

### 구현

```python
import numpy as np
from scipy.signal import correlate2d

class TerrainRelativeNav:
    def __init__(self, global_heightmap_path, resolution_m=0.05):
        self.global_heightmap = np.load(global_heightmap_path)
        self.resolution = resolution_m
        self.origin_m = (-25.0, -25.0)
    
    def localize(self, local_heightmap, prior_position, search_radius_m=3.0):
        """
        local_heightmap: RayCaster 출력 (100, 100) — 5m × 5m × 0.05/cell
        prior_position: EKF가 추정한 사전 위치 (numpy array, [x, y])
        search_radius_m: 검색 반경 (m)
        
        returns: (estimated_position, confidence)
        """
        # 1. prior 주변에서 전역 patch 추출
        prior_cell = self._world_to_cell(prior_position)
        search_cells = int(search_radius_m / self.resolution)
        
        best_score = -np.inf
        best_offset = (0, 0)
        
        # 2. Grid search (실전엔 더 효율적 방법 — gradient descent)
        for di in range(-search_cells, search_cells, 5):  # step 5 cells = 25cm
            for dj in range(-search_cells, search_cells, 5):
                cx, cy = prior_cell[0] + di, prior_cell[1] + dj
                
                # 전역 지도에서 5m × 5m 패치 추출
                half = 50  # 5m / 0.05 / 2 = 50 cells / 2
                try:
                    global_patch = self.global_heightmap[
                        cx - half : cx + half,
                        cy - half : cy + half
                    ]
                    if global_patch.shape != (100, 100):
                        continue
                except IndexError:
                    continue
                
                # 3. Cross-correlation 점수
                score = self._normalized_cross_correlation(
                    local_heightmap, global_patch
                )
                
                if score > best_score:
                    best_score = score
                    best_offset = (di, dj)
        
        # 4. 정밀화 (best offset 주변에서 1-cell 검색)
        for di in range(best_offset[0] - 2, best_offset[0] + 3):
            for dj in range(best_offset[1] - 2, best_offset[1] + 3):
                # ... same as above, finer search
                pass
        
        # 5. 결과 → world coords
        result_cell = (prior_cell[0] + best_offset[0], 
                       prior_cell[1] + best_offset[1])
        result_world = self._cell_to_world(result_cell)
        
        confidence = best_score  # -1 ~ 1, 1 = perfect match
        return result_world, confidence
    
    @staticmethod
    def _normalized_cross_correlation(a, b):
        """평균/표준편차 정규화한 cross-correlation"""
        a = (a - a.mean()) / (a.std() + 1e-6)
        b = (b - b.mean()) / (b.std() + 1e-6)
        return np.mean(a * b)
    
    def _world_to_cell(self, world_pos):
        cx = int((world_pos[0] - self.origin_m[0]) / self.resolution)
        cy = int((world_pos[1] - self.origin_m[1]) / self.resolution)
        return (cx, cy)
    
    def _cell_to_world(self, cell):
        wx = cell[0] * self.resolution + self.origin_m[0]
        wy = cell[1] * self.resolution + self.origin_m[1]
        return np.array([wx, wy])
```

### 사용 패턴

```python
# 매 step이 아니라 가끔 (5초마다)
if step % 150 == 0:  # 30Hz × 5s = 150
    local_heightmap = env.scene["height_scanner"].data.ray_hits_w
    # ... reshape to (100, 100)
    
    estimated, confidence = trn.localize(local_heightmap, ekf.current_pos)
    
    if confidence > 0.7:  # 신뢰도 높을 때만 적용
        ekf.update_with_measurement(estimated, covariance_from_confidence(confidence))
```

→ **드리프트 누적을 가끔 보정**. 매 step 하면 너무 비쌈.

---

## 4. Multi-sensor Fusion (EKF)

### 4개 센서 처리

#### Wheel Odometry
```python
def wheel_odometry(prev_pose, joint_vel, dt, cfg):
    """Ackermann 모델로 로봇 motion 추정"""
    # 6륜 평균 속도
    v_avg = np.mean(joint_vel[:6]) * cfg.wheel_radius  # m/s
    
    # 조향각 평균 (FL과 FR 평균)
    steer_avg = np.mean(joint_vel[6:8])  # rad
    
    # 자전거 모델
    dx = v_avg * np.cos(prev_pose[3]) * dt
    dy = v_avg * np.sin(prev_pose[3]) * dt
    dyaw = (v_avg / cfg.wheelbase) * np.tan(steer_avg) * dt
    
    return prev_pose + np.array([dx, dy, 0, dyaw])
```

#### IMU Integration
```python
def imu_integration(prev_pose, imu_data, dt):
    """자이로 + 가속도 적분"""
    ax, ay, az = imu_data.linear_acceleration
    wx, wy, wz = imu_data.angular_velocity
    
    # 자세 적분 (자이로)
    new_yaw = prev_pose[3] + wz * dt
    
    # 위치 적분 (가속도 — 노이즈 누적 큼!)
    # 일반적으로 IMU만으로 position 안 씀. 자이로만 사용
    return prev_pose + np.array([0, 0, 0, wz * dt])
```

#### Sun Yaw (절대 방위)
```python
def sun_yaw(sun_direction_world, robot_quat):
    """태양 방향 + 로봇 자세 → 절대 yaw"""
    # 로봇 frame에서 태양 방향
    sun_robot = quat_rotate(quat_conjugate(robot_quat), sun_direction_world)
    
    # 태양은 알려진 방향 (예: 정남쪽)에 있다고 가정
    expected_sun_world = np.array([0, -1, 0])
    
    # 실제와 기대의 각도 차이 = 로봇 yaw
    yaw_correction = np.arctan2(sun_robot[1], sun_robot[0]) - \
                     np.arctan2(expected_sun_world[1], expected_sun_world[0])
    return yaw_correction
```

### EKF 융합

```python
import numpy as np

class EKF:
    def __init__(self, initial_pose):
        self.x = initial_pose.copy()  # [x, y, z, yaw]
        self.P = np.eye(4) * 0.01      # 공분산
        
        # Process noise (motion model 불확실성)
        self.Q = np.diag([0.01, 0.01, 0.001, 0.005])
        
        # Measurement noise (각 센서별)
        self.R_wheel = np.diag([0.05, 0.05, 0.0, 0.01])
        self.R_sun = np.diag([0.0, 0.0, 0.0, 0.001])   # 정확
        self.R_trn = np.diag([0.1, 0.1, 0.0, 0.0])      # 가끔 발동
    
    def predict(self, motion_delta):
        """motion model 적용 (wheel odom)"""
        self.x = self.x + motion_delta
        self.P = self.P + self.Q
    
    def update(self, measurement, R, H=None):
        """measurement model"""
        if H is None:
            H = np.eye(4)
        
        # Kalman gain
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # 갱신
        innovation = measurement - H @ self.x
        self.x = self.x + K @ innovation
        self.P = (np.eye(4) - K @ H) @ self.P
    
    def get_state(self):
        return self.x, self.P
```

### 통합 사이클

```python
# 매 30Hz step
ekf.predict(wheel_odom_delta)            # 빠름, 항상 실행

if has_sun_measurement:                  # 1Hz
    ekf.update(sun_yaw_measurement, R_sun, H=[0,0,0,1])

if step % 150 == 0:                       # 5초마다
    trn_estimate, confidence = trn.localize(...)
    if confidence > 0.7:
        ekf.update(trn_estimate, R_trn / confidence)

# Publish
publish_estimated_pose(ekf.get_state())
```

---

## 5. ROS2 Infrastructure

### launch 파일

```python
# tracks/T5 (이지민)/launch/full_system.launch.py
import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(package='t5_localization', executable='localization_node'),
        Node(package='t2_perception', executable='vision_node'),
        Node(package='t2_manip', executable='manip_node'),
        Node(package='t3_driving', executable='driving_node'),
        Node(package='t4_ui', executable='ui_node'),
        
        # RViz
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', 'config/mission.rviz']),
    ])
```

### 토픽 schema (INTERFACE_CONTRACTS.md와 일치)

- `/rover/estimated_pose` (geometry_msgs/PoseWithCovarianceStamped) — 본인 publish
- `/rover/pose_sources` (custom) — 4개 센서 각각 publish (UI 디버깅용, deferred)
- `/perception/detections`, `/mission/*` — 다른 트랙

### rosbag 녹화 도구

```bash
# 발표 데모 시 자동 녹화
ros2 bag record -o demo_$(date +%Y%m%d_%H%M%S) \
    /rover/estimated_pose \
    /perception/detections \
    /mission/status \
    /robot/camera/image_raw
```

→ **백업 영상 + 재현 가능 데모** 보장.

---

## 6. Eval Pipeline

### 미션 결과 자동 수집

```python
# tracks/T5 (이지민)/eval/data_logger.py
class MissionLogger:
    def __init__(self, terrain_id):
        self.records = {
            "terrain_id": terrain_id,
            "start_time": time.time(),
            "minerals_collected": [],
            "phase_history": [],
            "pose_history": [],
            "covariance_history": [],
        }
    
    def log_pose(self, pose, cov):
        self.records["pose_history"].append({
            "t": time.time(),
            "pose": pose.tolist(),
            "cov": cov.tolist(),
        })
    
    def log_mineral_collected(self, mineral_id, value):
        self.records["minerals_collected"].append({
            "id": mineral_id,
            "value": value,
            "t": time.time(),
        })
    
    def save(self, path):
        with open(path, 'w') as f:
            json.dump(self.records, f, indent=2)
```

### 차트 자동 생성

```python
# tracks/T5 (이지민)/eval/charts.py
def plot_mission_summary(records, output_path):
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # 1. Mission 완료 시간
    axes[0, 0].bar(...)
    
    # 2. 광물 수집 누적
    axes[0, 1].plot(...)
    
    # 3. Pose 정확도 (estimated vs GT)
    axes[1, 0].plot(...)
    
    # 4. Coverage 진행률
    axes[1, 1].plot(...)
    
    plt.savefig(output_path, dpi=200)
```

---

## 7. Mars Physics Tier 2

### Friction Zone 적용

T1 (김현중)의 meta.json에 `physics_zones` 정의됨. T5 (이지민)가 PhysX에 적용:

```python
from isaaclab.sim import RigidBodyMaterialCfg

def apply_friction_zones(env, physics_zones):
    """각 zone에 다른 마찰 적용"""
    for zone in physics_zones:
        material = RigidBodyMaterialCfg(
            static_friction=zone["static_friction"],
            dynamic_friction=zone["dynamic_friction"],
            restitution=0.1,
        )
        
        # zone의 polygon 영역에 material 적용
        # (Isaac Sim API 사용)
        apply_to_terrain_region(env, zone["polygon"], material)
```

### 검증 실험

```
Earth (gravity 9.81, friction 0.5) vs Mars (gravity 3.72, friction zone)
   ├ 같은 정책 두 환경에서 실행
   ├ 휠 슬립률 측정
   ├ 미션 성공률 비교
   └ 발표용 차트
```

---

## 8. 인터페이스

### Consume (입력)

| 데이터 | Source |
|--------|--------|
| Isaac Sim IMU | `env.scene["imu"].data` |
| Wheel joint vel | `env.scene["robot"].data.joint_vel` |
| Sun direction | `env.scene["sphere_light"].pos` |
| RayCaster heightmap | `env.scene["height_scanner"].data.ray_hits_w` |
| **I1** heightmap.npy | T1 (김현중) 산출물 (TRN용) |

### Produce (출력)

| 인터페이스 | Consumer | 빈도 |
|----------|:--------:|:----:|
| **I5** /rover/estimated_pose | T3 (이찬휘), T4 (성선규) | 30 Hz |
| (deferred) /rover/pose_sources | T4 (성선규) UI | 30 Hz |
| (internal) eval CSV | T5 (이지민) Eval | 매 미션 종료 시 |

→ 상세는 [interfaces/INTERFACE_CONTRACTS.md](../interfaces/INTERFACE_CONTRACTS.md).

---

## 9. 일정과 마일스톤

```
Day 1 (화)
  □ Mock 1차 stub: GT + 가우시안 노이즈
  □ ROS2 publish (/rover/estimated_pose) 시작
  □ T1 (김현중) heightmap.npy 형식 확인 (T1 (김현중)과 sync)
  → EOD: T3 (이찬휘) 사용 가능 (stub 수준)

Day 2 (수)
  □ Wheel Odometry 모듈
  □ IMU integration 모듈
  □ Sun Yaw 모듈
  □ ROS2 launch 파일 골격

Day 3 (목) ⭐ TRN 핵심
  □ tracks/T5 (이지민)/trn.py 구현
  □ 합성 데이터로 검증 (gt pose에 노이즈 주고 TRN으로 보정)
  □ correlation_threshold 튜닝
  → EOD: TRN 단독 검증

Day 4 (금) EKF 통합
  □ 4개 센서 → EKF 융합
  □ TRN 가끔 보정
  □ T3와 첫 통합 (Day 4 sync 미팅)
  → EOD: Isaac Sim 환경에서 동작

Day 5 (토)
  □ 노이즈 sweep 실험 (다양한 σ에서 성공률)
  □ Mars Physics Tier 2 friction zone 적용

Day 6 (일) — 폴리싱
  □ Eval pipeline (CSV → 차트)
  □ rosbag 녹화 도구
  → 일요일 EOD ⚠️ 게이트: end-to-end 데모

Day 7 (월)
  □ 발표용 시각화 자료
  □ Earth vs Mars 비교 차트

Day 8 AM (수)
```

---

## 10. 흔한 함정

| 함정 | 증상 | 대응 |
|------|------|------|
| **TRN correlation 너무 자주 호출** | 시뮬 느림 | 5초마다 (150 step) |
| **TRN 평탄 지역에서 매칭 실패** | confidence 낮음 | threshold 0.7 미만이면 EKF에 반영 안 함 |
| **EKF 발산** | 공분산 폭발 | Q, R 매트릭스 보수적으로 시작 |
| **IMU만으로 position 적분** | 드리프트 폭발 | 자이로만 사용, 위치는 Wheel/TRN에 의존 |
| **heightmap 좌표계 혼동** | TRN 매칭 좌표 엉뚱 | T1 (김현중)과 origin/resolution 합의 |
| **/rover/estimated_pose 발행 멈춤** | T3 (이찬휘) 멈춤 | watchdog 모니터, 항상 publish |
| **Mars Tier 2가 PPO 깨트림** | PPO 잘 안 따라감 | friction 너무 낮게 잡지 말 것 (0.3 이상) |

---

## 11. DoD

### 최소 (Day 6 EOD)
- ✅ Mock 단순 stub 동작 (Day 1)
- ✅ Wheel/IMU/Sun 융합 (Day 4)
- ✅ TRN 단독 검증 (Day 3)
- ✅ ROS2 launch 파일 동작
- ✅ T3와 통합 (Day 4-5)
- ✅ end-to-end 데모에서 estimated_pose 사용

### 권장 (Day 7-8)
- ✅ 노이즈 σ별 성공률 차트
- ✅ Earth vs Mars 비교 차트
- ✅ rosbag 백업 시스템

### Stretch
- ⏳ Real V-SLAM (Isaac ROS VSLAM)
- ⏳ Pose source 분리 UI (I6)
- ⏳ Loop closure 시뮬

---

## 🤝 다른 트랙과 동기화

- **Day 1 T1 (김현중)과 합의**: heightmap.npy 형식, origin, resolution
- **Day 4 T3와 통합 미팅**: estimated_pose subscribe 확인
- **매일 18:00 DIST**: PM이 통합 테스트

---

## 💪 한 마디

이 트랙이 우리 프로젝트의 학술적 깊이를 만듭니다. **TRN은 실제 Perseverance가 쓰는 진짜 기법**. 발표에서 자랑할 수 있는 핵심 영역.

Day 3 TRN 단독 검증이 가장 중요한 마일스톤. 거기 통과하면 8일 안 완성 가능.

화이팅 📍
