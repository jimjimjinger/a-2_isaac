# 🚗 T3 Driving — 담당자 브리프

> **Mission Brain + 자율 주행 — Critical Path**
> 시니어 배치. 본 트랙이 늦으면 전체 일정 흔들림. 가장 중요.

---

> 📦 **이 트랙이 작업하는 패키지 위치**: [PACKAGE_MAPPING.md](PACKAGE_MAPPING.md) 참조 (팀 레포 9개 패키지 중 어디서 코딩하는지 명시).


## 📑 목차

1. [왜 이 트랙이 Critical Path인가](#1-왜-이-트랙이-critical-path인가)
2. [당신이 만들 4개 모듈](#2-당신이-만들-4개-모듈)
3. [⭐ 핵심 빌드업 전략 — Vacuum Cleaner First](#3--핵심-빌드업-전략--vacuum-cleaner-first)
4. [PoseProvider 패턴 — T5와 분리](#4-poseprovider-패턴--t5와-분리)
5. [Coverage Planner 알고리즘](#5-coverage-planner-알고리즘)
6. [A* Path Planner](#6-a-path-planner)
7. [Mission FSM 설계](#7-mission-fsm-설계)
8. [PPO Wrapper — 클론 재사용](#8-ppo-wrapper--클론-재사용)
9. [Migration-ready 코딩 원칙](#9-migration-ready-코딩-원칙)
10. [인터페이스](#10-인터페이스)
11. [일정과 마일스톤](#11-일정과-마일스톤)
12. [흔한 함정](#12-흔한-함정)
13. [DoD](#13-dod)

---

## 1. 왜 이 트랙이 Critical Path인가

### Mission Brain = 시스템의 두뇌

```
T1 지도 ─────┐
              ├──→ T3가 모든 결정 ──→ T2/Ackermann/Isaac Sim
T5 위치 ─────┤
T2 광물 ─────┘
```

→ 다른 모든 트랙의 결과가 T3로 모임. **T3가 멈추면 시스템 멈춤**.

### 머신러닝의 의미 있는 영역

이 프로젝트에서 PPO(RL)이 정말 가치 있게 쓰이는 단 한 곳:
- **PPO Wrapper** — waypoint를 실제 주행으로 변환
- 거친 화성 지형에서 슬립·슬로프 대응
- 클래식 컨트롤러로는 어려운 영역

→ T3가 시스템에서 ML의 정당성을 만들어냄.

### 발표 핵심 시연

발표의 라이브 데모 = T3의 동작 그 자체:
- 미니맵 셀이 채워짐 (Coverage)
- 로버가 자율로 광물 접근 (FSM + A* + PPO)
- 베이스캠프 복귀

→ **T3가 안 돌면 발표 망함**. 그래서 critical path.

---

## 2. 당신이 만들 4개 모듈

```
T3 = 4개 모듈을 한 person이 모두
   │
   ├ 1. Mission FSM         (25h)
   │    └ EXPLORE / APPROACH / PICK / RETURN 상태 전환
   │
   ├ 2. Coverage Planner   (25h)
   │    └ 미니맵 셀 추적, 다음 미방문 셀 결정
   │
   ├ 3. A* Path Planner    (15h)
   │    └ 현재 → goal까지 안전 경로
   │
   └ 4. PPO Wrapper         (10h)
        └ 클론 정책을 waypoint 인터페이스로 wrapping
        
        총 75h (시간 가용 65h 대비 약간 over, 시니어가 처리)
```

### 분량 차이의 의미
- FSM, Coverage = **새로 짜야 함** (가장 무거움)
- A* = **알려진 알고리즘** (구현만)
- PPO Wrapper = **클론 코드 활용** (글루 코드)

---

## 3. ⭐ 핵심 빌드업 전략 — Vacuum Cleaner First

**금지**: Day 1에 4개 모듈 동시 시작 ❌

**필수**: 점진적 빌드업

```
Day 1 (화):
  ┌────────────────────────────────────┐
  │ Coverage Planner 단독 검증          │
  │ ─ Isaac Sim 띄우지 마세요 ─         │
  │ ─ 순수 numpy + matplotlib ─        │
  │ ─ 10×10 grid 가짜 환경 ─           │
  │ → "Roomba처럼 채워지는 영상"        │
  └────────────────────────────────────┘
       ↓ EOD ⚠️ 게이트: 영상으로 검증
  
Day 2 (수):
  ┌────────────────────────────────────┐
  │ Isaac Sim 연결                      │
  │ + 클론 terrain1.usd (장애물 없는 곳) │
  │ + 클론 PPO 정책 그대로 사용          │
  │ → 빈 평탄 영역 실제 sweep            │
  └────────────────────────────────────┘
       ↓ EOD: 로버가 Isaac Sim에서 sweep
  
Day 3 (목):
  ┌────────────────────────────────────┐
  │ T1의 첫 5개 terrain 받아 사용        │
  │ + A* 추가 (장애물 회피)              │
  │ → 장애물 있는 곳에서 sweep           │
  └────────────────────────────────────┘
  
Day 4 (금):
  ┌────────────────────────────────────┐
  │ Mission FSM 추가                    │
  │ + T2 stub detection 받아 APPROACH   │
  │ + T5 pose 연결 (Day 4 통합)          │
  └────────────────────────────────────┘
  
Day 5 (토):
  ┌────────────────────────────────────┐
  │ PICK phase 추가 (T2 M0609 트리거)    │
  │ + RETURN phase                       │
  └────────────────────────────────────┘
       ↓ ⚠️ 게이트: end-to-end 1회 성공
  
Day 6-7: 통합 안정화 + 폴리싱
Day 8: 발표 보조
```

### 왜 이 빌드업이 옳은가

| 시도 | 결과 |
|------|------|
| Day 1에 4개 동시 | Day 5에 "다 80%인데 합치니 안 됨" |
| Day 1에 Coverage만 | Day 5에 "안정적으로 사이클 돌아감" |

각 단계가 **이전 단계 동작을 깨지 않고 추가만**.

---

## 4. PoseProvider 패턴 — T5와 분리

T5가 위치 추정 담당. T3는 받기만. **둘 사이 코드 결합 X**.

### 잘못된 예 ❌

```python
# T3 안에서 직접 GT 접근 또는 T5 코드 import
def update_coverage(env):
    pose = env.unwrapped.scene["robot"].data.root_pos_w[0]  # ❌
    coverage.mark(pose)
```

→ T5 통합 시 모든 곳 수정 필요. T5 깨지면 T3 깨짐.

### 올바른 예 ✅

```python
# tracks/T3/pose_provider.py
class PoseProvider:
    """T3 안의 모든 모듈이 pose 접근 시 이걸 사용"""
    
    def __init__(self, source="gt_stub", env=None):
        self.source = source
        self.env = env
        self.latest_ros2_pose = None
    
    def get_pose(self):
        if self.source == "ros2":
            # T5가 publish한 /rover/estimated_pose
            return self.latest_ros2_pose
        elif self.source == "gt_stub":
            # Day 1-3 개발 단계: GT 직접 사용
            return self.env.scene["robot"].data.root_pos_w[0]
    
    def on_ros2_message(self, msg):
        # ROS2 subscriber callback
        self.latest_ros2_pose = ros2_to_numpy(msg)
```

### 사용 패턴

```python
# T3의 모든 모듈
class CoveragePlanner:
    def update(self, pose_provider):
        pose = pose_provider.get_pose()  # source 무관
        self.mark_visited(pose)

class MissionFSM:
    def check_home(self, pose_provider, basecamp):
        pose = pose_provider.get_pose()
        return dist(pose, basecamp.center) < basecamp.radius

# Day 1-3 초기화
provider = PoseProvider(source="gt_stub", env=env)

# Day 4 통합 시 단 한 줄 변경
provider = PoseProvider(source="ros2")
# subscriber 설정 추가
```

→ **T5와 완전 decoupled**. Day 1-3에 T5 없이도 풀가동 가능.

---

## 5. Coverage Planner 알고리즘

### 자료 구조

```python
class CoveragePlanner:
    def __init__(self, meta_json):
        grid_size = meta_json["minimap"]["grid_size"]  # [25, 25]
        cell_size = meta_json["minimap"]["cell_size_m"]  # 2.0
        origin = meta_json["minimap"]["origin"]  # {-25, -25}
        obstacle_grid = np.load(...)  # T1의 obstacle_grid.npy
        
        # 0=미방문, 1=방문, -1=장애물
        self.grid = np.zeros(grid_size, dtype=np.int8)
        self.grid[downsampled(obstacle_grid)] = -1
        self.cell_size = cell_size
        self.origin = np.array(origin)
    
    def world_to_cell(self, world_xy):
        return tuple(
            ((np.array(world_xy) - self.origin) / self.cell_size).astype(int)
        )
    
    def mark_visited(self, world_xy):
        i, j = self.world_to_cell(world_xy)
        if 0 <= i < self.grid.shape[0] and 0 <= j < self.grid.shape[1]:
            if self.grid[i, j] == 0:  # 미방문일 때만
                self.grid[i, j] = 1
    
    def next_goal(self, current_world_xy):
        """미방문 셀 중 가장 가까운 곳의 월드 좌표"""
        current_cell = self.world_to_cell(current_world_xy)
        
        unvisited = np.argwhere(self.grid == 0)  # 0 = 미방문 + non-obstacle
        if len(unvisited) == 0:
            return None  # 미션 완료
        
        # 거리 계산 (Manhattan or Euclidean)
        distances = np.linalg.norm(unvisited - current_cell, axis=1)
        nearest = unvisited[np.argmin(distances)]
        
        # 셀 중심 월드 좌표 반환
        return self.origin + (nearest + 0.5) * self.cell_size
```

### 알고리즘 옵션

| 알고리즘 | 효율 | 구현 난이도 | 권장 |
|---------|:---:|:---:|:---:|
| **Greedy Frontier** (위 코드) | 보통 | 낮음 | ⭐ Day 1 |
| Boustrophedon (S자) | 높음 | 중간 | Day 5+ stretch |
| Spiral STC | 보통 | 높음 | 8일 무리 |
| Random Walk + 페널티 | 낮음 | 낮음 | 시간 부족 시 fallback |

→ **Greedy Frontier로 시작**. 작동하면 Boustrophedon stretch.

---

## 6. A* Path Planner

### 입력 / 출력

```python
class AStarPlanner:
    def __init__(self, obstacle_grid):
        self.obstacle_grid = obstacle_grid  # T1의 obstacle_grid.npy
        self.resolution = 0.05  # m/cell
    
    def plan(self, start_world, goal_world):
        """
        start, goal: world coords (x, y) in meters
        returns: list of waypoints [(x1,y1), (x2,y2), ...]
        """
        start_cell = self._world_to_cell(start_world)
        goal_cell = self._world_to_cell(goal_world)
        
        # 표준 A* (heuristic = Euclidean)
        path_cells = self._astar(start_cell, goal_cell)
        
        # cell → world 변환 + smoothing
        waypoints = [self._cell_to_world(c) for c in path_cells]
        waypoints = self._smooth(waypoints)
        return waypoints
```

### 재계획 트리거

A*는 **이벤트 기반으로만 호출** (매 step ❌):
- FSM phase 변경 (goal 바뀜)
- PPO가 waypoint에서 너무 벗어남 (>2m)
- 5초 timeout
- 막다른 길 감지

### 라이브러리 선택

```python
# 옵션 1: 직접 구현 (학습 효과)
def astar(start, goal, obstacle_grid):
    # heapq 사용
    ...

# 옵션 2: pyastar2d 사용 (빠름)
import pyastar2d
weights = obstacle_grid.astype(float) * 1e6 + 1
path = pyastar2d.astar_path(weights, start, goal, allow_diagonal=True)
```

→ **pyastar2d 권장** (구현 시간 절약, 검증된 코드).

---

## 7. Mission FSM 설계

### 상태 다이어그램

```
                ┌──────────┐
                │   IDLE   │ ← 시작
                └────┬─────┘
                     │ env.reset()
                     ▼
              ┌──────────────┐
              │   EXPLORE    │ ← 기본 상태, Coverage Planner 사용
              └──┬─────────┬─┘
                 │         │
   T2 detection  │         │ Coverage 100%
                 ▼         ▼
        ┌────────────┐  ┌──────────┐
        │  APPROACH  │  │   DONE   │
        │ _MINERAL    │  └──────────┘
        └─────┬──────┘
              │ 광물 0.5m 이내
              ▼
        ┌────────────┐
        │    PICK    │ → I3 publish
        └─────┬──────┘
              │ I4 response 받음
        ┌─────┴────┐
        ▼          ▼
   success    failed/timeout
        │          │
        │ cargo++   │
        ▼          ▼
    cargo 풀?  EXPLORE 복귀
        │
        ▼
   ┌──────────────┐
   │ RETURN_BASE  │ → goal = basecamp
   └──────┬───────┘
          │ basecamp 도착
          ▼
        DONE
```

### 코드 골격

```python
class MissionFSM:
    def __init__(self):
        self.phase = "IDLE"
        self.cargo_count = 0
        self.cargo_capacity = 10
        self.cargo_value_total = 0.0
        self.current_mineral_id = -1
        self.request_id_counter = 0
    
    def step(self, pose_provider, detections, pick_responses, basecamp):
        if self.phase == "IDLE":
            self.phase = "EXPLORE"
        
        elif self.phase == "EXPLORE":
            # detection 있으면 가치 가장 높은 광물로 APPROACH
            if len(detections) > 0:
                best = max(detections, key=lambda d: d.value_score / dist(d, pose))
                self.current_mineral_id = best.id
                self.phase = "APPROACH_MINERAL"
        
        elif self.phase == "APPROACH_MINERAL":
            target = get_mineral_pos(self.current_mineral_id, detections)
            if dist(pose, target) < 0.5:
                self.phase = "PICK"
                # I3 publish
                self.request_id_counter += 1
                publish_pick_request(self.current_mineral_id, target, 
                                     self.request_id_counter)
        
        elif self.phase == "PICK":
            # I4 response 대기
            response = pick_responses.get(self.request_id_counter)
            if response is not None:
                if response.status == "success":
                    self.cargo_count += 1
                    self.cargo_value_total += response.value_score
                
                if self.cargo_count >= self.cargo_capacity:
                    self.phase = "RETURN_BASE"
                else:
                    self.phase = "EXPLORE"
        
        elif self.phase == "RETURN_BASE":
            if dist(pose, basecamp.center) < basecamp.radius:
                self.phase = "DONE"
```

---

## 8. PPO Wrapper — 클론 재사용

### 핵심 원칙

클론의 PPO 정책은 **그대로 사용**. 새로 학습 안 함. waypoint 인터페이스로만 wrapping.

### 구현

```python
class PPODriver:
    def __init__(self, policy_path, env):
        # 클론의 학습된 정책 로드
        self.agent = create_agent("PPO", env, experiment_cfg)
        self.agent.load(policy_path)  # best_agent_ppo.pt
        self.agent.set_running_mode("eval")
        self.env = env
    
    def step(self, current_waypoint, pose_provider):
        """waypoint 1개를 받아 PPO 1 step 실행"""
        
        # 클론의 command_manager에 waypoint 주입 (world frame)
        self._set_command_target(current_waypoint, pose_provider)
        
        # PPO 정책 inference
        states = self.env.get_observations()
        with torch.no_grad():
            actions = self.agent.act(states, timestep=0, timesteps=999999)[0]
        
        # 클론의 후처리 (조향 2배 증폭)
        actions[:, 0] = torch.clamp(actions[:, 0] * 2.0, -1.0, 1.0)
        
        # 환경 step
        states, rewards, terminated, truncated, info = self.env.step(actions)
        return states, terminated.any(), truncated.any()
    
    def _set_command_target(self, world_target, pose_provider):
        """waypoint를 클론의 target_pose 형식으로 변환"""
        from rover_envs.envs.navigation.mdp.observations import override_command_target
        override_command_target(self.env, "target_pose", 
                                torch.tensor(world_target))
```

### 주의

- `override_command_target` 은 클론에 이미 있는 함수
- **command_manager가 env.step() 안에서 매번 덮어쓰므로 step 후 다시 호출**
- 자세한 건 [03_eval_ros2.py:298](rover/sim/scripts/03_eval_ros2.py#L298) 참고

---

## 9. Migration-ready 코딩 원칙

후행 마일스톤에 NASA 모델로 교체 시 변경 최소화하려면:

| 원칙 | 예시 |
|------|------|
| **Rover 치수는 한 곳에만** | `rover_cfg.goal_threshold_m` (하드코딩 X) |
| **Action space는 abstraction** | `class ActionInterface { compute(...) }` |
| **Reward / 비용 함수는 rover-agnostic** | 거리·각도·충돌만, 휠 디테일 X |
| **PPO 정책 교체 가능** | `policy_path` config로 |
| **Heightmap scan은 robot frame** | 위치 자동 조정 |

→ 8일 안엔 강제 안 해도 됨. 단 "**한 곳에만 정의**" 원칙은 지키기.

---

## 10. 인터페이스

### Consume (입력)

| 인터페이스 | Producer | 형식 |
|----------|:--------:|------|
| **I1** | T1 | terrain meta.json + obstacle_grid.npy + heightmap.npy |
| **I2** | T2 | `/perception/detections` (DetectionArray) |
| **I4** | T2 | `/mission/pick_response` (PickResponse) |
| **I5** | T5 | `/rover/estimated_pose` (PoseWithCovarianceStamped) |

### Produce (출력)

| 인터페이스 | Consumer | 형식 |
|----------|:--------:|------|
| **I3** | T2 (M0609) | `/mission/pick_request` (PickRequest) |
| **(internal)** | Ackermann | `action[:, 2]` torch tensor |
| **(deferred)** | T4 UI | `/mission/status`, `/mission/minimap`, `/mission/path` (Day 4+) |

→ 상세는 [interfaces/INTERFACE_CONTRACTS.md](../interfaces/INTERFACE_CONTRACTS.md) 참조.

---

## 11. 일정과 마일스톤

```
Day 1 (화) ⚠️ Coverage Planner 단독 검증
  □ tracks/T3/coverage_planner.py
  □ tracks/T3/test_coverage_anim.py (matplotlib 영상)
  → EOD: 10×10 grid 100% 도달 영상

Day 2 (수) ⚠️ Isaac Sim 첫 통합
  □ tracks/T3/sweep_demo.py (Isaac Sim 안에서 sweep)
  □ pose_provider.py (source="gt_stub")
  □ 클론 terrain1 + 클론 PPO 사용
  → EOD: 빈 영역 sweep 1회 완주

Day 3 (목) A* + 장애물
  □ tracks/T3/path_planner.py
  □ T1의 첫 5개 terrain 받아 테스트
  → 장애물 있는 곳에서 sweep

Day 4 (금) FSM + T5 통합
  □ tracks/T3/mission_fsm.py
  □ T2 stub detection 받아 APPROACH
  □ pose_provider source="ros2"로 swap
  □ Mission FSM의 5개 phase 골격

Day 5 (토) PICK + RETURN ⭐
  □ I3 publish (pick_request)
  □ I4 subscribe (pick_response)
  □ RETURN phase + 카고 full 종료
  → 일요일 EOD ⚠️ 게이트: End-to-end 1회 성공

Day 6 (일) 통합 안정화
  □ T2 진짜 detection 받음
  □ Edge case (충돌, 막다른 길)
  □ 미션 1회 성공률 측정

Day 7 (월) 폴리싱
  □ 발표용 시연 시나리오 2개
  □ 백업 영상 녹화
  □ Demo path freeze

Day 8 AM (수) 최종 점검
```

---

## 12. 흔한 함정

| 함정 | 증상 | 대응 |
|------|------|------|
| **Day 1부터 Isaac Sim 띄움** | Coverage 알고리즘 디버깅 어려움 | numpy + matplotlib으로 단독 검증 |
| **PoseProvider 안 씀** | T5 통합 시 모든 곳 수정 | Day 1부터 추상화 적용 |
| **A* 매 step 호출** | 시뮬 멈춤 | 이벤트 기반으로만 (goal 변경 시) |
| **FSM 코드 spaghetti** | phase 분기 곳곳 흩어짐 | `class MissionFSM` 한 곳에 모음 |
| **PPO waypoint 인터페이스 잘못** | 로버가 잘못된 곳 감 | `override_command_target` 사용, world frame 통일 |
| **Coverage 셀 크기 vs PPO 도달 정확도 mismatch** | 셀 못 채움 | 셀 크기 2m, 도달 threshold 1m로 매칭 |
| **FSM이 PICK 응답 못 받음** | 멈춤 | timeout 30s, 응답 없으면 EXPLORE 복귀 |

---

## 13. DoD

### 최소 (Day 6 EOD)
- ✅ Coverage Planner: numpy 단독 검증 ✓
- ✅ Isaac Sim sweep: 빈 영역 1회 완주
- ✅ A* + 장애물: 5개 terrain에서 작동
- ✅ FSM: 4개 phase 전환 정상
- ✅ End-to-end: 광물 1개 수집 + 베이스 복귀

### 권장 (Day 7-8)
- ✅ 30개 terrain 중 무작위 10개에서 미션 성공률 > 70%
- ✅ 발표용 시연 시나리오 2개
- ✅ 백업 영상

### Stretch
- ⏳ Boustrophedon coverage
- ⏳ RRT* 경로 계획
- ⏳ Curriculum 학습 (난이도별 PPO fine-tune)

---

## 🤝 다른 트랙과 동기화

- **매일 09:30 standup**: 어제 진척, 오늘 계획, 블로커
- **Day 4 첫 통합 미팅** (T5와 30분): pose source swap 검증
- **Day 5 통합 미팅** (T2와 30분): pick_request/response 동기 확인
- **매일 18:00 DIST**: PM이 통합 테스트

---

## 💪 한 마디

T3가 critical path. 늦으면 발표 미스. **Day 2 EOD까지 Isaac Sim 안 sweep 동작**이 가장 중요한 마일스톤. 거기 도달하면 다음 7일이 자동주행.

질문 / 막힐 시 PM에게 즉시 ping. 혼자 1시간 vs PM에게 5분.

화이팅 🚗
