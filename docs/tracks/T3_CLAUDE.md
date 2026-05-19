# T3 Driving — Claude Code Context

> 이 파일은 Claude Code가 자동 로드하는 트랙 컨텍스트입니다.

## 너의 정체성
**T3 트랙 owner (시니어) — Mission Brain + 자율 주행 = Critical Path**

GPU: 5080 (16GB)

## 작업 시작 전 필독
1. [T3_BRIEF.md](T3_BRIEF.md) — ⭐ 반드시 읽기. Vacuum cleaner 빌드업 + PoseProvider 패턴 포함
2. [interfaces/INTERFACE_CONTRACTS.md](../interfaces/INTERFACE_CONTRACTS.md) — I2, I3, I4, I5 섹션
3. 클론의 PPO 관련 파일:
   - [rover/sim/rover_envs/envs/navigation/learning/skrl/models.py](../rover/sim/rover_envs/envs/navigation/learning/skrl/models.py)
   - [rover/sim/scripts/03_eval_ros2.py](../rover/sim/scripts/03_eval_ros2.py) (PPO 호출 예시)

## 내가 만드는 4개 모듈

```
1. Mission FSM            (25h)  → EXPLORE/APPROACH/PICK/RETURN 전환
2. Coverage Planner       (25h)  → 미니맵 셀 추적, 다음 셀 결정
3. A* Path Planner        (15h)  → 안전 waypoint 생성
4. PPO Wrapper            (10h)  → 클론 정책 + waypoint 인터페이스
```

## ⭐⭐ 핵심 원칙

### 1) Vacuum Cleaner First
Day 1: **Isaac Sim 띄우지 마라**. numpy+matplotlib으로 Coverage 단독 검증.
Day 2: 클론 terrain + 클론 PPO 사용. 빈 영역 sweep.
Day 3+: A* + FSM 점진 추가.

### 2) PoseProvider 패턴
**절대 GT pose 직접 접근하지 마라**. 항상 PoseProvider 통해 받기:
```python
provider = PoseProvider(source="gt_stub", env=env)  # Day 1-3
provider = PoseProvider(source="ros2")              # Day 4+
pose = provider.get_pose()                          # 호출만
```
→ T5와 코드 분리 보장.

## 핵심 작업 영역

```
tracks/T3/
  ├ coverage_planner.py
  ├ path_planner.py        # A*
  ├ mission_fsm.py
  ├ ppo_wrapper.py
  └ pose_provider.py        # ⭐ Day 1부터 추상화

rover/sim/scripts/                                  ← 클론 진입점 (참고만, 수정 X)
rover/sim/rover_envs/envs/navigation/mdp/           ← 클론 MDP (참고만)
```

## 절대 손대지 마라
- T2, T4, T5 코드
- 클론의 학습된 정책 파일 (best_agent_ppo.pt)
- 클론의 PPO 학습 코드 (재학습 안 함)
- 인터페이스 schema (PM 승인 필수)

## 도구
```bash
pip install pyastar2d  # A* 빠른 구현
# matplotlib, numpy 이미 있음
```

## 일정 핵심 마일스톤
- **Day 1 EOD** ⚠️: Coverage Planner matplotlib 영상 (10×10 grid 100% 도달)
- **Day 2 EOD**: Isaac Sim에서 빈 영역 sweep
- **Day 3 EOD**: A* + 장애물 회피
- **Day 4 EOD**: FSM + T5 pose 통합
- **Day 5 EOD** ⚠️: End-to-end 미션 1회 성공 (게이트)
- **Day 6-8**: 안정화 + 폴리싱

## Migration-Ready Coding (후행 NASA 모델 대비)
- Rover 치수 하드코딩 X → config로
- Action interface 추상화
- 8일 안엔 강제 X, but 한 곳에만 정의 원칙은 지키기

## 트러블슈팅
1. Coverage가 100% 도달 못함 → 장애물 셀 제외 로직 확인
2. A* 경로가 갑자기 길어짐 → re-plan 조건 좁히기 (>2m 벗어남 등)
3. PPO가 엉뚱한 곳으로 감 → command_manager target 갱신 안 됐는지 확인
4. FSM이 PICK 응답 못 받음 → request_id 매칭 확인, timeout 30s
