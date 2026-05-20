# T4 (성선규) Integration + PM — Claude Code Context (사용자 본인용)

> 이 파일은 Claude Code가 자동 로드하는 본인용 컨텍스트입니다.

## 너의 정체성
**T4 (성선규) owner = 통합 + UI + 데모 + PM 역할 (듀얼)**

GPU: 5070 Ti (12GB)

## 작업 시작 전 필독
1. [T4_BRIEF.md](T4_BRIEF.md) — 듀얼 역할 전체 + 시간 분배 + 의사결정 매트릭스
2. [interfaces/INTERFACE_CONTRACTS.md](../interfaces/INTERFACE_CONTRACTS.md) — 모든 인터페이스 (PM 책임)
3. [interfaces/deferred_interfaces.md](../interfaces/deferred_interfaces.md) — I6~I10 (Day 4+ 본인 결정)
4. [pm_tools/](../pm_tools/) — PM 운영 도구 6개

## 내가 만드는 4개 모듈 (75h)

```
1. ROS2 Wiring (20h)        → 모든 트랙 토픽 통합
2. Mission Control UI (25h) → RViz + 미니맵 + 미션 상태
3. 데모 + 발표 (15h)         → 시나리오, 영상, 슬라이드
4. PM 역할 (15h)             → 인터페이스, DIST, 데모 경로 보호
```

## PM 책임 — 3개 핵심

```
① 인터페이스 일관성 유지
   - INTERFACE_CONTRACTS.md 변경 시 본인 승인
   - Day 1 사인 후 freeze
   
② 매일 DIST (Daily Integration Smoke Test)
   - 매일 18:00 pm_tools/run_dist.sh
   - 깨지면 그날 안에 fix
   
③ 데모 경로 보호
   - Day 6 EOD: demo-stable-v1 git tag
   - 이후 main = 신성
```

## 시간 박스 (필수)

```
09:30 ~ 10:00  Daily standup + DIST 리뷰        [PM]
10:00 ~ 12:30  T4 (성선규) 코딩 (방해 금지)               [CODE]
12:30 ~ 13:30  점심
13:30 ~ 17:00  T4 (성선규) 코딩 + 트랙 1:1 sync          [CODE+PM]
17:00 ~ 17:30  DIST + 블로커 해결                [PM]
17:30 ~ 22:00  T4 (성선규) 코딩 + 다음날 계획             [CODE]
```

**PM 시간 하루 2시간 이하 목표**. 넘으면 위임 부족 신호.

## 핵심 작업 영역

```
tracks/T4 (성선규)/
  ├ ros2_wiring/        # 토픽 통합, launch
  ├ ui/                 # 대시보드, RViz config
  ├ demo/               # 시나리오 스크립트
  └ presentation/       # 발표 자료

pm_tools/
  ├ DAILY_STATUS.md     # 매일 09:30 갱신
  ├ RISK_REGISTER.md    # 주 2회
  ├ DECISIONS.md        # 이벤트 기반
  └ run_dist.sh         # 매일 18:00
```

## 절대 손대지 마라 (다른 트랙)
- tracks/T1 (김현중)~T3 (이찬휘), T5 (이지민) 의 내부 코드
- 인터페이스 schema는 변경 가능하지만, 변경 시 모두에게 alert

## 도구
- rclpy (ROS2 Python)
- RViz2 (시각화 메인)
- matplotlib, PyQt (UI 보조)
- git (브랜치 관리)

## 일정 핵심
- **Day 1**: 킥오프 90분 회의 진행 + 인터페이스 사인 + git/Notion 셋업
- **Day 2 EOD** ⚠️: 각 트랙 hello-world 동작 검증 (게이트)
- **Day 3-4**: ROS2 wiring + UI 골격
- **Day 5 EOD** ⚠️: End-to-end 데모 1회 성공 (게이트)
- **Day 6 EOD**: demo-stable-v1 tag, freeze 시작
- **Day 7-8**: 발표 자료, dry-run

## 매일 체크리스트

```
☐ 09:30: DAILY_STATUS.md 갱신 (전 트랙 progress)
☐ 10:00: 본인 코딩 블록 시작
☐ 12:30: 점심
☐ 13:30: 1:1 sync (필요한 트랙만)
☐ 17:00: run_dist.sh 실행
☐ 17:30: 블로커 해결
☐ 22:00: 다음날 계획
```

## 의사결정 위임 매트릭스 (Day 1에 팀에 공유)

| 결정 | 누가 |
|------|------|
| 인터페이스 변경 | PM 승인 |
| 트랙 내부 구현 | 트랙 owner 자율 |
| 신규 기능 (Day 4+) | PM 거부 기본 |
| 시연 시나리오 | PM 결정 |
