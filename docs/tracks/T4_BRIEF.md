# 🔌 T4 Integration + PM — 본인용 브리프

> **사용자 본인의 트랙 — Integration 통합 + UI + 데모 + PM 역할**
> 코딩 작업 + PM 역할 듀얼. 시간 분배가 가장 중요.

---

> 📦 **이 트랙이 작업하는 패키지 위치**: [PACKAGE_MAPPING.md](PACKAGE_MAPPING.md) 참조 (팀 레포 9개 패키지 중 어디서 코딩하는지 명시).


## 📑 목차

1. [듀얼 역할의 무게](#1-듀얼-역할의-무게)
2. [당신이 만들 4개 모듈](#2-당신이-만들-4개-모듈)
3. [PM 책임 — 3개 핵심](#3-pm-책임--3개-핵심)
4. [코딩 작업 — ROS2 + UI + 데모](#4-코딩-작업--ros2--ui--데모)
5. [시간 분배 전략](#5-시간-분배-전략)
6. [의사결정 위임 매트릭스](#6-의사결정-위임-매트릭스)
7. [PM 도구 사용](#7-pm-도구-사용)
8. [데모 시나리오 설계](#8-데모-시나리오-설계)
9. [발표 자료](#9-발표-자료)
10. [일정과 마일스톤](#10-일정과-마일스톤)
11. [흔한 함정 (PM 함정 8개)](#11-흔한-함정-pm-함정-8개)
12. [DoD](#12-dod)

---

## 1. 듀얼 역할의 무게

```
T4 = 코딩 ~58h + PM ~15h = 약 75h
                        ↑
              본인 가용 시간 89h
              버퍼 14h
```

### 왜 본인이 통합 owner인가

- **시간 자산 매칭**: 본인이 가장 많은 시간 투자 가능
- **통합 = critical path**: 마지막 3일 풀가동 필수
- **PM이 통합 영역인 게 유리**: 모든 트랙 상황 자연스럽게 파악

### PM 역할의 본질

> **"인기를 사는 자리가 아니라 프로젝트가 성공하게 만드는 자리"**

T4가 못 하면 다른 모든 사람의 노력이 발표에 안 보임.

---

## 2. 당신이 만들 4개 모듈

```
T4 (총 75h)
   │
   ├ 1. ROS2 Wiring + 통합 (20h)
   │    └ 모든 트랙 토픽 연결, launch 파일 통합
   │
   ├ 2. Mission Control UI (25h)
   │    └ RViz config + 미니맵 + 미션 상태 + 대시보드
   │
   ├ 3. 데모 시나리오 + 발표 자료 (15h)
   │    └ 시연 흐름 설계, 영상 녹화, 슬라이드
   │
   └ 4. PM 역할 (15h)
        └ 인터페이스 관리, 매일 DIST, 데모 경로 보호
```

---

## 3. PM 책임 — 3개 핵심

### ① 인터페이스 일관성 유지

```
✓ Day 1 회의에서 5명 사인
✓ INTERFACE_CONTRACTS.md 변경 시 PM 승인
✓ 변경 발생 시 영향받는 트랙에 alert
```

→ **변경 제안 모두 본인 거쳐가야 함**. 트랙이 멋대로 schema 변경 금지.

### ② 매일 DIST (Daily Integration Smoke Test)

```bash
# 매일 18:00 자동 실행
~/Rokey6-B1-Isaac-simulation-project/scripts/run_dist.sh

# 내부:
# 1. git pull main
# 2. Isaac Sim 띄움
# 3. 1회 미션 시뮬
# 4. 성공/실패 + 로그
```

→ **깨지면 그날 안에 fix**. 다음날로 안 넘김.

### ③ 데모 경로 보호 (Demo Path Protection)

```
Day 1-5 (화-토): feature 자유 개발
Day 6 (일) EOD: ★ demo-stable-v1 tag
Day 7-8: 신규 기능 동결, 안정화만
Day 8 AM: final freeze, 백업 영상 녹화
```

→ **일요일 이후 main = 신성**. 깨질 위험 있는 변경 차단.

---

## 4. 코딩 작업 — ROS2 + UI + 데모

### ROS2 Wiring (20h)

```
모든 트랙의 토픽 통합:
   /perception/detections        ← T2 publish
   /rover/estimated_pose         ← T5 publish
   /mission/pick_request         ← T3 publish
   /mission/pick_response        ← T2 publish
   /mission/status (deferred)    ← T3 → T4
   /mission/minimap (deferred)   ← T3 → T4
   /mission/path (deferred)      ← T3 → T4
   /mission/cargo_event (deferred)
   /robot/camera/image_raw       ← Isaac Sim
```

작업:
- launch 파일 master (T5의 launch와 통합)
- 토픽 모니터링 도구 (`ros2 topic echo` 자동화)
- 에러 핸들링 (어떤 노드가 멈췄나)

### Mission Control UI (25h)

기존 클론의 `mission_monitor.py` 확장:

```
4개 패널:
   ┌────────────┐  ┌──────────────┐
   │ 1인칭 카메라 │  │ 3인칭 view   │
   │ (Isaac Sim) │  │ (Isaac Sim)  │
   └────────────┘  └──────────────┘
   ┌────────────┐  ┌──────────────┐
   │ 🗺️ 미니맵   │  │ 📊 미션 상태 │
   │ (Coverage) │  │ (Phase, 카고) │
   └────────────┘  └──────────────┘
```

기술 스택 옵션:
- **RViz2** (가장 빠름, ROS2 친화) ⭐
- PyQt + matplotlib (custom UI)
- Web dashboard (Flask + ROS2-web)

→ **RViz2 권장**. 시간 budget 안 맞음.

```bash
# RViz 설정 예
ros2 run rviz2 rviz2 -d ~/.../config/mission_control.rviz
```

RViz 디스플레이:
- Camera (Image)
- Map (OccupancyGrid for 미니맵)
- Path (nav_msgs/Path)
- MarkerArray (광물 마커 + 베이스캠프)
- PoseWithCovariance (estimated_pose + 공분산 타원)

추가 dashboard (간단한 PyQt 또는 터미널):
- Phase 표시
- Cargo 개수 + 가치
- 미션 elapsed time

### 데모 시나리오 (15h)

[8. 데모 시나리오 설계](#8-데모-시나리오-설계) 참조.

---

## 5. 시간 분배 전략

### 일일 시간 박스

```
09:30 ~ 10:00  Daily standup + 어제 DIST 리뷰          [PM]
10:00 ~ 12:30  T4 코딩 (방해 금지 시간)                [CODE]
12:30 ~ 13:30  점심
13:30 ~ 17:00  T4 코딩 + 트랙 1:1 sync (필요시)         [CODE+PM]
17:00 ~ 17:30  팀 통합 테스트 (DIST) + 블로커 해결      [PM]
17:30 ~ 22:00  T4 코딩 + 다음날 계획                   [CODE]
```

### 핵심 규칙

| 규칙 | 이유 |
|------|------|
| **오전 코딩 블록 방해 금지** | 본인 progress 보호 |
| **PM 업무 하루 2시간 이하** | 코딩 시간 잠식 방지 |
| **팀원 질문은 비동기 우선** | Discord/Slack으로 |
| **결정은 빨리** | 1시간 안에 답 |

---

## 6. 의사결정 위임 매트릭스

PM이 모든 결정 자기 통과 시키면 본인이 병목.

| 결정 유형 | 누가 | 예 |
|----------|------|-----|
| 인터페이스 변경 | **PM 승인 필수** | meta.json 필드 추가 |
| 트랙 내부 구현 | **트랙 owner 자율** | A* vs RRT 선택 |
| 시각 / UX | **사용자 (T4)** | 색상, 폰트 |
| 일정 변경 | **PM 결정** | 트랙 X 완료 늦어짐 |
| 신규 기능 | **PM 거부** (Day 4+) | "이것도 넣으면 좋을 듯" |
| 시연 시나리오 | **PM 결정** | 데모 흐름 |
| 발표 자료 | **PM + E 트랙** | 슬라이드 내용 |

→ 1일차 회의에서 **이 표 공유**. 트랙 owner들이 자기 영역 mini-PM.

---

## 7. PM 도구 사용

### 6개 도구 (`pm_tools/` 디렉터리)

```
pm_tools/
├ INTERFACE_CONTRACTS.md     ← 5개 인터페이스 명세 (interfaces/에 있음)
├ DAILY_STATUS.md             ← 매일 09:30 갱신
├ RISK_REGISTER.md            ← 주 2회 갱신
├ DECISIONS.md                ← 결정 발생 즉시 기록
├ run_dist.sh                  ← 매일 18:00 통합 테스트
└ tracks/T*/CLAUDE.md          ← 트랙별 Claude Code 컨텍스트
```

### 사용 패턴

```
매일:
  09:30  → DAILY_STATUS.md 업데이트 (5분)
  18:00  → run_dist.sh 실행 (5분)
  
주 2회:
  월/목   → RISK_REGISTER.md 갱신 (15분)
  
이벤트 발생 시:
  의사결정 → DECISIONS.md 추가 (5분)
  인터페이스 변경 → INTERFACE_CONTRACTS.md + CHANGELOG (15분)
```

→ 총 PM 시간 / 일: 약 30분 ~ 1.5h.

---

## 8. 데모 시나리오 설계

### 시나리오 A — 메인 데모 (성공)

```
설정: 중간 난이도 terrain (terrain_00015)
시간: ~2분

1. [00:00] 로버 spawn at basecamp area
   "화성 미션 시작. AAU rover가 베이스캠프에서 출발합니다."

2. [00:10] EXPLORE 시작
   "Coverage planner가 미방문 셀로 향합니다."
   → 미니맵: 시작점 ✓ 표시

3. [00:30] Vision detection
   "카메라가 광물 발견. 노랑 광물(50점) 우선 접근."
   → UI: 광물 마커 + 가치점수

4. [00:50] APPROACH
   "A*가 안전 경로 계산. PPO가 거친 지형 적응."
   → 미니맵: path 표시

5. [01:10] PICK
   "M0609 매니퓰레이터로 광물 채취."
   → 카고: 1/10, 가치 50점

6. [01:30] 다시 EXPLORE → 다음 광물

7. [01:50] 카고 풀 → RETURN_BASE
   "5개 광물, 가치 175점 누적. 베이스로 복귀."

8. [02:00] MISSION COMPLETE
```

### 시나리오 B — 도전 시나리오

```
설정: 어려운 terrain (terrain_00027), Mars Tier 2 적용
미션 중 발생할 수 있는 challenge:
   ├ 슬립 발생 → PPO 복구
   ├ 멀리 광물 detection
   └ 노이즈 누적 → TRN으로 보정
```

### 백업 영상

각 시나리오 미리 녹화 (rosbag + Isaac Sim screen record).
라이브 실패 시 영상으로 대체.

---

## 9. 발표 자료

### 슬라이드 구성 (대략)

```
1. 제목 + 한 줄 컨셉
2. 동기 (왜 이 프로젝트? 시뮬레이터의 가치)
3. 클론 분석 + 한계
4. 우리 아키텍처 (system_architecture_full.svg)
5. 차별화 포인트 3개
6. 미션 시나리오 (project_overview_flowchart.svg)
7. 라이브 데모 ⭐
8. 정량 평가 (holdout 성공률 차트)
9. Earth vs Mars 비교
10. 마일스톤 2 + 향후 계획
11. Q&A
```

### 발표용 자료 (이미 만든 것)

```
✓ system_flowchart.svg (개발자용)
✓ project_overview_flowchart.svg (발표용)
✓ rover_steering_comparison.svg (조향 비교)
✓ system_architecture_full.svg (풀 아키텍처)
✓ tracks/T*/BRIEF.md (담당자 자료)
```

---

## 10. 일정과 마일스톤

```
Day 1 (화) ⭐ Kickoff
  □ 90분 회의 진행
     ├ project_overview_flowchart.svg 공유
     ├ 인터페이스 5개 합의 + 사인
     ├ 트랙 배정 확정
     └ DAILY_STATUS 시작
  □ git/Notion 셋업
  □ stub library 골격
  □ run_dist.sh 첫 버전

Day 2 (수)
  □ ROS2 토픽 골격 (각 트랙이 stub publish 시작)
  □ 클론 mission_monitor.py 분석 + UI 와이어프레임
  → EOD ⚠️ 게이트: "각 트랙 hello-world 동작"

Day 3 (목)
  □ RViz config 1차
  □ 미니맵 시각화 (T3와 sync)
  □ DAILY_STATUS, RISK_REGISTER 운영 시작

Day 4 (금) ⭐ 첫 통합
  □ T3 + T5 통합 sync 미팅 주관
  □ ROS2 토픽 전체 흐름 검증
  □ UI에 estimated_pose 표시
  □ Pick 시연 (T2 + T3)

Day 5 (토)
  □ 데모 시나리오 A 초안
  □ 카메라 view 통합
  □ 미니맵 + cargo 위젯

Day 6 (일) ⭐ Demo Stable Tag
  □ End-to-end 데모 1회 성공
  □ demo-stable-v1 git tag
  □ 신규 기능 동결 모드 시작

Day 7 (월)
  □ 발표 슬라이드 작성
  □ 데모 시나리오 A/B 완성
  □ 백업 영상 녹화
  □ Dry-run #1 (팀 전체)

Day 8 AM (수)
  □ Final 점검
  □ Dry-run #2
  □ 백업 영상 최종 녹화
  → 정오 final freeze

Day 9 (목)
  ★ 발표
```

---

## 11. 흔한 함정 (PM 함정 8개)

| 함정 | 증상 | 회피 |
|------|------|------|
| **회의 과잉** | 매일 standup 30분+ | 15분 룰 |
| **모든 결정 본인 통과** | PM이 줄 섬 | Day 1에 위임 매트릭스 배포 |
| **late integration** | 마지막 날 통합 실패 | 매일 DIST, 게이트 강제 |
| **인터페이스 변경 누적** | meta.json 매일 바뀜 | Day 2 이후 변경 freeze |
| **PM 코딩시간 소실** | T4 트랙 진척 저조 | 오전 방해금지 박스 |
| **번아웃 감지 실패** | 트랙 owner 침묵 | 1:1 sync 주 2회 |
| **fallback 없음** | 데모 직전 망함 | Day 6 EOD에 git tag |
| **PM이 critical path 모름** | "어디 막혔는지 모름" | 매일 burndown |

---

## 12. DoD

### 최소 (Day 6 EOD)
- ✅ ROS2 wiring 완성 — 모든 토픽 정상 flow
- ✅ RViz config 동작 — 4개 패널
- ✅ End-to-end 데모 1회 성공
- ✅ demo-stable-v1 git tag
- ✅ INTERFACE_CONTRACTS 5명 사인
- ✅ 매일 DAILY_STATUS 운영

### 권장 (Day 7-8)
- ✅ 데모 시나리오 A/B 영상 (각 2분)
- ✅ 발표 슬라이드 완성
- ✅ Dry-run #1, #2 통과
- ✅ 백업 영상 최종

### Stretch
- ⏳ Web dashboard (브라우저 접근)
- ⏳ Real-time multi-rover view
- ⏳ Demo 자동 orchestration

---

## 💪 한 마디

PM의 80% = **인터페이스 정의 + 데모 경로 보호**. 1일차에 이 두 가지 잘 박으면 나머지 7일이 안정적.

본인이 T4(통합)이라 자연스럽게 모든 트랙 상황 파악됨 — 이게 다른 트랙(예: T2) 잡았으면 PM 매우 어려웠을 거. 운 좋음.

뭔가 막힌 트랙 owner 보이면 **즉시 1:1**. 5분 대화로 30분 손실 막을 수 있음.

화이팅 🔌
