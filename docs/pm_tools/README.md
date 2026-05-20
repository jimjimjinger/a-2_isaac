# 🛠️ PM Tools — 사용 가이드

> T4 성선규(사용자 본인)가 PM 역할 수행 시 사용하는 도구 모음.
> 1일차 킥오프 후부터 매일 운영.

## 도구 목록 (6개)

| 도구 | 용도 | 빈도 |
|------|------|:----:|
| [KICKOFF_AGENDA.md](KICKOFF_AGENDA.md) | Day 1 90분 회의 진행 가이드 | 1회 (Day 1) |
| [DAILY_STATUS.md](DAILY_STATUS.md) | 매일 트랙 진행상황 + 블로커 | 매일 09:30 |
| [RISK_REGISTER.md](RISK_REGISTER.md) | 리스크 추적 | 주 2회 (월/목) |
| [DECISIONS.md](DECISIONS.md) | 의사결정 기록 (ADR-lite) | 이벤트 기반 |
| [run_dist.sh](run_dist.sh) | Daily Integration Smoke Test | 매일 18:00 |
| [INTERFACE_CONTRACTS.md](../interfaces/INTERFACE_CONTRACTS.md) | 5개 인터페이스 명세 | Day 1 lock |

## 일일 운영 사이클

```
09:30  →  DAILY_STATUS.md 갱신 (각 트랙 어제/오늘/블로커)
10:00  →  본인 코딩 시작
12:30  →  점심
13:30  →  필요 시 트랙 1:1 sync
17:00  →  run_dist.sh 실행
17:30  →  블로커 해결
22:00  →  내일 계획
```

## 이벤트 기반 작업

- **인터페이스 변경 제안 발생** → DECISIONS.md 추가 + INTERFACE_CONTRACTS.md CHANGELOG
- **위험 발견** → RISK_REGISTER.md 추가
- **트랙 지연** → DAILY_STATUS의 블로커 섹션 + 1:1 미팅

## PM 시간 박스

매일 PM 업무에 **2시간 이하** 목표:
- 09:30 ~ 10:00 (30분, 매일)
- 17:00 ~ 17:30 (30분, 매일)
- 이벤트 발생 시 (수시)

→ 넘으면 위임 부족 신호. 위임 매트릭스 ([T4 BRIEF](../tracks/T4_BRIEF.md)) 재검토.

## 마일스톤 게이트

| 시점 | 게이트 | 도구 |
|------|--------|------|
| Day 1 12:00 | Interface 5개 사인 완료 | KICKOFF_AGENDA |
| Day 2 EOD | 각 트랙 hello-world | DAILY_STATUS + DIST |
| Day 5 EOD | End-to-end 데모 1회 | DIST |
| Day 6 EOD | `demo-stable-v1` git tag | (수동) |
| Day 8 정오 | Final freeze | (수동) |
