# 🎯 Day 1 Kickoff Meeting — Agenda (90분)

> 화 5/19 오전 진행. 모든 트랙 owner 참석.
> 회의 끝나면 모든 트랙이 작업 시작 가능 상태.

## 준비물 (PM이 사전 준비)

- [ ] [project_overview_flowchart.svg](../project_overview_flowchart.svg) 화면 공유 준비
- [ ] [system_architecture_full.svg](../system_architecture_full.svg) 화면 공유 준비
- [ ] [interfaces/INTERFACE_CONTRACTS.md](../interfaces/INTERFACE_CONTRACTS.md) 인쇄 or 화면 공유
- [ ] 각 트랙의 BRIEF.md 사전 공유 (회의 전 1시간 읽기 요청)
- [ ] [pm_tools/DAILY_STATUS.md](DAILY_STATUS.md) 초안 준비

## Agenda

### Part 1: 컨셉 + 동기 (10분)
```
□ 한 줄 컨셉: "화성 광물 자율 수집 로버 시뮬레이션"
□ 왜 이걸? (시뮬레이터의 본질적 가치)
□ project_overview_flowchart.svg 공유 — 미션 시나리오 4 phase
□ 클론 / 물류창고 팀 대비 차별화 3개 어필
```

### Part 2: 시스템 아키텍처 (15분)
```
□ system_architecture_full.svg 공유
□ 5 트랙 × 5 인터페이스 한눈에
□ ML이 들어가는 영역 (T3 PPO 하나)
□ TRN 컨셉 간단 (T5 (이지민)의 핵심)
```

### Part 3: 트랙 배정 + 책임 (10분)
```
□ T1 Environment        담당자: 김현중  (5060)
□ T2 Perception+M0609   담당자: 최진우  (5080)
□ T3 Driving            담당자: 이찬휘  (5080)
□ T4 Integration+PM     담당자: 성선규 (사용자 본인, 5070 Ti)
□ T5 Localization+Infra 담당자: 이지민  (5080)

각 담당자에게:
- 본인 트랙의 BRIEF.md 위치 안내
- CLAUDE.md 자동 로드 설명
- Day 1 작업 시작 가이드
```

### Part 4: 5개 인터페이스 합의 (40분)
```
INTERFACE_CONTRACTS.md 함께 검토.

□ [10분] I1 — Terrain meta.json 스키마
  - 김현중(T1)이 30~50개 batch 생성, 각 디렉터리 5개 파일
  - heightmap.npy 좌표계 (김현중↔이지민 합의 critical)
  - 광물 색 사양 (김현중↔최진우 합의)
  
□ [10분] I2 — /perception/detections
  - HSV detection 결과 형식 (최진우 담당)
  - value_score 분포 (blue 10 / red 25 / yellow 50)
  
□ [10분] I3 + I4 — /mission/pick_request, /mission/pick_response
  - request_id 중복 방지 (이찬휘 ↔ 최진우)
  - status: success / failed_grasp / timeout / no_object
  
□ [10분] I5 — /rover/estimated_pose (TRN 기반)
  - PoseWithCovarianceStamped 표준
  - 이지민(T5)은 단순 stub부터 점진 TRN
  - 이찬휘(T3)는 PoseProvider 추상화로 분리

✅ 합의 시 5명 사인 (INTERFACE_CONTRACTS.md 사인란)
✅ 변경 freeze 시작
```

### Part 5: 일정 + 게이트 (10분)
```
주요 게이트 공유:
□ Day 2 EOD: 각 트랙 hello-world 동작
□ Day 5 EOD: End-to-end 미션 1회 성공
□ Day 6 EOD: demo-stable-v1 git tag
□ Day 8 정오: Final freeze
□ Day 9 목: 발표

매일:
- 09:30 standup (15분 룰)
- 18:00 DIST 자동
```

### Part 6: 의사결정 매트릭스 (5분)
```
PM 결정 vs 트랙 owner 자율 vs 회의 필요 명시.

| 결정 | 누가 |
|------|------|
| 인터페이스 변경 | PM 승인 |
| 트랙 내부 구현 | owner 자율 |
| 신규 기능 (Day 4+) | PM 거부 기본 |
| 시연 시나리오 | PM 결정 |
```

## 회의 후 즉시

```
□ git/Notion 셋업
□ DAILY_STATUS.md 첫 갱신
□ Day 1 작업 시작 ping (각 트랙)
□ 이지민(T5)과 1:1 (시간 가용성 확인, 옵션 결정)
```

## 결정 사항 기록 (회의 중)

- 결정 사항이 발생하면 즉시 [DECISIONS.md](DECISIONS.md)에 추가.
- 인터페이스 합의 사항 → INTERFACE_CONTRACTS.md CHANGELOG.

---

## 종료 체크리스트

회의 끝나기 전 확인:

- [ ] 모든 트랙 담당자 확정
- [ ] 5개 인터페이스 사인 완료
- [ ] 각 트랙이 Day 1 EOD 목표 명확히 인지
- [ ] DAILY_STATUS 시작
- [ ] 다음 standup 시간 (내일 09:30) 확정
