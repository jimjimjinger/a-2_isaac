# ⚠️ Risk Register — 주 2회 갱신

> 월/목 09:30 standup 후 PM이 검토.
> 새 위험 발견 시 즉시 추가.

## 분류

- **Impact**: 🔴 High (발표 미스 가능) / 🟡 Medium (트랙 1개 지연) / 🟢 Low (보완 가능)
- **Probability**: 🔴 Likely / 🟡 Possible / 🟢 Unlikely
- **Status**: 🆕 New / 🔄 Active / ✅ Resolved / ❌ Realized

---

## 🔴 Active Risks (현재 활성)

| # | 리스크 | Impact | Prob | 담당 | 대응 | 발견일 |
|---|--------|:------:|:----:|:----:|------|:------:|
| R001 | M0609 USD asset 호환 안 됨 | 🔴 | 🟡 | T2 | Day 1 spike, 실패 시 단순 매니퓰레이터 직접 작성 | Day 0 |
| R002 | T1↔T5 heightmap.npy 좌표계 혼동 | 🟡 | 🟡 | T1+T5 | Day 1 인터페이스 합의에서 명시, sed로 통일 | Day 0 |
| R003 | T3 시니어가 PPO 정책 이해 못함 | 🟡 | 🟢 | T3 | T3 BRIEF에 클론 파일 참조 명시, 첫 spike Day 1 | Day 0 |
| R004 | TRN correlation이 평탄 지역에서 실패 | 🟡 | 🟡 | T5 | confidence < 0.7이면 EKF 보정 적용 안 함 | Day 0 |
| R005 | 5060 (8GB)로 Isaac Sim 검증 어려움 | 🟢 | 🟡 | T1 | 검증은 5080 owner들과 협력 | Day 0 |

---

## 🟡 Watch List (감시)

| # | 잠재 리스크 | 트리거 |
|---|-----------|--------|
| W001 | T4 PM 시간이 코딩 잠식 | PM 시간 > 2h/day 측정 시 |
| W002 | 인터페이스 변경 누적 | Day 2 이후 변경 발생 시 |
| W003 | DIST 매일 깨짐 | 3일 연속 실패 시 통합 일정 재조정 |
| W004 | 트랙 owner 번아웃 | 1:1 sync 시 침묵 / 부정적 신호 |

---

## ✅ Resolved Risks

(해결된 리스크는 여기로 이동, 학습 기록)

---

## ❌ Realized Risks

(실제 발생한 리스크 — 회고용)

---

## 📋 대응 액션 템플릿

리스크 발견 시 다음 항목 작성:

```markdown
| RXXX | 리스크 설명 | 🔴/🟡/🟢 | 🔴/🟡/🟢 | 담당 | 대응 방안 | 발견일 |
```

대응 방안 작성 가이드:
- **회피**: 리스크가 발생하지 않게 사전 차단
- **완화**: 발생해도 영향 최소화
- **수용**: 인지하고 계획에 반영
- **전가**: 다른 트랙/도구로 위임

## 매주 검토

- **월요일 09:30** 후: 신규 리스크 추가 + Active 갱신
- **목요일 09:30** 후: Watch List → Active 승격 여부 결정
