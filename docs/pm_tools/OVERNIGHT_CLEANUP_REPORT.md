# 🌙 Overnight Cleanup Report

> 사용자(성선규)가 자고 있는 동안 진행한 작업공간 최적화 보고서.
> 작성일: 2026-05-20

---

## 1. 완료 작업

### P0 — 시급/높은 영향 (5건)

| # | 작업 | 상태 |
|---|------|:---:|
| 1 | `.gitignore`에 `.vscode/`, `.claude/` 추가 (1.3GB 누락 방지) | ✅ |
| 2 | `Tx → 사람 이름 병기` rewrite (sed 일괄 + spot fix) — 16개 파일 | ✅ |
| 3 | `INTERFACE_CONTRACTS.md` vs `META_JSON_FIELDS.md` 중복 해소 (풀 예시 → 요약+링크) | ✅ |
| 4 | `isaac_drive/04_*_README.md` rename mismatch fix (`isaac_navigation` → `isaac_drive`) | ✅ |
| 5 | `isaac_sim/02_*_README.md` 전면 갱신 (I1 1샘플 + markers/ + scripts 반영) | ✅ |

### P1 — 패키지 README 정합 (8건)

| 패키지 | 변경 |
|--------|------|
| `isaac_bringup` | 노드 stub 상태 명시 + DIST 연계 |
| `isaac_sim` | 전면 재작성 (현재 상태 반영) |
| `isaac_drive` | 전면 재작성 + Critical Path 강조 |
| `isaac_interfaces` | 전면 재작성 + docs/interfaces ↔ msg/ 책임 분리 |
| `isaac_perception` ⭐신규 | T2 최진우 stub README |
| `isaac_rl` ⭐신규 | T3 이찬휘 stub README |
| `isaac_supervisor` ⭐신규 | T4 성선규 stub README |
| `isaac_manipulation` ⭐신규 | T2 최진우 stub README |
| `isaac_localization` ⭐신규 | T5 이지민 stub README |

→ 9개 패키지 모두 `0N_<pkg>_README.md` 보유. 사이즈 1.8~4.9 KB.

### P1 — Tx → 사람 이름 병기 결과

| 카테고리 | 처리 방식 |
|---------|----------|
| 트랙 BRIEF/CLAUDE 10개 | sed로 전체 변환 (`T1` → `T1 (김현중)` 등) |
| msg/*.msg 4개 + schema 1개 | sed 변환 + schema description 수동 축약 |
| pm_tools/* | sed 변환 (대부분 자연스러운 약식 표 헤더는 유지) |
| README.md, 패키지 README | 새로 작성하면서 자연스럽게 이름 사용 |
| **STUDY_AND_PLAN.md** | **의도적으로 건너뜀** — 내부 설계 문서, raw Tx가 자연스러움 |
| **SVG flowcharts** | **건너뜀** — 레이아웃 깨질 위험 + 영향도 낮음 |

자연스러운 약식 사용 (예: 표 헤더 "T1 Environment 담당자: 김현중", `[이찬휘 T3]` 라벨)은 강제 변환 안 함.

### P2 — 기타

| 작업 | 상태 |
|------|:---:|
| `README.md` 전면 갱신 (트랙 owner 표 + 현재 상태 + 빠른 시작 + 파일 트리 + 패키지 매핑) | ✅ |
| `docs/README.enhanced.md` 상단에 "클론 reference" 안내 1줄 추가 | ✅ |
| `docs/interfaces/INTERFACE_CONTRACTS.md` 에 새 문서 cross-link | ✅ |

---

## 2. 변경 파일 통계

```
git status:
  modified:  33 files
  new:        7 files (5 package READMEs + I1_TERRAIN_ASSETS.md + META_JSON_FIELDS.md + report)
  untracked: 4 user files (project_overview_flowchart_landscape* — 사용자가 만든 자료, 그대로 둠)
```

---

## 3. 의사결정 / 선택점 (사용자 검토 후 결정 필요)

### 🟡 docs/README.enhanced.md
406 lines짜리 클론 reference. 현재 상단에 ⚠️ 안내만 추가. **사용자 결정 필요**:
- [ ] 옵션 A: 그대로 두기 (클론 분석 자료로)
- [ ] 옵션 B: `docs/archive/` 폴더 만들어 이동
- [ ] 옵션 C: 삭제 (git history엔 남음)

### 🟡 빈 stub Python 파일 5개 패키지
isaac_{perception, rl, supervisor, manipulation, localization}의 모든 .py가 0 byte. **이 자체는 정상** — 각 트랙 owner가 Day 1부터 채울 자리. 패키지 README에 명시했음.

### 🟡 SVG flowcharts Tx → 이름 미적용
147 references × 4 SVG. 레이아웃 깨질 위험과 영향도(시각 자료라 보는 사람이 컨텍스트 파악 가능) 고려하여 보류. **필요시 사용자 결정 후 별도 작업.**

### 🟡 STUDY_AND_PLAN.md Tx → 이름 미적용
1383 lines, 27 references. 내부 설계 문서고 raw Tx가 자연스러운 약식 표현으로 사용됨. 변환하면 오히려 가독성 ↓. **현 상태 유지 권장.**

---

## 4. 변경된 진실의 원천 (Source of Truth)

| 정보 | 위치 |
|------|------|
| **트랙 ↔ 사람 매핑** | [README.md](../../README.md) 상단 표 |
| **5개 인터페이스 계약** | [docs/interfaces/INTERFACE_CONTRACTS.md](../interfaces/INTERFACE_CONTRACTS.md) |
| **I1 풀 구현 가이드** | [docs/interfaces/I1_TERRAIN_ASSETS.md](../interfaces/I1_TERRAIN_ASSETS.md) |
| **meta.json 필드 가이드** | [docs/interfaces/META_JSON_FIELDS.md](../interfaces/META_JSON_FIELDS.md) |
| **각 패키지 책임** | `{pkg}/0N_{pkg}_README.md` |
| **트랙 onboarding** | `docs/tracks/T{1-5}_BRIEF.md` |
| **Claude Code context** | `docs/tracks/T{1-5}_CLAUDE.md` |

---

## 5. 다음 단계 (사용자 아침 결정)

### 즉시 실행 가능
1. **git commit** — 33개 modified + 6개 new docs (사용자 검토 후)
   ```bash
   git add -A  # 다만 .vscode/는 이미 .gitignore라 자동 제외
   git status  # 확인
   git commit -m "docs: workspace optimization - READMEs, Tx→name, I1 guide split"
   ```

2. **김현중 합류 후 작업 인수인계** — 1:1 미팅 (15분)
   - `isaac_sim/scripts/procedural_terrain_generator.py` 코드 의도 설명
   - markers/ 모형 교체 워크플로 시연
   - `mars_physics_config.py` Tier 1 작성 인계

### 사용자 결정 필요
- [ ] `docs/README.enhanced.md` 처리 방향 (옵션 A/B/C 위에 명시)
- [ ] SVG flowcharts Tx → 이름 진행 여부
- [ ] 회의 시작 시간 (kickoff 9:30?)

### Day 1 회의 자료 (이미 준비됨)
- [docs/pm_tools/KICKOFF_AGENDA.md](KICKOFF_AGENDA.md) — 90분 회의 진행 가이드
- [docs/pm_tools/DAILY_STATUS.md](DAILY_STATUS.md) — Day 1 entry 시작 가능
- 회의 후 4명에게 보낼 안내 (이전 채팅 응답에 슬랙 친화적 형식으로 정리됨)

---

## 6. 미해결/보류 항목

| 항목 | 사유 |
|------|------|
| SVG flowcharts Tx → 이름 | 레이아웃 위험 + 영향도 낮음 |
| STUDY_AND_PLAN.md raw Tx | 내부 설계 문서, 자연스러움 |
| `docs/README.enhanced.md` 처리 | 사용자 결정 사항 (archive vs 삭제 vs 유지) |
| `mars_physics_config.py` 채우기 | 김현중 (또는 사용자 PM 짬) 작업 |
| 5개 패키지 노드 구현 | 각 트랙 owner Day 1+ 작업 |

---

## 7. 한 줄 요약

> **9개 패키지 README 완비, 인터페이스 가이드 3종 정합, 트랙별 사람 이름 병기 완료, .gitignore 안전성 확보. 김현중/최진우/이찬휘/이지민이 아침에 본인 트랙 자료를 바로 읽고 작업 시작 가능한 상태.**
