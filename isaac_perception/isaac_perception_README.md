# isaac_perception

> **트랙 owner**: 최진우 (T2 — Perception + M0609 Vision 측)
> **책임**: 광물 인식 (HSV 색기반) + 장애물/지형 분석 + (확장) depth/lidar

---

## 1. 모듈 역할

Isaac Sim 카메라(RGB)에서 광물/장애물/지형을 인식하고 ROS2로 publish.

- **vision**: HSV 색기반 광물 detection (3색: blue 10pt / red 25pt / yellow 50pt)
- **depth**: RGBD 기반 거리 (확장 영역)
- **lidar**: LiDAR 기반 장애물 (확장 영역, 8일 스코프 아님)

진짜 CNN 학습은 하지 않음. **단색 USD 광물 + HSV threshold**라는 단순화 전략. 자세한 설계 의도: [docs/pm_tools/DECISIONS.md #006](../docs/pm_tools/DECISIONS.md).

---

## 2. 폴더 구조 (현재 상태)

```text
isaac_perception/
├─ isaac_perception/
│  ├─ __init__.py                           ⏳ stub
│  ├─ perception_node.py                    ⏳ stub — main publisher
│  ├─ vision/
│  │  ├─ mineral_detector.py                ⏳ stub — HSV detection
│  │  ├─ obstacle_detector.py               ⏳ stub
│  │  ├─ terrain_analyzer.py                ⏳ stub
│  │  └─ value_scorer.py                    ⏳ stub — 광물 가치 점수
│  ├─ depth/
│  │  └─ depth_estimator.py                 ⏳ stub
│  └─ lidar/                                ⏳ 미구현 (확장)
├─ models/
│  └─ mineral_detector.pt                   📦 학습 weights (Replicator 사용 시)
├─ package.xml
└─ setup.py
```

**모든 .py가 0 byte stub.** 최진우 작업 영역.

---

## 3. 작업 시작 가이드

| 자료 | 위치 |
|------|------|
| 트랙 onboarding | [docs/tracks/T2_BRIEF.md](../docs/tracks/T2_BRIEF.md) |
| Claude Code context | [docs/tracks/T2_CLAUDE.md](../docs/tracks/T2_CLAUDE.md) |
| I2 인터페이스 | [docs/interfaces/INTERFACE_CONTRACTS.md I2](../docs/interfaces/INTERFACE_CONTRACTS.md#i2-perceptiondetections-t2-최진우--t3-이찬휘-t4-성선규) |
| 광물 GT 좌표 | `../isaac_sim/assets/generated_terrains/terrain_00001/meta.json` (minerals[]) |
| 광물 USD (HSV target) | `../isaac_sim/assets/markers/mineral_{blue,red,yellow}.usd` |

---

## 4. Day별 마일스톤

| Day | 목표 |
|:---:|------|
| 1 | ⚠️ M0609 USD asset 호환성 spike + HSV detection PoC (단색 sphere 1개) |
| 2 | HSV 3색 모두 동작 + `/perception/detections` publish 10Hz |
| 3-5 | M0609 통합, 2D→3D 투영, 이지민 T5 estimated_pose 연동, end-to-end pick |
| 6+ | 가치점수, edge case, 폴리싱 |

자세한 일정: [T2_BRIEF.md §7](../docs/tracks/T2_BRIEF.md)

---

## 5. 한 줄 요약

> **최진우의 광물 vision + (별도 패키지 isaac_manipulation에서 M0609 매니퓰레이션).** 8일에 CNN 학습 안 함, HSV 색기반으로 충분.
