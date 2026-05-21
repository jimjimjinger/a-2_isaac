# isaac_localization

> **트랙 owner**: 이지민 (T5 — Localization + Infra)
> **책임**: GPS-less 화성 위치 추정 — TRN + EKF + sensor fusion

---

## 1. 모듈 역할

화성에는 GPS가 없음. 로버는 IMU + Wheel + Sun + **TRN** (Terrain Relative Navigation, 실제 Perseverance 기법)을 EKF로 융합해 자기 위치를 추정.

이 패키지의 **핵심 publish**: `/rover/estimated_pose` (geometry_msgs/PoseWithCovarianceStamped, 30Hz) — 모든 다른 트랙이 사용.

**학술적으로 가장 깊은 트랙**. 자세한 설계 의도: [docs/STUDY_AND_PLAN.md](../docs/STUDY_AND_PLAN.md), [docs/pm_tools/DECISIONS.md #002](../docs/pm_tools/DECISIONS.md).

---

## 2. 폴더 구조 (현재 상태)

```text
isaac_localization/
├─ isaac_localization/
│  ├─ __init__.py                        ⏳ stub
│  ├─ localization_node.py               ⏳ stub — /rover/estimated_pose publisher
│  ├─ ekf_fusion.py                      ⏳ stub — 다중 센서 EKF
│  ├─ trn.py                             ⏳ stub — ⭐ TRN 핵심 (cross-correlation)
│  └─ sensors/
│     ├─ wheel_odom.py                   ⏳ stub
│     ├─ imu_integrator.py               ⏳ stub
│     └─ sun_yaw.py                      ⏳ stub — 태양 광원 → 절대 방위
├─ package.xml
└─ setup.py
```

**모든 .py가 0 byte stub.** 이지민 작업 영역.

---

## 3. 작업 시작 가이드

| 자료 | 위치 |
|------|------|
| 트랙 onboarding | [docs/tracks/T5_BRIEF.md](../docs/tracks/T5_BRIEF.md) |
| Claude Code context | [docs/tracks/T5_CLAUDE.md](../docs/tracks/T5_CLAUDE.md) |
| TRN 입력 (글로벌 heightmap) | `../isaac_sim/assets/generated_terrains/terrain_00001/heightmap.npy` ✅ 사용 가능 |
| 좌표계 합의 ⭐ | [docs/interfaces/I1_TERRAIN_ASSETS.md §6](../docs/interfaces/I1_TERRAIN_ASSETS.md#6-heightmapnpy-상세--가장-중요한-좌표-합의) |
| I5 인터페이스 | [docs/interfaces/INTERFACE_CONTRACTS.md I5](../docs/interfaces/INTERFACE_CONTRACTS.md#i5-roverestimated_pose-t5-이지민--t3-이찬휘-t4-성선규) |

---

## 4. Day별 단계적 구현

| Day | 구현 수준 | publish 상태 |
|:---:|----------|--------------|
| 1 | Stub: GT + 가우시안 노이즈 | T3 이찬휘가 받을 수 있게 인터페이스만 |
| 2 | + Wheel/IMU/Sun 적분 | covariance 누적 |
| 3 | + TRN 단독 검증 ⭐ | TRN 신뢰도 기반 |
| 4 | EKF 융합 (4개 센서) | 동적 (TRN 보정 시 감소) |
| 5+ | 노이즈 σ별 sweep, Mars Tier 2 friction zone | |

T3 이찬휘는 **Day 1-3엔 GT stub 사용**, Day 4에 ros2 mode로 swap (PoseProvider 패턴). → 이지민 본격 TRN 검증은 Day 3 마일스톤.

---

## 5. 의존 데이터 (T1 김현중에게 받음)

```
heightmap.npy  (1000×1000 float32)  ← T1 김현중이 제공, 이미 1샘플 있음
meta.json.origin, resolution        ← 좌표계 합의 기반
```

좌표계 합의: `heightmap[i,j] = world Z at (origin.x + j*res, origin.y + i*res)`, **i=y, j=x, row-major**. 김현중과 한 번 더 점검 필수.

---

## 6. 한 줄 요약

> **이지민의 GPS-less 위치 추정 (TRN + EKF).** 학술적으로 가장 깊은 트랙. heightmap.npy 좌표계 합의가 critical.
