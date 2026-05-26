# isaac_bringup

> **트랙 owner**: 성선규 (T4 — Integration + PM)
> **책임**: 시스템 진입점 — launch 파일 8개로 트랙별/전체 실행 묶음

---

## 1. 모듈 역할

`isaac_bringup`은 시스템의 **launch 진입점**. 다른 8개 패키지의 노드를 묶어 실행.

- 전체 통합 실행 (`full_system.launch.py`) 또는 부분 실행 (트랙별)
- 트랙 owner가 자기 영역만 개발/디버그할 때 부분 launch 사용
- T4 성선규의 PM 책임 영역. ROS2 wiring + DIST(Daily Integration Smoke Test)의 entry point.

---

## 2. 폴더 구조 (현재 상태)

```text
isaac_bringup/
├─ launch/                       ✅ 8개 launch 파일 골격 존재 (내용은 stub 또는 미완성)
│  ├─ full_system.launch.py       # 전체 통합
│  ├─ sim.launch.py               # Isaac Sim bridge만
│  ├─ perception.launch.py        # T2 최진우 perception만
│  ├─ rl.launch.py                # T3 이찬휘 RL inference만
│  ├─ drive.launch.py             # T3 이찬휘 driving만
│  ├─ supervisor.launch.py        # T4 성선규 mission supervisor만
│  ├─ manipulation.launch.py      # T2 최진우 M0609만
│  └─ localization.launch.py      # T5 이지민 TRN/EKF만
├─ package.xml
└─ setup.py
```

**현재 상태**: launch 파일들이 골격(파일은 존재)만 있고, 각 트랙 노드가 실제 채워지면 launch 내용도 같이 작성됨.

---

## 3. 실행 패턴

### 전체 통합 (Day 5 이후)
```bash
ros2 launch isaac_bringup full_system.launch.py
```

### 트랙별 부분 실행 (Day 1-4 개발용)
```bash
ros2 launch isaac_bringup sim.launch.py            # 김현중 환경 검증
ros2 launch isaac_bringup perception.launch.py     # 최진우 vision 단독
ros2 launch isaac_bringup drive.launch.py          # 이찬휘 주행 단독
ros2 launch isaac_bringup localization.launch.py   # 이지민 TRN 단독
ros2 launch isaac_bringup manipulation.launch.py   # 최진우 M0609 단독
ros2 launch isaac_bringup supervisor.launch.py     # 성선규 mission 단독
ros2 launch isaac_bringup rl.launch.py             # 이찬휘 RL inference
```

---

## 4. Day별 우선순위 (성선규 작업)

| Day | 작업 | 상태 |
|:---:|------|:---:|
| 1 | git/Notion 셋업, kickoff 진행 | ⏳ |
| 2 | 각 트랙 hello-world launch 검증 (DIST) | ⏳ |
| 3-4 | ROS2 wiring (토픽 전체 흐름 검증), RViz config 1차 | ⏳ |
| 5 | end-to-end 통합 (full_system.launch.py) ⚠️ 게이트 | ⏳ |
| 6 | `demo-stable-v1` git tag | ⏳ |
| 7-8 | 발표 자료, dry-run | ⏳ |

자세한 일정: [docs/tracks/T4_BRIEF.md §10](../docs/tracks/T4_BRIEF.md)

---

## 5. 다른 패키지 의존성

이 패키지는 **모든 9개 패키지의 launch dependency**. 따라서 `package.xml`의 `<exec_depend>`에 모두 명시:

```xml
<exec_depend>isaac_sim</exec_depend>
<exec_depend>isaac_perception</exec_depend>
<exec_depend>isaac_rl</exec_depend>
<exec_depend>isaac_drive</exec_depend>
<exec_depend>isaac_supervisor</exec_depend>
<exec_depend>isaac_manipulation</exec_depend>
<exec_depend>isaac_localization</exec_depend>
<exec_depend>isaac_interfaces</exec_depend>
```

---

## 6. DIST 도구 연계

매일 18:00 실행되는 [docs/pm_tools/run_dist.sh](../docs/pm_tools/run_dist.sh)가 본 패키지의 launch를 호출해 통합 검증. 깨지면 그날 안에 fix.

---

## 7. 한 줄 요약

> **시스템의 entry point. 8개 launch 파일로 전체/부분 실행 분리.** T4 성선규의 ROS2 wiring 통합 책임 영역. DIST가 매일 여기서 시작.
