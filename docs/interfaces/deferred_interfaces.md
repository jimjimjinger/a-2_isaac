# ⏳ Deferred Interfaces (I6 ~ I10)

> **Day 1 합의 항목이 아닙니다.** T4 (성선규)(UI) 가 Day 4-5 작업 시 T3와 1:1 협의로 확정.
> 본 문서는 **초안 + 후행 참고용**.

---

## 왜 미루는가

이 5개 인터페이스는 **T3 (이찬휘) → T4 (성선규) (UI)** 단방향. T4 (성선규)(사용자/PM) 외에 consumer 없음.
- 다른 트랙 작업 막지 않음
- T4 (성선규) owner 본인이 결정 가능
- T3 (이찬휘) 시니어와 1:1 협의면 충분

→ Day 1 합의 90분 회의에 욱여넣을 필요 X.

**확정 시점**: Day 4 (T4 (성선규)가 UI 본격 작업 시작할 때).

---

## I6. `/rover/pose_sources` (T5 (이지민) → T4 (성선규))

각 센서별 추정치를 분리해서 UI에 표시. **UI 디자인용**.

```
# PoseSources.msg (초안)
std_msgs/Header header
geometry_msgs/PoseStamped wheel_odom     # 휠 적분만
geometry_msgs/PoseStamped imu_integrated # IMU 적분만
geometry_msgs/PoseStamped sun_heading    # 태양 sensor (yaw만)
geometry_msgs/PoseStamped visual_odom    # vision 추정 (가끔 미발행)
geometry_msgs/PoseStamped fused          # I5와 동일 (최종 융합)
bool[5] source_active                    # 각 센서 활성 여부
```

**Rate**: 30 Hz (I5와 동기)
**UI 표시 예**: "센서 5개 활성, fused pose 신뢰도 87%"

---

## I7. `/mission/status` (T3 (이찬휘) → T4 (성선규))

미션 진행 상황. UI 대시보드의 핵심.

```
# MissionStatus.msg (초안)
std_msgs/Header header
string phase                   # "EXPLORE" | "APPROACH_MINERAL" | "PICK" | "RETURN_BASE" | "IDLE" | "DONE"
int32 current_mineral_id       # APPROACH 중일 때만, 그 외 -1
int32 cargo_count
int32 cargo_capacity           # 기본 10
float32 cargo_value_total      # 누적 점수
float32 mission_elapsed_s
geometry_msgs/Point current_goal
float32 distance_to_goal_m
```

**Rate**: 5 Hz (UI 갱신 충분)

---

## I8. `/mission/minimap` (T3 (이찬휘) → T4 (성선규))

**ROS2 표준 사용**: `nav_msgs/OccupancyGrid`

```
std_msgs/Header header
nav_msgs/MapMetaData info
  float32 resolution           # m/cell
  uint32 width, height
  geometry_msgs/Pose origin
int8[] data                    # row-major
  # 값 인코딩:
  #  -1 = unknown
  #   0 = unvisited
  #  25 = mineral_spotted
  #  50 = obstacle
  # 100 = visited
```

**Rate**: 2 Hz (변경 있을 때만, throttled)
**시각화**: RViz의 OccupancyGrid 플러그인으로 즉시 표시 가능

---

## I9. `/mission/path` (T3 (이찬휘) → T4 (성선규))

A*가 계산한 path 시각화용.

**ROS2 표준 사용**: `nav_msgs/Path`

```
std_msgs/Header header
geometry_msgs/PoseStamped[] poses    # waypoint 시퀀스
```

**Rate**: event-based (A* 재계획 시만)
**시각화**: RViz의 Path 플러그인으로 즉시 표시 가능

---

## I10. `/mission/cargo_event` (T3 (이찬휘) → T4 (성선규))

발표용 알림.

```
# CargoEvent.msg (초안)
std_msgs/Header header
string event_type              # "mineral_collected" | "cargo_full" | "deposited" | "mission_complete"
int32 mineral_id               # event_type=collected일 때만
float32 value_score
int32 cargo_count_after
```

**Rate**: event-based
**UI 효과**: "✨ 광물 #3 수집! (+25점)" 토스트 알림

---

## Day 4에 결정할 것

1. 위 5개 메시지 필드 최종 확정
2. RViz 사용 여부 (`OccupancyGrid`, `Path`는 RViz 친화적)
3. 커스텀 UI 라이브러리 (matplotlib vs PyQt vs web dashboard)
4. 발표 데모 시나리오 2개 (성공 데모 / 도전 데모)에 맞춰 UI 우선순위

---

## 관련 작업물 (Day 4 이후)

```
tracks/T4 (성선규)/
├── ui/
│   ├── mission_dashboard.py      # 메인 UI
│   ├── widgets/
│   │   ├── minimap_widget.py
│   │   ├── pose_widget.py
│   │   ├── cargo_widget.py
│   │   └── camera_widget.py
│   └── rviz_config.rviz
└── demo_scenarios/
    ├── scenario_a_success.py
    └── scenario_b_challenge.py
```
