# 시스템 아키텍처 평가 — 2026-05-25 (Phase 2 통합 테스트 직전)

> 시점: T2 (a) Mineral Detect ROS2 단위 PASS · main 머지(5c3d035) 직후, Phase 2 supervisor (`mission_manager_node`) 작성 직후, 4 터미널 최소 통합 테스트 직전.

---

## 1. 현재 아키텍처 (스냅샷)

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│  vehicle_v3.usd (In-graph OmniGraph)  ← isaac_sim, PyPI wrapper             │
│  pub: /camera/{rover,wrist,sun}/{image_raw,depth,camera_info}                │
│       /ground_truth/odom  /imu/data  /joint_states_raw                       │
│  sub: /cmd_vel  /arm/joint_command                                           │
└──────┬─────────────────────────────┬──────────────────────────┬──────────────┘
       │ image/depth/info            │ ground_truth/odom        │ cmd_vel
       ↓                             ↓                          ↑
┌──────────────────┐         ┌──────────────────────┐  ┌──────────────────────┐
│ yolo_perception_ │         │ odom_to_estimated_   │  │ mission_manager_node │
│      node (T2 a) │         │   pose (T5 placeh.)  │  │  (T4 FSM + cmd_vel   │
│ pub:/perception/ │         │ pub:/rover/estimat.. │  │   mux)               │
│   detections     │         └──────────┬───────────┘  │ pub:/cmd_vel         │
│   image_annotat. │                    │              │     /mission/phase   │
└─────────┬────────┘                    ↓              └──┬────┬─────┬────────┘
          │                ┌──────────────────────┐       │    │     │
          │ detections     │   coverage_node (T3) │───────┘ remap    │
          └────────────────┤   sweep brain        │  /coverage/      │
                           │ pub:/cmd_vel (REMAP) │  cmd_vel_raw     │
                           │     /mission_state   │                  │
                           └──────────────────────┘                  │
                                                                     │
                                          (detections) ──────────────┘
```

`coverage_node` 는 실행 시 `--ros-args -r /cmd_vel:=/coverage/cmd_vel_raw` 로 remap — coverage 소스 무변경.

---

## 2. 노드 책임 매트릭스

| 노드 | 입력 | 출력 | 단일 책임? | 비고 |
|---|---|---|:-:|---|
| **vehicle_v3 (in-graph)** | cmd_vel, arm_cmd | 센서 일체 + GT odom | ✅ | 하드웨어 모사 |
| **yolo_perception_node** | rover RGB+D+Info+odom | DetectionArray + annotated image | ✅ | 시야→감지. world 좌표 추정 (depth+odom) |
| **odom_to_estimated_pose** | /odom | /rover/estimated_pose | ✅ | T5 placeholder adapter |
| **coverage_node** | estimated_pose | cmd_vel(→remap) + mission_state | ✅ | sweep brain (A* + Roomba) |
| **mission_manager_node** | detections + cov_raw + odom | /cmd_vel + /mission/phase | ✅ | 상위 FSM + mux |

---

## 3. 평가 (4축)

| 축 | 점수 | 근거 |
|---|:-:|---|
| **효율성** | **8/10** | YOLO 실측 5 Hz (frame ~30 Hz 중 ~5 frame 추론), latency 200~400ms — 주행 충분. 이미지 토픽 traffic ~18 MB/s (raw+annotated) — 단일 머신 OK, multi-host 가면 압축 필요. coverage 가 idle 시에도 cmd_vel publish — 작은 CPU 낭비 |
| **가독성** | **9/10** | 토픽 네임스페이스 `/camera/`, `/perception/`, `/mission/`, `/rover/` 일관. 노드 책임 단일. 패턴 (declare_parameter + sub + pub + timer) 통일 |
| **확장성** | **8/10** | T5 localization 통합 = `odom_topic` 1줄 swap. mineral 우선순위 / multi-target 큐 확장 쉬움. wrist cam Phase 3 추가도 별도 노드로 클린 |
| **유지보수성** | **9/10** | INTERFACE_CONTRACTS.md 갱신 동기, 메모리 정책화, mock(perception_node/battery_monitor)도 보존해 mock↔real 스왑 쉬움 |

---

## 4. 강점 Top 3

1. **계층 분리가 깨끗** — perception(센서→정보) / coverage(주행 결정) / supervisor(상위 FSM+mux) 세 층이 토픽 인터페이스로만 연결. 한 층 swap 해도 다른 층 영향 0.
2. **don't-modify-working-modules 준수** — coverage_node 소스 단 한 줄 안 건드리고 `--ros-args -r /cmd_vel:=/coverage/cmd_vel_raw` 만으로 mux 가능.
3. **환경 일관성 표준화** — system humble + PyPI wrapper 2가지로 클램프. 노드별 source 패턴 갈리지 않음.

---

## 5. 약점 / 개선 제안

| # | 약점 | 우선순위 | 권장 시점 |
|:-:|---|:-:|---|
| 1 | **APPROACH 시 obstacle 회피 부재** — supervisor `_approach_twist` 가 단순 P-control 직진. mineral 과 rover 사이 큰 바위 있으면 박거나 넘어짐. 해결: `isaac_drive/navigation/path_planner.py` (A*) 를 supervisor 에서 **import 재사용** 권장 — coverage_node 노드는 안 건드리고 알고리즘만 공유 | 🔴 High | Phase 2 통합 직후 |
| 2 | **PICK_READY 영구 정지 (FSM dead-end)** — Phase 3 arm action 완료 handshake 부재. 시연 시 rover 영원 정지 위험 | 🔴 High | Phase 3 같이 |
| 3 | **mineral 1.5 m stop_dist 는 placeholder** — T2 standalone 은 class 별 정밀 거리 (blue 0.25 / yellow 0.75 / green 0.75 m, arm reach 기준). Phase 3 arm 통합 시 이 매핑 적용 필요 | 🟡 Med | Phase 3 |
| 4 | **far-mineral world 좌표 부정확** (15 m 거리 ~1.5 m XY 오차) — bbox 중심 픽셀의 depth 가 mineral edge 잡는 경우. 해결: bbox 내부 median depth | 🟡 Med | 다음 iteration |
| 5 | **단일 launch 파일 부재** — 4 터미널 수동. `isaac_bringup` 에 통합 launch (`a2_isaac.launch.py`) | 🟡 Med | Phase 2 PASS 후 |
| 6 | **tf 미사용** — rover_camera→rover_body, rover_body→world 변환을 노드별 odom 직접 구독. 정공법 robot_state_publisher + tf2_ros | 🟢 Low | vehicle_v3 가 /tf publish 추가될 때 |
| 7 | **이미지 압축 없음** — raw bgr8 두 토픽. multi-host / record 부담. `image_transport` compressed plugin | 🟢 Low | 필요 시 |

---

## 6. FAQ (2026-05-25 사용자 질문)

### Q1. multi-host = 여러 rover (multi-robot) 인가?

아니요. **multi-host = 여러 컴퓨터/머신**. 단일 rover 라도 perception 을 GPU 머신, supervisor 를 노트북, UI 를 다른 머신 등에 분산하는 경우.

**multi-robot** (여러 rover) 은 별개 — `ROS_DOMAIN_ID` 분리 또는 namespace 분리로 처리. 현재 a2_isaac 미션 요구사항에는 없음.

### Q2. APPROACH 시 obstacle_grid 참고해서 경로 짜나?

**현재 미구현**. supervisor `_approach_twist` 는 P-control 직진. 위 약점 #1 참조.

해결 옵션:
- **A (권장)**: supervisor 에서 `isaac_drive/navigation/path_planner.py` (A*) import 재사용. coverage_node 노드 무변경.
- **B**: coverage_node 에 mineral goal 인터페이스 추가 — don't-modify 위반.
- **C**: reactive 회피만 (depth/raycast 기반 stop & turn). 정밀도 X.

### Q3. mineral 1.5 m stop_dist 는 T2 가 정한 기준인가?

아니요. **mission_manager_node 에 임의로 박은 placeholder**.

T2 standalone (`rover_yolo_demo.py`) 의 `STOP_DISTANCE_PER_CLASS`:
| Mineral | Stop distance | 근거 |
|---|:-:|---|
| `blue_mineral`   | 0.25 m | arm reach + gripper open + TCP offset |
| `yellow_mineral` | 0.75 m | mesh 크기 (spike) + safe margin |
| `green_gas`      | 0.75 m | cube 크기 + safe margin |

Phase 3 arm 통합 시 위 매핑을 mission_manager 에 이식.

---

## 7. 통합 테스트 진행 여부

**진행 권장 ✅**.

약점 #2 (PICK_READY handshake), #3 (1.5 m placeholder) 은 Phase 3 와 짝지어야 자연스러움. 통합 테스트에서는 "EXPLORE → APPROACH → PICK_READY 도달 후 정지 + 수동 abort" 까지 확인. 약점 #1 (obstacle 회피) 은 통합 테스트에서 안 끼는 obstacle-free mineral 로 한 번 검증한 뒤 Phase 2 후속 작업으로.
