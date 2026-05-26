# TREE_CLEANUP_PLAN.md — 트리 정리 작업 가이드

> 작성 2026-05-26 시연 직전. 시연 후 (2026-05-27 발표 + 2026-05-28 이후) main 을 팀원이 깔끔하게 받을 수 있게 정리하기 위한 작업 list.

⚠️ **시연 전에는 진행 X**. 시연 시각 / mvp.launch.py 동작 변화 위험.

---

## 📋 우선순위 별 작업 목록

### 🔴 우선순위 1 — outdated launch 파일 정리 (영향 큼)

**문제** — `isaac_bringup/launch/` 의 다음 launch 들이 mvp.launch.py 와 mismatch. 팀원이 잘못된 launch 를 띄우면 시연 동작 안 함.

| 파일 | 현재 (outdated) | 권장 조치 |
|---|---|---|
| `supervisor.launch.py` | `battery_monitor_node` 띄움 | `mission_manager_node` 로 교체 + mvp 와 동일 params |
| `perception.launch.py` | `perception_node` (stub) 띄움 | `yolo_perception_node` 로 교체 |
| `drive.launch.py` | coverage_node 만, param/remap 없음 | mvp 의 coverage 노드 설정 그대로 |
| `manipulation.launch.py` | arm_executor 만, params 없음 | mvp 의 arm_executor 설정 (ik_descend_dz=-0.40) |
| `sim.launch.py` | `sim_bridge_node` 띄움 | Isaac Sim 자체는 launch 못 띄움 — 해당 launch 폐기 |
| `full_system.launch.py` | 위 outdated launch 들 include | mvp + localization 옵션으로 재작성 |

**작업 추정** — 각 launch 30분 × 6 = 3h.

**안전성** — mvp.launch.py + localization.launch.py 는 검증됐으니 그대로 유지. 나머지만 손댐.

---

### 🟡 우선순위 2 — stub / 빈 파일 정리

**isaac_drive/primitives/**:
- `drive_to_target.py`, `avoid_obstacle.py`, `stop_rover.py` — stub. coverage_node 가 직접 처리하므로 미사용.
- **조치** — 디렉토리 삭제 또는 `# stub` 명시 README 추가.

**isaac_manipulation/primitives/**:
- `pick_mineral.py`, `place_to_cargo.py`, `unload_to_base.py`, `deploy_solar_panel.py` — stub. arm_executor 가 action 으로 직접 처리.
- **조치** — 디렉토리 삭제 또는 archive.

**isaac_perception/**:
- `perception_node.py` (legacy stub) vs `yolo_perception_node.py` (실 사용). 전자 미사용.
- `models/mineral_detector.pt` (1 byte placeholder) — 실 모델 `mineral_yolo_best.pt` 외에 무의미.
- **조치** — perception_node.py 삭제 + mineral_detector.pt 삭제.

**isaac_sim/scripts/**:
- `basecamp_visual_builder.py`, `mars_physics_config.py` — 빈 파일.
- **조치** — 삭제 또는 stub 명시.

**isaac_sim/assets/vehicle/**:
- `vehicle_origin_T2.usd` — T2 초기 자산. v3 가 대체. 참조 안 됨.
- **조치** — archive 또는 삭제.

**작업 추정** — 30분~1h.

---

### 🟡 우선순위 3 — docs 정리

**docs/README.enhanced.md** — README.md 와 중복. archive 디렉토리 이동 또는 삭제.

**docs/tracks/T*_BRIEF.md / T*_CLAUDE.md** — Day 1 시점 stub. 현재 상태 (시연 후) 로 갱신 또는 archive.

**docs/system_design/ARCHITECTURE_EVAL_2026_05_25.md** — 시연 후 2026-05-27 시점 평가 추가 (cheat 청산 경로 포함).

**작업 추정** — 트랙별 README 갱신 1h + 평가 갱신 30분.

---

### 🟢 우선순위 4 — isaac_rl stub 정리

`isaac_rl/` 전체가 stub (`driving_policy_node.py` 만 골격). 시연 미사용.

**조치 선택지**:
- (A) 그대로 둠 — 시연 후 RL 통합 작업 시점에 채움. list_to_fix 의 "머신러닝 활용" 참조.
- (B) `driving_policy_node` 외 빈 파일 (`ppo_wrapper`, `rl_trainer` 등) 삭제.

권장 (A).

---

### 🟢 우선순위 5 — temp/ 정리

`.gitignore` 에 등록 → commit 영향 X. 단 개인 디스크 정리:

```bash
# 사용자 개인 판단에 따라
rm -rf temp/build_camera_test_usd.py temp/_dump_*.py temp/camera_test.usd temp/step*.usd ...
# tools/isaac-pypi 로 대체된 temp/ros-isaac-python-pypi 제거
rm temp/ros-isaac-python-pypi
```

다른 트랙 owner 가 본인 작업물 보존 원할 수 있으니 강제 X.

---

## 🛠️ 작업 진행 순서 (내일 오전 권장)

1. **0830 — 환경 셋업** — `colcon clean` + `colcon build --symlink-install` 로 깨끗한 상태 확인.
2. **0900 — 우선순위 1 (launch 정리)** — `supervisor.launch.py` → `perception.launch.py` → `drive.launch.py` → `manipulation.launch.py` → `sim.launch.py` → `full_system.launch.py`. 각 수정 후 `ros2 launch ... --show-args` 로 syntax 검증.
3. **1030 — 우선순위 2 (stub 정리)** — 디렉토리/파일 삭제. `git rm` 사용.
4. **1130 — 우선순위 3 (docs)** — README.enhanced.md archive + T*_BRIEF 갱신.
5. **1230 — colcon build 재확인 + mvp.launch.py 실행 검증** — 정리 후에도 시연 동작 그대로 PASS.
6. **1300 — 커밋 + push** — `chore(cleanup): outdated launch + stub + docs 정리`.

총 ~4시간.

---

## ⚠️ 손대지 말 것 (시연 동작 영향)

- `isaac_bringup/launch/mvp.launch.py` ✋
- `isaac_bringup/launch/rqt_views.launch.py` ✋
- `isaac_bringup/launch/localization.launch.py` ✋
- `isaac_sim/assets/vehicle/vehicle_v3.usd` (binary, 재빌드 시 변화) ✋
- `isaac_sim/assets/generated_terrains/terrain_00001~00022/` ✋
- `isaac_perception/models/mineral_yolo_best.pt` ✋
- `isaac_*/setup.py` 의 `data_files`, `entry_points` (실행 영향) ✋
- arm_executor / coverage / supervisor / yolo_perception / ekf_fusion 등 핵심 노드 코드 ✋

---

## 검증 체크리스트

정리 후:

- [ ] `colcon build --symlink-install` 성공
- [ ] `ros2 launch isaac_bringup mvp.launch.py --show-args` 에러 없음
- [ ] `ros2 launch isaac_bringup localization.launch.py --show-args` 에러 없음
- [ ] `ros2 launch isaac_bringup full_system.launch.py --show-args` (갱신 후) 에러 없음
- [ ] 3 터미널 시연 (T1 + T2 + T3) PASS — rover EXPLORE → PICK → CARGO 흐름 동작
- [ ] `git status` 깨끗 (모든 파일 staged + committed)

---

## 관련

- 시연 작업 사항: [list_to_fix.md](../list_to_fix.md)
- 시연 실행: [../README.md#-팀원이-main-pull-후-해야-할-것-quick-start](../README.md)
