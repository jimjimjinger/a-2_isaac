# 🔧 ~/.bashrc 셋업 가이드

> 8일 프로젝트 동안 자주 칠 명령어 95%를 alias로 등록 + ROS2 source 자동화 + 팀원 간 ROS_DOMAIN_ID 충돌 방지.
> 본인 트랙에 맞게 §4 추가 사항 함께 적용 권장.

---

## 0. 위치 무관성 — 어디 두든 작동, 다만 통일 권장

ROS2 colcon, git, Python 모두 **상대 경로로 동작**해서 워크스페이스 위치 (`~/dev_ws/`, `~/Desktop/dev_ws/`, `~/projects/...` 등) 기능적으로 무관. 우리 코드/스크립트/문서 어디에도 `/home/<특정유저>` 절대경로 하드코딩 없음.

다만 **협업 일관성** 차원에서 `~/dev_ws/rover_ws/` 권장:

| 상황 | 위치 통일 안 했을 때 | 통일 했을 때 |
|------|---------------------|-------------|
| 에러 메시지 공유 | "에러 at `/home/kim/Desktop/dev_ws/...`" vs "`~/dev_ws/...`" | 동일 경로로 공유 |
| 화면 공유 (zoom) | 터미널 prompt 다 다름 | 같은 경로 보임 |
| README 명령어 copy-paste | 매번 mental translate 필요 | 그대로 붙여넣기 |

이미 다른 위치에 둔 사람은 굳이 옮기지 않아도 됨 — 본인 환경 알고 있으면 충분.

---

## 1. 🔴 필수 (모두 적용)

`~/.bashrc` 끝에 추가:

```bash
# ════════════════════════════════════════════════════════════════
# Mars Rover Project — Day 1 환경
# ════════════════════════════════════════════════════════════════

# ─── ROS2 + 워크스페이스 source ────────────────────────────────
source /opt/ros/humble/setup.bash                                    # ROS2 distro에 맞게 수정
[ -f ~/dev_ws/rover_ws/install/setup.bash ] && \
    source ~/dev_ws/rover_ws/install/setup.bash                      # colcon build 후에만 작동

# ─── ROS_DOMAIN_ID 충돌 방지 (같은 LAN에 팀원 있으면 필수) ────
# 같은 LAN에서 같은 ID면 서로 노드 발견함 → 각자 다른 값
# 합의: 김현중=11, 최진우=22, 이찬휘=33, 성선규=44, 이지민=55
export ROS_DOMAIN_ID=44                                              # ← 본인 값으로 수정
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp                         # (또는 rmw_fastrtps_cpp)
```

적용:
```bash
source ~/.bashrc
```

검증:
```bash
echo $ROS_DOMAIN_ID                      # 44 출력
ros2 pkg list | grep isaac_              # 9개 isaac_* 패키지 출력
```

---

## 2. 🟡 권장 (자주 쓰는 명령 alias)

```bash
# ─── 워크스페이스 단축 ────────────────────────────────────────
alias cdws='cd ~/dev_ws/rover_ws'                                    # 워크스페이스 root
alias cdpkg='cd ~/dev_ws/rover_ws/src/a2_isaac'                      # 우리 repo 루트
alias cdsim='cd ~/dev_ws/rover_ws/src/a2_isaac/isaac_sim'

# ─── colcon build ─────────────────────────────────────────────
alias cb='cd ~/dev_ws/rover_ws && colcon build --symlink-install && source install/setup.bash'
alias cbsel='cd ~/dev_ws/rover_ws && colcon build --symlink-install --packages-select'
# 사용 예: cbsel isaac_drive

# ─── Isaac Sim ────────────────────────────────────────────────
# Isaac Sim 설치 경로는 본인 환경에 맞게 수정
alias isaac='~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/isaac-sim.sh'
alias isaac-python='~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh'

# ─── 우리 프로젝트 자주 쓰는 명령 ─────────────────────────────
alias mars-world='isaac ~/dev_ws/rover_ws/src/a2_isaac/isaac_sim/worlds/mars_exploration_world.usd'
alias gen-terrain='cdpkg && python3 isaac_sim/scripts/procedural_terrain_generator.py'
# 사용 예: gen-terrain --seed 99 --terrain-id terrain_00002

# ─── DIST (Daily Integration Smoke Test) ──────────────────────
alias dist='bash ~/dev_ws/rover_ws/src/a2_isaac/docs/pm_tools/run_dist.sh'
```

---

## 3. 🟢 선택 (편의 함수)

```bash
# ─── 본인 트랙 BRIEF 빠르게 열기 (본인 트랙 번호로 수정) ─────
alias mybrief='less ~/dev_ws/rover_ws/src/a2_isaac/docs/tracks/T4_BRIEF.md'
alias myclaude='less ~/dev_ws/rover_ws/src/a2_isaac/docs/tracks/T4_CLAUDE.md'

# ─── 워크스페이스 상태 한눈에 ─────────────────────────────────
ws-status() {
    cd ~/dev_ws/rover_ws/src/a2_isaac
    echo "=== Branch ==="
    git branch --show-current
    echo "=== Status ==="
    git status --short | head -10
    echo "=== Recent commits ==="
    git log --oneline -5
    cd - > /dev/null
}

# ─── 모든 stub 파일 (0 byte) 확인 ─────────────────────────────
ws-stubs() {
    find ~/dev_ws/rover_ws/src/a2_isaac -name "*.py" -size 0 2>/dev/null \
        | sed "s|$HOME|~|" | head -30
}

# ─── PR 빠른 확인 (gh CLI 필요) ───────────────────────────────
alias prs='cd ~/dev_ws/rover_ws/src/a2_isaac && gh pr list && cd - > /dev/null'
```

---

## 4. 트랙 owner별 추가 권장

### 🗺️ T1 (김현중) — Environment

```bash
# generator 의존성 확인
alias check-gen-deps='python3 -c "from noise import pnoise2; from pxr import Usd; import jsonschema; print(\"✓ generator deps OK\")"'

# medium 난이도 새 terrain 빠르게
gen-medium() {
    cdpkg && python3 isaac_sim/scripts/procedural_terrain_generator.py \
        --seed "$1" --terrain-id "terrain_$(printf '%05d' $1)"
}
# 사용 예: gen-medium 42
```

### 👁️🦾 T2 (최진우) — Perception + M0609

```bash
# 광물 USD 색 빠른 확인
alias check-minerals='ls -la ~/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/markers/mineral_*.usd'

# meta.json의 광물 좌표 출력 (HSV detection GT)
show-minerals() {
    python3 -c "
import json
m = json.load(open('$HOME/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/generated_terrains/terrain_00001/meta.json'))
for x in m['minerals']:
    print(f'  id={x[\"id\"]:2d}  type={x[\"type\"]:6s}  pos=({x[\"position\"][\"x\"]:6.2f}, {x[\"position\"][\"y\"]:6.2f})  value={x[\"value\"]}')
"
}
```

### 🚗 T3 (이찬휘) — Driving (Critical Path)

```bash
# obstacle_grid 빠른 시각화 (matplotlib 필요)
viz-obstacles() {
    python3 -c "
import numpy as np, matplotlib.pyplot as plt
g = np.load('$HOME/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/generated_terrains/terrain_00001/obstacle_grid.npy')
plt.imshow(g, origin='lower', extent=[-25, 25, -25, 25])
plt.title(f'obstacle_grid — {(g==1).mean()*100:.1f}% blocked')
plt.xlabel('x (m)'); plt.ylabel('y (m)')
plt.savefig('/tmp/obstacles.png', dpi=80); print('saved /tmp/obstacles.png')
"
}

# Coverage planner 단독 테스트
alias t3-cov='cdpkg && python3 isaac_drive/isaac_drive/navigation/coverage_planner.py'
alias t3-astar='cdpkg && python3 isaac_drive/isaac_drive/navigation/path_planner.py'
```

### 📍 T5 (이지민) — Localization + TRN

```bash
# heightmap 좌표계 빠른 검증
t5-check-heightmap() {
    python3 -c "
import numpy as np, json
hm = np.load('$HOME/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/generated_terrains/terrain_00001/heightmap.npy')
meta = json.load(open('$HOME/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/generated_terrains/terrain_00001/meta.json'))
print(f'shape={hm.shape}, dtype={hm.dtype}, range=[{hm.min():.3f}, {hm.max():.3f}]')
print(f'origin={meta[\"origin\"]}, res={meta[\"resolution_m\"]}')
# 광물 z 매칭 검증 (z = heightmap_at(x,y) + 0.10)
m = meta['minerals'][0]
i = int((m['position']['y'] - meta['origin']['y']) / meta['resolution_m'])
j = int((m['position']['x'] - meta['origin']['x']) / meta['resolution_m'])
print(f'mineral_01: pos={m[\"position\"]}, heightmap_at={hm[i,j]:.3f} (+0.10={hm[i,j]+0.10:.3f})')
"
}
```

### 🎯 T4 (성선규) — Integration + PM

```bash
# PM 자주 쓰는 자료 빠른 접근
alias kickoff='less ~/dev_ws/rover_ws/src/a2_isaac/docs/pm_tools/KICKOFF_AGENDA.md'
alias daily='vim ~/dev_ws/rover_ws/src/a2_isaac/docs/pm_tools/DAILY_STATUS.md'
alias decisions='vim ~/dev_ws/rover_ws/src/a2_isaac/docs/pm_tools/DECISIONS.md'
alias risks='vim ~/dev_ws/rover_ws/src/a2_isaac/docs/pm_tools/RISK_REGISTER.md'

# 모든 트랙 BRIEF grep (PM이 알아야 할 키워드 찾기)
pm-grep() {
    grep -rn "$1" ~/dev_ws/rover_ws/src/a2_isaac/docs/tracks/ | head -20
}
# 사용 예: pm-grep "ETA"  /  pm-grep "위험"
```

---

## 5. ⚠️ 주의 사항

### Conda / Mamba 사용 시

`conda init bash`가 PATH 앞쪽에 conda python을 두면 ROS2 python 충돌 가능.
ROS2 작업할 때만 `conda deactivate` 후 시작 권장:

```bash
# 작업 전
conda deactivate
source ~/dev_ws/rover_ws/install/setup.bash
```

### Zsh 사용자

`~/.bashrc` 대신 `~/.zshrc`에 적용. alias 문법 동일.

### colcon build 실패 시

```bash
# clean 후 재빌드
cdws && rm -rf build install log && colcon build --symlink-install
```

### ROS_DOMAIN_ID 충돌 진단

같은 LAN에서 다른 팀원의 노드가 본인 `ros2 topic list`에 보이면 ID 충돌:

```bash
# 본인이 띄운 노드 외에도 다른 게 보이면 ROS_DOMAIN_ID 변경
ros2 node list
ros2 topic list
```

---

## 6. 한 번에 붙여넣을 블록 (모두용 minimal)

§1 필수만 적용하고 §2~§4는 필요에 따라 추가하는 패턴 권장:

```bash
# ════════════════════════════════════════════════════════════════
# Mars Rover Project — minimal
# ════════════════════════════════════════════════════════════════
source /opt/ros/humble/setup.bash
[ -f ~/dev_ws/rover_ws/install/setup.bash ] && source ~/dev_ws/rover_ws/install/setup.bash

export ROS_DOMAIN_ID=44   # ← 본인 값으로 (김현중=11, 최진우=22, 이찬휘=33, 성선규=44, 이지민=55)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

alias cdws='cd ~/dev_ws/rover_ws'
alias cdpkg='cd ~/dev_ws/rover_ws/src/a2_isaac'
alias cb='cd ~/dev_ws/rover_ws && colcon build --symlink-install && source install/setup.bash'
alias isaac='~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/isaac-sim.sh'
alias mars-world='isaac ~/dev_ws/rover_ws/src/a2_isaac/isaac_sim/worlds/mars_exploration_world.usd'
alias gen-terrain='cdpkg && python3 isaac_sim/scripts/procedural_terrain_generator.py'
```

적용:
```bash
source ~/.bashrc
echo $ROS_DOMAIN_ID   # 44 확인
```

---

## 7. 한 줄 요약

> **§1 필수만 적용해도 ROS2 source 자동 + 노드 충돌 방지로 협업 시작 OK. §2~§4는 본인 워크플로우에 따라 점진적으로 추가.**
