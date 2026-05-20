"""별도 프로세스 시각화 (시스템 파이썬 matplotlib).

main.py 가 /tmp/starcraft_map_state.npz 에 상태를 쓰면 이 스크립트가 폴링.

표시:
  - 왼쪽: 원본 맵 (장애물 + 9구역 경계 + 로봇)
  - 오른쪽: 안개 맵 (밝혀진 영역 + 로봇 + reveal 원)

데이터 (.npz):
  - rover:     (3,)   x, y, yaw
  - fog:       (H, W) uint8  0=안개 / 1=밝힘
  - obstacles: (N, 4) float  x, y, w, h (각 큐브 중심+크기)
  - map_size:  (2,)   W, H
  - cell_size: () float
  - reveal_radius: () float
  - grid_n:    () int
  - sector_ratios: (N²,) float
  - current_sector: () int

실행:
  /usr/bin/python3 viewer.py /tmp/starcraft_map_state.npz
"""
import os
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.colors import ListedColormap

DATA_PATH = sys.argv[1] if len(sys.argv) > 1 else "/tmp/starcraft_map_state.npz"
POLL = 0.05

plt.ion()
fig, axes = plt.subplots(1, 2, figsize=(14, 7))
try:
    fig.canvas.manager.set_window_title("Starcraft Map")
except Exception:
    pass

ax_map, ax_fog = axes[0], axes[1]
ax_map.set_title("Map (Ground Truth)")
ax_fog.set_title("Fog of War")
for ax in (ax_map, ax_fog):
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.grid(True, alpha=0.2)

# 동적 객체들 (재사용)
fog_img = None
rover_dot_map, rover_dot_fog = None, None
heading_line_map, heading_line_fog = None, None
reveal_circle = None
obstacle_patches = []
sector_lines_map, sector_lines_fog = [], []
sector_labels_fog = []
path_line = None              # mission 이 정한 현재 A* 경로
target_marker = None          # 현재 목표 anchor
candidate_dots = None         # 남은 anchor 후보
title_obj = fig.suptitle("waiting…", y=0.98)
plt.show(block=False)

last_mtime = 0.0
initialized = False
print(f"[viewer] 대기: {DATA_PATH}")

while plt.fignum_exists(fig.number):
    try:
        if os.path.exists(DATA_PATH):
            mtime = os.path.getmtime(DATA_PATH)
            if mtime != last_mtime:
                last_mtime = mtime
                with np.load(DATA_PATH, allow_pickle=False) as data:
                    rover = data["rover"]
                    fog = data["fog"]
                    obstacle_mask = data["obstacle_mask"]
                    map_size = data["map_size"]
                    cell_size = float(data["cell_size"])
                    reveal_radius = float(data["reveal_radius"])
                    grid_n = int(data["grid_n"])
                    sector_ratios = data["sector_ratios"]
                    current_sector = int(data["current_sector"])
                    # mission 동선 (구버전 npz 호환 위해 존재 여부 확인)
                    path = (data["path"] if "path" in data.files
                            else np.zeros((0, 2), np.float32))
                    candidates = (data["candidates"] if "candidates" in data.files
                                  else np.zeros((0, 2), np.float32))

                W, H = float(map_size[0]), float(map_size[1])
                rx, ry, yaw = float(rover[0]), float(rover[1]), float(rover[2])
                extent = [-W / 2, W / 2, -H / 2, H / 2]

                if not initialized:
                    # ── 원본 맵 (왼쪽) — 한 번만 그림 ──
                    ax_map.set_xlim(-W / 2, W / 2)
                    ax_map.set_ylim(-H / 2, H / 2)
                    # 장애물 격자 오버레이 (0=투명, 1=색). obstacle_grid.npy 기반.
                    obs_m = np.ma.masked_where(obstacle_mask == 0, obstacle_mask)
                    ax_map.imshow(obs_m, extent=extent, origin="lower",
                                  cmap=ListedColormap(["gray"]), alpha=0.75)
                    ax_fog.imshow(obs_m, extent=extent, origin="lower",
                                  cmap=ListedColormap(["orangered"]),
                                  alpha=0.85, zorder=4)

                    # 9구역 경계선
                    sec_w = W / grid_n
                    sec_h = H / grid_n
                    for k in range(1, grid_n):
                        for ax in (ax_map, ax_fog):
                            ax.axvline(-W / 2 + k * sec_w, color="dimgray",
                                       linestyle="--", linewidth=0.8, alpha=0.6)
                            ax.axhline(-H / 2 + k * sec_h, color="dimgray",
                                       linestyle="--", linewidth=0.8, alpha=0.6)

                    # 구역 번호 라벨 (fog 쪽에)
                    for s in range(grid_n * grid_n):
                        row = s // grid_n
                        col = s % grid_n
                        cx = -W / 2 + (col + 0.5) * sec_w
                        cy = -H / 2 + (row + 0.5) * sec_h
                        lbl = ax_fog.text(
                            cx, cy, f"{s + 1}", color="white",
                            ha="center", va="center", fontsize=14,
                            fontweight="bold", alpha=0.3,
                        )
                        sector_labels_fog.append(lbl)

                    # ── 안개 맵 (오른쪽) ──
                    ax_fog.set_xlim(-W / 2, W / 2)
                    ax_fog.set_ylim(-H / 2, H / 2)
                    fog_img = ax_fog.imshow(
                        fog, extent=extent, origin="lower",
                        cmap="gray", vmin=0, vmax=1, alpha=0.85,
                    )

                    # 로봇 + 진행방향
                    rover_dot_map, = ax_map.plot([], [], "ro", markersize=10, zorder=5)
                    heading_line_map, = ax_map.plot([], [], "r-", linewidth=2, zorder=5)
                    rover_dot_fog, = ax_fog.plot([], [], "o", color="lime",
                                                  markersize=10, zorder=5)
                    heading_line_fog, = ax_fog.plot([], [], "-", color="lime",
                                                     linewidth=2, zorder=5)

                    # 현재 reveal 반경 원 (fog 쪽)
                    reveal_circle = Circle(
                        (rx, ry), reveal_radius,
                        fill=False, edgecolor="yellow", linewidth=1.5,
                        linestyle=":", alpha=0.7, zorder=4,
                    )
                    ax_fog.add_patch(reveal_circle)

                    # ── mission 동선 오버레이 (fog 쪽) ──
                    candidate_dots, = ax_fog.plot(
                        [], [], ".", color="cyan", markersize=5,
                        alpha=0.55, zorder=5,
                    )
                    path_line, = ax_fog.plot(
                        [], [], "-", color="deepskyblue", linewidth=2.0,
                        zorder=6,
                    )
                    target_marker, = ax_fog.plot(
                        [], [], "*", color="magenta", markersize=20,
                        markeredgecolor="black", markeredgewidth=0.6, zorder=8,
                    )
                    initialized = True

                # ── 갱신 ──
                fog_img.set_data(fog)
                rover_dot_map.set_data([rx], [ry])
                rover_dot_fog.set_data([rx], [ry])
                hx = rx + 0.8 * np.cos(yaw)
                hy = ry + 0.8 * np.sin(yaw)
                heading_line_map.set_data([rx, hx], [ry, hy])
                heading_line_fog.set_data([rx, hx], [ry, hy])
                reveal_circle.center = (rx, ry)

                # mission 동선: 남은 후보 → 현재 A* 경로 → 목표 anchor
                if candidates.shape[0] > 0:
                    candidate_dots.set_data(candidates[:, 0], candidates[:, 1])
                else:
                    candidate_dots.set_data([], [])
                if path.shape[0] > 0:
                    path_line.set_data(path[:, 0], path[:, 1])
                    target_marker.set_data([path[-1, 0]], [path[-1, 1]])
                else:
                    path_line.set_data([], [])
                    target_marker.set_data([], [])

                # 현재 구역 라벨 강조
                for s, lbl in enumerate(sector_labels_fog):
                    if s == current_sector:
                        lbl.set_alpha(0.9)
                        lbl.set_color("yellow")
                    elif sector_ratios[s] > 0.95:
                        lbl.set_alpha(0.7)
                        lbl.set_color("lime")
                    else:
                        lbl.set_alpha(0.3)
                        lbl.set_color("white")

                overall = float(fog.mean()) * 100
                title_obj.set_text(
                    f"pos=({rx:+.2f},{ry:+.2f}) yaw={np.rad2deg(yaw):+.0f}° | "
                    f"전체 {overall:.1f}% | 현재구역={current_sector + 1} "
                    f"({sector_ratios[current_sector] * 100:.0f}%)"
                )
                fig.canvas.draw_idle()
        plt.pause(POLL)
    except KeyboardInterrupt:
        break
    except Exception as e:
        print(f"[viewer] 에러: {e}")
        time.sleep(POLL)

print("[viewer] 종료")
