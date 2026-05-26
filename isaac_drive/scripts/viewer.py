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
  단일:  /usr/bin/python3 viewer.py /tmp/starcraft_map_state.npz
  다중:  /usr/bin/python3 viewer.py "/tmp/starcraft_map_state_*.npz"
         (인용 부호 필수 — shell 이 glob 하기 전에 viewer 가 받음)
"""
import glob
import os
import re
import sys
import time
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.patches import Circle
from matplotlib.colors import ListedColormap

DATA_ARG = sys.argv[1] if len(sys.argv) > 1 else "/tmp/starcraft_map_state.npz"
MULTI_MODE = "*" in DATA_ARG
POLL = 0.05

# 다중 rover 시 각자 다른 색
ROVER_COLORS = ["red", "deepskyblue", "lime", "magenta", "orange", "yellow"]


def _set_korean_font():
    candidates = [
        "Noto Sans CJK KR", "Noto Sans CJK JP", "Noto Sans KR",
        "NanumGothic", "NanumBarunGothic", "NanumSquare",
        "Malgun Gothic", "AppleGothic", "UnDotum", "Gulim", "Batang",
    ]
    available = {f.name for f in fm.fontManager.ttflist}
    for name in candidates:
        if name in available:
            matplotlib.rcParams["font.family"] = name
            matplotlib.rcParams["axes.unicode_minus"] = False
            print(f"[viewer] 한글 폰트: {name}", flush=True)
            return
    print("[viewer] 한글 폰트 없음 — 라벨이 깨질 수 있습니다. "
          "설치: sudo apt install fonts-noto-cjk", flush=True)


_set_korean_font()

if "figure.raise_window" in matplotlib.rcParams:
    matplotlib.rcParams["figure.raise_window"] = False

plt.ion()
fig, axes = plt.subplots(1, 2, figsize=(14, 7))
try:
    fig.canvas.manager.set_window_title(
        "Starcraft Map (multi)" if MULTI_MODE else "Starcraft Map")
except Exception:
    pass

ax_map, ax_fog = axes[0], axes[1]
ax_map.set_title("Map (Ground Truth)")
ax_fog.set_title("Fog of War")
for ax in (ax_map, ax_fog):
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.grid(True, alpha=0.2)

# 동적 객체 — 단일 모드 / 다중 모드 별도 관리
fog_img = None
sector_labels_fog = []
initialized = False
# 다중 모드: rover_id → {dot_map, head_map, dot_fog, head_fog, reveal, label, color}
rover_handles: dict = {}
# 단일 모드 (back-compat)
rover_dot_map = rover_dot_fog = None
heading_line_map = heading_line_fog = None
reveal_circle = None
path_line = target_marker = candidate_dots = None
title_obj = fig.suptitle("waiting…", y=0.98)
plt.show(block=False)


def _rover_id_from_path(p: str) -> str:
    """파일명에서 rover_id 추출. 예: starcraft_map_state_rover_1.npz → rover_1."""
    base = os.path.basename(p)
    m = re.match(r"starcraft_map_state_(.+)\.npz$", base)
    if m:
        return m.group(1)
    return "rover"


def _list_data_paths():
    if MULTI_MODE:
        return sorted(glob.glob(DATA_ARG))
    if os.path.exists(DATA_ARG):
        return [DATA_ARG]
    return []


def _init_base_layer(data, W, H):
    global initialized, fog_img
    obstacle_mask = data["obstacle_mask"]
    grid_n = int(data["grid_n"])
    extent = [-W / 2, W / 2, -H / 2, H / 2]

    ax_map.set_xlim(-W / 2, W / 2)
    ax_map.set_ylim(-H / 2, H / 2)
    obs_m = np.ma.masked_where(obstacle_mask == 0, obstacle_mask)
    ax_map.imshow(obs_m, extent=extent, origin="lower",
                  cmap=ListedColormap(["gray"]), alpha=0.75)
    ax_fog.imshow(obs_m, extent=extent, origin="lower",
                  cmap=ListedColormap(["orangered"]),
                  alpha=0.85, zorder=4)

    sec_w = W / grid_n
    sec_h = H / grid_n
    for k in range(1, grid_n):
        for ax in (ax_map, ax_fog):
            ax.axvline(-W / 2 + k * sec_w, color="dimgray",
                       linestyle="--", linewidth=0.8, alpha=0.6)
            ax.axhline(-H / 2 + k * sec_h, color="dimgray",
                       linestyle="--", linewidth=0.8, alpha=0.6)
    for s in range(grid_n * grid_n):
        row = s // grid_n
        col = s % grid_n
        cx = -W / 2 + (col + 0.5) * sec_w
        cy = -H / 2 + (row + 0.5) * sec_h
        lbl = ax_fog.text(cx, cy, f"{s + 1}", color="white",
                          ha="center", va="center", fontsize=14,
                          fontweight="bold", alpha=0.3)
        sector_labels_fog.append(lbl)

    ax_fog.set_xlim(-W / 2, W / 2)
    ax_fog.set_ylim(-H / 2, H / 2)
    fog_img = ax_fog.imshow(
        data["fog"], extent=extent, origin="lower",
        cmap="gray", vmin=0, vmax=1, alpha=0.85,
    )
    initialized = True


def _ensure_rover_handles(rover_id: str, color: str):
    """다중 모드 — rover 별 dot/heading/reveal/label 생성."""
    if rover_id in rover_handles:
        return rover_handles[rover_id]
    dot_map, = ax_map.plot([], [], "o", color=color, markersize=10, zorder=5)
    head_map, = ax_map.plot([], [], "-", color=color, linewidth=2, zorder=5)
    dot_fog, = ax_fog.plot([], [], "o", color=color, markersize=10, zorder=5)
    head_fog, = ax_fog.plot([], [], "-", color=color, linewidth=2, zorder=5)
    reveal = Circle((0, 0), 0, fill=False, edgecolor=color, linewidth=1.5,
                    linestyle=":", alpha=0.7, zorder=4)
    ax_fog.add_patch(reveal)
    label = ax_fog.text(0, 0, rover_id, color=color, fontsize=9,
                        fontweight="bold", zorder=6,
                        ha="left", va="bottom")
    rover_handles[rover_id] = {
        "dot_map": dot_map, "head_map": head_map,
        "dot_fog": dot_fog, "head_fog": head_fog,
        "reveal": reveal, "label": label, "color": color,
    }
    return rover_handles[rover_id]


def _init_single_rover_handles():
    """단일 모드 — 기존 객체 + path/candidate/target overlay 생성."""
    global rover_dot_map, rover_dot_fog, heading_line_map, heading_line_fog
    global reveal_circle, path_line, target_marker, candidate_dots
    rover_dot_map, = ax_map.plot([], [], "ro", markersize=10, zorder=5)
    heading_line_map, = ax_map.plot([], [], "r-", linewidth=2, zorder=5)
    rover_dot_fog, = ax_fog.plot([], [], "o", color="lime",
                                  markersize=10, zorder=5)
    heading_line_fog, = ax_fog.plot([], [], "-", color="lime",
                                     linewidth=2, zorder=5)
    reveal_circle = Circle((0, 0), 0,
                           fill=False, edgecolor="yellow", linewidth=1.5,
                           linestyle=":", alpha=0.7, zorder=4)
    ax_fog.add_patch(reveal_circle)
    candidate_dots, = ax_fog.plot(
        [], [], ".", color="cyan", markersize=5, alpha=0.55, zorder=5)
    path_line, = ax_fog.plot(
        [], [], "-", color="deepskyblue", linewidth=2.0, zorder=6)
    target_marker, = ax_fog.plot(
        [], [], "*", color="magenta", markersize=20,
        markeredgecolor="black", markeredgewidth=0.6, zorder=8)


last_mtimes: dict = {}
# 다중 모드: 각 rover 의 fog 를 따로 보관 → union (OR) 해서 표시
rover_fogs: dict = {}
print(f"[viewer] 대기: {DATA_ARG}  (multi={MULTI_MODE})")

while plt.fignum_exists(fig.number):
    try:
        paths = _list_data_paths()
        any_loaded = False
        # 메타(맵 크기 등)는 첫 valid file 에서만 초기화
        for i, p in enumerate(paths):
            try:
                mtime = os.path.getmtime(p)
            except OSError:
                continue
            if last_mtimes.get(p) == mtime:
                continue
            last_mtimes[p] = mtime
            try:
                with np.load(p, allow_pickle=False) as data:
                    W = float(data["map_size"][0])
                    H = float(data["map_size"][1])
                    reveal_radius = float(data["reveal_radius"])
                    fog = data["fog"]
                    rover = data["rover"]
                    rx, ry, yaw = float(rover[0]), float(rover[1]), float(rover[2])
                    sector_ratios = data["sector_ratios"]
                    current_sector = int(data["current_sector"])
                    path = (data["path"] if "path" in data.files
                            else np.zeros((0, 2), np.float32))
                    candidates = (data["candidates"] if "candidates" in data.files
                                  else np.zeros((0, 2), np.float32))

                    if not initialized:
                        _init_base_layer(data, W, H)
                        if not MULTI_MODE:
                            _init_single_rover_handles()

                    if MULTI_MODE:
                        rid = _rover_id_from_path(p)
                        color = ROVER_COLORS[
                            sorted(rover_handles.keys() | {rid}).index(rid)
                            % len(ROVER_COLORS)]
                        h = _ensure_rover_handles(rid, color)
                        h["dot_map"].set_data([rx], [ry])
                        h["dot_fog"].set_data([rx], [ry])
                        hx = rx + 0.8 * np.cos(yaw)
                        hy = ry + 0.8 * np.sin(yaw)
                        h["head_map"].set_data([rx, hx], [ry, hy])
                        h["head_fog"].set_data([rx, hx], [ry, hy])
                        h["reveal"].center = (rx, ry)
                        h["reveal"].set_radius(reveal_radius)
                        h["label"].set_position((rx + 0.5, ry + 0.5))
                    else:
                        rover_dot_map.set_data([rx], [ry])
                        rover_dot_fog.set_data([rx], [ry])
                        hx = rx + 0.8 * np.cos(yaw)
                        hy = ry + 0.8 * np.sin(yaw)
                        heading_line_map.set_data([rx, hx], [ry, hy])
                        heading_line_fog.set_data([rx, hx], [ry, hy])
                        reveal_circle.center = (rx, ry)
                        reveal_circle.set_radius(reveal_radius)
                        if candidates.shape[0] > 0:
                            candidate_dots.set_data(
                                candidates[:, 0], candidates[:, 1])
                        else:
                            candidate_dots.set_data([], [])
                        if path.shape[0] > 0:
                            path_line.set_data(path[:, 0], path[:, 1])
                            target_marker.set_data([path[-1, 0]], [path[-1, 1]])
                        else:
                            path_line.set_data([], [])
                            target_marker.set_data([], [])

                    # fog 갱신 — 다중 모드에서는 모든 rover fog 의 union (OR).
                    if fog_img is not None:
                        if MULTI_MODE:
                            rover_fogs[_rover_id_from_path(p)] = fog
                            # 모든 rover fog 의 union → 어느 rover 든 한 번이라도 봤으면 밝힘
                            combined = None
                            for rf in rover_fogs.values():
                                if combined is None:
                                    combined = rf.astype(np.uint8).copy()
                                else:
                                    combined = np.maximum(combined,
                                                          rf.astype(np.uint8))
                            if combined is not None:
                                fog_img.set_data(combined)
                        else:
                            fog_img.set_data(fog)

                    # sector 강조 — 단일 모드만
                    if not MULTI_MODE:
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
                    if MULTI_MODE:
                        title_obj.set_text(
                            f"rovers={len(rover_handles)}  "
                            f"전체 {overall:.1f}%")
                    else:
                        title_obj.set_text(
                            f"pos=({rx:+.2f},{ry:+.2f}) "
                            f"yaw={np.rad2deg(yaw):+.0f}° | "
                            f"전체 {overall:.1f}% | "
                            f"현재구역={current_sector + 1} "
                            f"({sector_ratios[current_sector] * 100:.0f}%)")
                    any_loaded = True
            except Exception as e:
                print(f"[viewer] 로드 실패 {p}: {e}")
                continue
        if any_loaded:
            fig.canvas.draw_idle()
        try:
            fig.canvas.flush_events()
        except Exception:
            pass
        time.sleep(POLL)
    except KeyboardInterrupt:
        break
    except Exception as e:
        print(f"[viewer] 에러: {e}")
        time.sleep(POLL)

print("[viewer] 종료")
