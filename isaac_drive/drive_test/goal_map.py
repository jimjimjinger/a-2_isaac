# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""matplotlib 2D 맵 — 차량 위치·장애물 표시 + 마우스 클릭으로 goal 설정.

좌·우 2개의 서브플롯이 있는 한 창으로 띄운다:

  · 왼쪽 (truth)     — terrain_00022 의 obstacle_grid 를 사전에 깔아 모든
                       바위를 한눈에 보여준다.  '정답지' 역할.
  · 오른쪽 (discovered) — 빈 맵에서 시작.  레이캐스트로 새로 본 장애물만
                       누적해서 점으로 찍힌다.  센서가 본 만큼만 그려져
                       '내가 안 것' 을 시각화.

두 맵 모두 같은 월드 좌표계 — 차량·goal 마커가 동기화되고, 어느 쪽에서
좌클릭해도 goal 이 잡힌다.

main 루프(play_avoid.py)에서:
  · 매 프레임 update(x, y, yaw) 로 차량 마커를 갱신,
  · 매 프레임 add_detections(world_coords) 로 새 감지 누적,
  · gmap.goal 속성으로 현재 goal(x, y) 또는 None 을 읽고,
  · 도착·리셋·수동 입력 시 clear_goal() 로 해제한다.
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

# Isaac Sim 의 Qt 와 충돌하지 않도록 TkAgg 를 명시 — isaaclab venv 기본도 TkAgg.
matplotlib.use("TkAgg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402

# terrain_00022 의 장애물 그리드(2D binary).  drive_test 는 외부 파일을 수정하지
# 않고 경로 참조만 한다 (terrain_00022_new.usdc 와 동일 정책).
_OBSTACLE_GRID_PATH = (
    Path(__file__).resolve().parents[2]
    / "isaac_sim" / "assets" / "generated_terrains" / "terrain_00022"
    / "obstacle_grid.npy"
)


class GoalMap:
    """차량 위치·장애물(사전·감지)을 실시간 표시하는 2-패널 2D 맵.

    Args:
        half_extent: 맵 가시 범위 반경 (m).  terrain_00022 는 50×50m → 25m.
        disc_grid_step: discovered 맵 dedup 격자 (m).  같은 셀에 떨어지는
                        감지는 한 점으로 합친다 — 출력 점 폭주 방지.
    """

    def __init__(
        self,
        half_extent: float = 25.0,
        disc_grid_step: float = 0.2,
    ) -> None:
        self.goal: tuple[float, float] | None = None
        self._disc_grid_step = disc_grid_step
        self._disc_cells: set[tuple[int, int]] = set()  # 누적 감지 셀 좌표

        plt.ion()
        self.fig, (self.ax_truth, self.ax_disc) = plt.subplots(
            1, 2, figsize=(13.0, 6.5)
        )

        for ax, title in (
            (self.ax_truth, "전체 장애물 (사전)"),
            (self.ax_disc, "레이캐스트 감지 (실시간 누적)"),
        ):
            ax.set_xlim(-half_extent, half_extent)
            ax.set_ylim(-half_extent, half_extent)
            ax.set_aspect("equal", adjustable="box")
            ax.set_title(title)
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.grid(True, alpha=0.3)
            ax.plot([0.0], [0.0], "k+", ms=14, mew=2)  # 베이스캠프(원점)

        self.fig.suptitle("좌클릭 = goal · 우클릭 = goal 해제", fontsize=11)

        # 왼쪽 — 사전 장애물 레이어.
        self._draw_obstacle_layer(self.ax_truth, half_extent)

        # 두 axes 의 차량 점·heading 화살표 (화살표는 매 프레임 다시 그림).
        (self.veh_truth,) = self.ax_truth.plot([0.0], [0.0], "bo", ms=10, label="vehicle")
        (self.veh_disc,) = self.ax_disc.plot([0.0], [0.0], "bo", ms=10, label="vehicle")
        self._veh_arrow_truth = None
        self._veh_arrow_disc = None

        # 두 axes 의 goal 마커.
        (self.goal_truth,) = self.ax_truth.plot([], [], "r*", ms=20, label="goal")
        (self.goal_disc,) = self.ax_disc.plot([], [], "r*", ms=20, label="goal")

        # 오른쪽 — discovered 장애물 scatter (빈 상태에서 시작).
        self._disc_scatter = self.ax_disc.scatter(
            [], [], s=20, marker="s",
            c=[(0.55, 0.35, 0.25, 0.85)], label="detected",
        )

        self.ax_truth.legend(loc="upper right", fontsize=9)
        self.ax_disc.legend(loc="upper right", fontsize=9)

        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.draw()
        plt.show(block=False)

    # ---- obstacle layer (truth) ----------------------------------------
    def _draw_obstacle_layer(self, ax, half_extent: float) -> None:
        """terrain_00022/obstacle_grid.npy 를 truth ax 배경에 깐다."""
        try:
            grid = np.load(_OBSTACLE_GRID_PATH)
        except Exception as exc:  # noqa: BLE001
            print(f"[goal_map] obstacle_grid 로드 실패: {exc}")
            return
        cmap = ListedColormap([(0.0, 0.0, 0.0, 0.0),    # 0 → 투명
                               (0.55, 0.35, 0.25, 0.7)])  # 1 → 갈색 반투명
        ax.imshow(
            grid,
            extent=(-half_extent, half_extent, -half_extent, half_extent),
            origin="lower",
            cmap=cmap,
            interpolation="nearest",
            zorder=0,
        )

    # ---- mouse ----------------------------------------------------------
    def _on_click(self, event) -> None:
        # 두 axes 어느 쪽이든 클릭 인정.
        if event.inaxes not in (self.ax_truth, self.ax_disc):
            return
        if event.xdata is None or event.ydata is None:
            return
        if event.button == 1:
            self.set_goal(float(event.xdata), float(event.ydata))
        elif event.button == 3:
            self.clear_goal()

    def set_goal(self, gx: float, gy: float) -> None:
        self.goal = (gx, gy)
        self.goal_truth.set_data([gx], [gy])
        self.goal_disc.set_data([gx], [gy])
        print(f"[GOAL] ({gx:+.2f}, {gy:+.2f}) m  로 이동")

    def clear_goal(self) -> None:
        if self.goal is None:
            return
        self.goal = None
        self.goal_truth.set_data([], [])
        self.goal_disc.set_data([], [])
        print("[GOAL] 해제")

    # ---- detections (discovered) ---------------------------------------
    def add_detections(self, world_coords) -> None:
        """레이캐스트로 새로 본 장애물 월드 좌표들을 discovered 맵에 누적.

        Args:
            world_coords: iterable of (wx, wy) tuples — 이번 프레임 감지 위치들.
        """
        added = False
        step = self._disc_grid_step
        for wx, wy in world_coords:
            ci = int(round(wx / step))
            cj = int(round(wy / step))
            if (ci, cj) not in self._disc_cells:
                self._disc_cells.add((ci, cj))
                added = True
        if not added:
            return
        # 누적 셀들을 월드 좌표 점들로 환산 → scatter 갱신.
        pts = np.array(
            [[ci * step, cj * step] for ci, cj in self._disc_cells],
            dtype=float,
        )
        self._disc_scatter.set_offsets(pts)

    def clear_detections(self) -> None:
        """discovered 맵 누적 감지 초기화 — env 리셋 시 호출 가능."""
        self._disc_cells.clear()
        self._disc_scatter.set_offsets(np.empty((0, 2), dtype=float))

    # ---- update ---------------------------------------------------------
    def update(self, x: float, y: float, yaw: float) -> None:
        """차량 마커·heading 화살표를 새 pose 로 갱신하고 GUI 이벤트를 흘린다."""
        dx = 1.5 * math.cos(yaw)
        dy = 1.5 * math.sin(yaw)

        self.veh_truth.set_data([x], [y])
        if self._veh_arrow_truth is not None:
            self._veh_arrow_truth.remove()
        self._veh_arrow_truth = self.ax_truth.arrow(
            x, y, dx, dy,
            head_width=0.6, head_length=0.4, fc="b", ec="b",
            length_includes_head=True,
        )

        self.veh_disc.set_data([x], [y])
        if self._veh_arrow_disc is not None:
            self._veh_arrow_disc.remove()
        self._veh_arrow_disc = self.ax_disc.arrow(
            x, y, dx, dy,
            head_width=0.6, head_length=0.4, fc="b", ec="b",
            length_includes_head=True,
        )

        try:
            self.fig.canvas.flush_events()
        except Exception:
            # 사용자가 창을 닫은 경우 — 다음 is_alive() 호출에서 처리.
            pass

    def is_alive(self) -> bool:
        """맵 창이 살아 있는지 — 사용자가 ✕ 로 닫으면 False."""
        return plt.fignum_exists(self.fig.number)
