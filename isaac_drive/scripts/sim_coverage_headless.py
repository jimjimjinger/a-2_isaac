"""실제 맵 coverage 알고리즘 headless 검증 (Isaac Sim 없이).

terrain_00001 자산 + navigation 모듈로 BCD sweep 을 운동학 시뮬레이션하고
결과를 matplotlib PNG 로 저장한다. Isaac Sim 물리 없이 알고리즘만 빠르게
검증 — "Vacuum Cleaner First" (T3_BRIEF §3) 검증 단계에 해당.

실행:
    cd .../isaac_drive
    python3 scripts/sim_coverage_headless.py
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))   # isaac_drive 패키지

from isaac_drive.navigation.terrain_loader import load_terrain
from isaac_drive.navigation.coverage_planner import SectorPlanner
from isaac_drive.navigation.navigator import Navigator
from isaac_drive.navigation.mission_fsm import Mission

WS = "/home/rokey/dev_ws/rover_ws/src/a2_isaac"
TERRAIN_DIR = f"{WS}/isaac_sim/assets/generated_terrains/terrain_00001"

ROBOT_RADIUS  = 0.7
REVEAL_RADIUS = 2.0
CELL_SIZE     = 0.1
GRID_N        = 3
DT            = 1.0 / 60.0      # Isaac Sim 기본 step
MAX_STEPS     = 120000


class KinematicRover:
    """(lin, ang) 으로 pose 를 적분하는 mock 로버 (unicycle 운동학).

    Isaac Sim 물리 없이 navigator/mission 을 검증하기 위한 것.
    RoverController 와 같은 get_pose_2d / drive 인터페이스를 제공.
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, dt=DT):
        self.x, self.y, self.yaw, self.dt = x, y, yaw, dt

    def get_pose_2d(self):
        return self.x, self.y, self.yaw

    def drive(self, lin, ang):
        self.yaw += ang * self.dt
        self.x += lin * np.cos(self.yaw) * self.dt
        self.y += lin * np.sin(self.yaw) * self.dt


def main():
    meta, ogrid, fog = load_terrain(
        TERRAIN_DIR, cell_size=CELL_SIZE, robot_radius=ROBOT_RADIUS,
        reveal_radius=REVEAL_RADIUS, grid_n=GRID_N,
    )

    rover = KinematicRover(0.0, 0.0, 0.0)
    planner = SectorPlanner(fog, ogrid, reveal_radius=REVEAL_RADIUS)
    navigator = Navigator(
        rover, waypoint_tol=0.2, final_tol=0.3,
        kp_ang=2.0, max_lin=7.0, max_ang=1.5, point_turn_deg=45,
    )
    mission = Mission(
        fog, ogrid, planner, navigator, rover, sector_done_ratio=0.95,
    )

    traj = [(rover.x, rover.y)]
    final_step = MAX_STEPS
    for step in range(MAX_STEPS):
        cx, cy, _ = rover.get_pose_2d()
        fog.reveal_around(cx, cy)
        if mission.is_done():
            final_step = step
            print(f"[headless] 미션 완료 @ step {step}")
            break
        lin, ang = mission.update(step)
        rover.drive(lin, ang)
        traj.append((rover.x, rover.y))
        if step % 5000 == 0:
            print(f"[headless] step {step:6d}: "
                  f"pos=({rover.x:+6.1f},{rover.y:+6.1f}) "
                  f"sector={mission.current_sector + 1} "
                  f"state={mission.state:11s} "
                  f"reveal={fog.overall_ratio() * 100:5.1f}%")
    else:
        print("[headless] MAX_STEPS 도달 (미션 미완)")

    print(f"\n[headless] 최종 reveal {fog.overall_ratio() * 100:.1f}%  "
          f"({final_step} step, 이동 ~{len(traj) * 7.0 * DT:.0f}m)")
    ratios = fog.all_sector_ratios()
    print("[headless] 구역별: "
          + " ".join(f"S{i+1}={r*100:.0f}%" for i, r in enumerate(ratios)))

    # ── 시각화 ──
    W, H = fog.map_w, fog.map_h
    ext = [-W / 2, W / 2, -H / 2, H / 2]
    fig, ax = plt.subplots(figsize=(11, 11))
    ax.imshow(fog.fog, extent=ext, origin="lower", cmap="gray",
              vmin=0, vmax=1, alpha=0.85)
    obs_m = np.ma.masked_where(fog.obstacle_mask == 0, fog.obstacle_mask)
    ax.imshow(obs_m, extent=ext, origin="lower",
              cmap=ListedColormap(["orangered"]), alpha=0.85)

    tx = [p[0] for p in traj]
    ty = [p[1] for p in traj]
    ax.plot(tx, ty, "-", color="deepskyblue", linewidth=0.7, alpha=0.7)
    ax.plot([0], [0], "*", color="gold", markersize=22,
            markeredgecolor="black", zorder=10)

    for k in range(1, GRID_N):
        ax.axvline(-W / 2 + k * W / GRID_N, color="dimgray", ls="--", lw=0.8)
        ax.axhline(-H / 2 + k * H / GRID_N, color="dimgray", ls="--", lw=0.8)

    ax.set_xlim(-W / 2, W / 2)
    ax.set_ylim(-H / 2, H / 2)
    ax.set_aspect("equal")
    ax.set_title(f"headless coverage — terrain_00001 — "
                 f"reveal {fog.overall_ratio() * 100:.1f}%  ({final_step} step)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")

    out = os.path.join(HERE, "coverage_headless_result.png")
    plt.savefig(out, dpi=100, bbox_inches="tight")
    print(f"[headless] saved -> {out}")


if __name__ == "__main__":
    main()
