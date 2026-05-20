"""실제 맵(mars_exploration_world)에서 BCD coverage sweep 테스트.

isaac_drive/navigation 모듈을 terrain_00001 자산 + master scene USD 에
연결해 자율 sweep 을 돌린다. ROS2 노드가 아닌 독립 실행 스크립트.

흐름:
  1. terrain_00001 의 meta.json + obstacle_grid.npy 로드 (numpy)
  2. Isaac Sim World + mars_exploration_world.usd (master scene) 로드
  3. 로버 spawn (basecamp 중심)
  4. SectorPlanner + Navigator + Mission FSM 연결
  5. viewer 프로세스 시작
  6. 시뮬 루프: pose → fog reveal → mission step → drive

실행 (Isaac Sim 파이썬으로):
    cd .../isaac_drive/scripts
    <isaac python> run_coverage_test.py
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import os
import sys

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage

# isaac_drive 패키지 + 같은 폴더 스크립트 import 가능하게
HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)            # .../isaac_drive
sys.path.insert(0, PKG_ROOT)
sys.path.insert(0, HERE)

from isaac_drive.navigation.terrain_loader import load_terrain
from isaac_drive.navigation.coverage_planner import SectorPlanner
from isaac_drive.navigation.navigator import Navigator
from isaac_drive.navigation.mission_fsm import Mission

from rover import RoverController
from state_writer import StateWriter

# ── 경로 ── (repo 루트 = run_coverage_test.py 기준 상대경로)
WS          = os.path.dirname(PKG_ROOT)     # .../a2_isaac
MARS_WORLD  = f"{WS}/isaac_sim/worlds/mars_exploration_world.usd"
TERRAIN_DIR = f"{WS}/isaac_sim/assets/generated_terrains/terrain_00001"
VIEWER      = os.path.join(HERE, "viewer.py")

# ── 설정 (물리/센서 상수 + 격자 해상도) ──
ROBOT_RADIUS  = 0.7    # 장애물 inflate 반경 (m). 로버 ~0.62m + 마진
REVEAL_RADIUS = 2.0    # 센서 reveal 반경 (m)
CELL_SIZE     = 0.1    # navigation 격자 해상도 (m/cell). raw 0.05 → 다운샘플
GRID_N        = 3      # sector 3×3 = 9구역

# 로버 초기 spawn 위치 (x, y, z) — Isaac Sim GUI 에서 확인한 좌표
SPAWN_POSITION = (-0.2182, 3.87964, 0.2)


def main():
    # ── 1) terrain 데이터 로드 (Isaac Sim 무관, numpy) ──
    meta, ogrid, fog = load_terrain(
        TERRAIN_DIR, cell_size=CELL_SIZE, robot_radius=ROBOT_RADIUS,
        reveal_radius=REVEAL_RADIUS, grid_n=GRID_N,
    )
    print(f"[run_coverage] 맵 {fog.map_w:.0f}×{fog.map_h:.0f}m, "
          f"basecamp={meta['basecamp']['center']}")

    # ── 2) Isaac Sim World + master scene USD ──
    my_world = World(stage_units_in_meters=1.0)
    add_reference_to_stage(usd_path=MARS_WORLD, prim_path="/World/MarsScene")
    print(f"[run_coverage] master scene 로드: {MARS_WORLD}")

    # ── 3) 로버 spawn (GUI 에서 확인한 고정 좌표) ──
    rover = RoverController(my_world)
    rover.spawn(initial_position=SPAWN_POSITION)
    for _ in range(10):
        simulation_app.update()
    rover.attach_camera()
    for _ in range(5):
        simulation_app.update()

    my_world.reset()
    rover.initialize()

    # ── 4) navigation 모듈 ──
    planner = SectorPlanner(fog, ogrid, reveal_radius=REVEAL_RADIUS)
    navigator = Navigator(
        rover, waypoint_tol=0.2, final_tol=0.3,
        kp_ang=2.0, max_lin=7.0, max_ang=1.5, point_turn_deg=45,
    )
    mission = Mission(
        fog, ogrid, planner, navigator, rover, sector_done_ratio=0.95,
    )

    # ── 5) viewer 시작 ──
    writer = StateWriter(fog, viewer_script_path=VIEWER, write_every=3)

    # ── 6) 시뮬 루프 ──
    print("\n[run_coverage] 시뮬레이션 시작 — 실제 맵 BCD sweep\n")
    try:
        while simulation_app.is_running():
            my_world.step(render=True)
            if not my_world.is_playing():
                continue

            cx, cy, yaw = rover.get_pose_2d()
            fog.reveal_around(cx, cy)

            if mission.is_done():
                lin_vel, ang_vel = 0.0, 0.0
            else:
                lin_vel, ang_vel = mission.update(
                    my_world.current_time_step_index)
            rover.drive(lin_vel, ang_vel)

            writer.maybe_write(
                my_world.current_time_step_index, (cx, cy, yaw), mission)

            if my_world.current_time_step_index % 60 == 0:
                ratios = fog.all_sector_ratios()
                rstr = " ".join(f"{r*100:.0f}%" for r in ratios)
                print(f"[step {my_world.current_time_step_index}] "
                      f"pos=({cx:+.2f},{cy:+.2f}) "
                      f"sector={mission.current_sector + 1} "
                      f"state={mission.state} "
                      f"전체={fog.overall_ratio()*100:.1f}% [{rstr}]")
    finally:
        writer.close()
        simulation_app.close()


if __name__ == "__main__":
    main()
