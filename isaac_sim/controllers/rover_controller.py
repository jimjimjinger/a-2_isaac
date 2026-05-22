# rover_controller.py
#
# ROS2 없이 Isaac Sim 내부에서 rover articulation DOF를 직접 테스트하는 스크립트.
# rover_v2 scene을 우선 로드하고, DOF 이름을 자동 감지해 drive/steer를 분류한다.
# Isaac Python 3.11 — rclpy/geometry_msgs 절대 import 금지.
#
# 실행 방법:
#   cd ~/dev_ws/rover_ws/src/a2_isaac/isaac_sim/controllers
#   isaac-python rover_controller.py

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.usd
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from isaacsim.core.utils.stage import add_reference_to_stage
from pathlib import Path

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
SCENE_PRIM_PATH = "/World"
ROVER_PRIM_CANDIDATES = ("/World/Vehicle/rover", "/Vehicle/rover")
TEST_LINEAR_SPEED = 1.0
TEST_ANGULAR_STEER = 0.0
WHEEL_VELOCITY_SCALE = 10.0
STEER_ANGLE_SCALE = 0.5

_REPO_ROOT = Path(__file__).resolve().parents[2]
ROVER_SCENE_USD_PATH = _REPO_ROOT / "isaac_sim" / "assets" / "rover_v2" / "rover_m0609_localization.usd"

# ---------------------------------------------------------------------------
# DOF classification
# ---------------------------------------------------------------------------
_DRIVE_KEYWORDS = ("drive", "wheel")
_STEER_KEYWORDS = ("steer", "steering")


def _classify_dofs(dof_names):
    drive, steer = [], []
    for name in dof_names:
        lower = name.lower()
        if any(kw in lower for kw in _DRIVE_KEYWORDS):
            drive.append(name)
        elif any(kw in lower for kw in _STEER_KEYWORDS):
            steer.append(name)
    return drive, steer


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------
class RoverController:

    def __init__(self):
        self.world = World()

        stage = omni.usd.get_context().get_stage()
        stage.DefinePrim(SCENE_PRIM_PATH, "Xform")
        add_reference_to_stage(usd_path=str(ROVER_SCENE_USD_PATH), prim_path=SCENE_PRIM_PATH)
        for _ in range(10):
            simulation_app.update()

        rover_prim = None
        for prim_path in ROVER_PRIM_CANDIDATES:
            candidate = stage.GetPrimAtPath(prim_path)
            if candidate.IsValid():
                rover_prim = candidate
                self.rover_prim_path = prim_path
                break
        if rover_prim is None:
            raise RuntimeError(
                "Rover prim을 찾지 못했습니다. "
                f"scene={ROVER_SCENE_USD_PATH} candidates={ROVER_PRIM_CANDIDATES}"
            )

        self.rover = Articulation(self.rover_prim_path)
        self.world.scene.add(self.rover)

        self.drive_dofs: list[str] = []
        self.steer_dofs: list[str] = []
        self.drive_indices: list[int] = []
        self.steer_indices: list[int] = []

    def init_dofs(self):
        """world.reset() 이후 호출 — DOF 이름이 확정된 시점에서 분류."""
        all_dofs = list(self.rover.dof_names)

        print(f"\n{'='*60}")
        print(f"[Rover] Prim path  : {self.rover_prim_path}")
        print(f"[Rover] All DOFs   : {all_dofs}")

        drive_candidates, steer_candidates = _classify_dofs(all_dofs)

        for name in drive_candidates:
            try:
                idx = self.rover.get_dof_index(name)
                self.drive_dofs.append(name)
                self.drive_indices.append(idx)
            except Exception as e:
                print(f"[WARN] drive DOF '{name}' 인덱스 획득 실패: {e}")

        for name in steer_candidates:
            try:
                idx = self.rover.get_dof_index(name)
                self.steer_dofs.append(name)
                self.steer_indices.append(idx)
            except Exception as e:
                print(f"[WARN] steer DOF '{name}' 인덱스 획득 실패: {e}")

        print(f"[Rover] Drive DOFs : {self.drive_dofs}")
        print(f"[Rover] Steer DOFs : {self.steer_dofs}")

        if not self.drive_dofs and not self.steer_dofs:
            print(
                "[Rover] WARNING: drive/steer DOF를 하나도 찾지 못했습니다.\n"
                "        DOF 이름에 drive/wheel/steer/steering 키워드가 있는지 확인하세요."
            )
        print(f"{'='*60}\n")

    def update(self):
        wheel_velocity = TEST_LINEAR_SPEED * WHEEL_VELOCITY_SCALE
        steer_angle = TEST_ANGULAR_STEER * STEER_ANGLE_SCALE

        articulation_ctrl = self.rover.get_articulation_controller()

        if self.drive_indices:
            articulation_ctrl.apply_action(ArticulationAction(
                joint_velocities=[wheel_velocity] * len(self.drive_indices),
                joint_indices=self.drive_indices,
            ))

        if self.steer_indices:
            articulation_ctrl.apply_action(ArticulationAction(
                joint_positions=[steer_angle] * len(self.steer_indices),
                joint_indices=self.steer_indices,
            ))

        print(
            f"\r[update] wheel_vel={wheel_velocity:.2f}  "
            f"steer_angle={steer_angle:.3f}  "
            f"drive_dofs={len(self.drive_indices)}  "
            f"steer_dofs={len(self.steer_indices)}",
            end="",
            flush=True,
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
ctrl = RoverController()
ctrl.world.reset()
ctrl.init_dofs()

while simulation_app.is_running():
    ctrl.update()
    ctrl.world.step(render=True)

print()
simulation_app.close()
