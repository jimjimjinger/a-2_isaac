"""M0609 home pose probe.

pickup_demo.py 의 HOME_JOINT_DEG 를 결정하기 위해, 씬을 1회 build 한 뒤
여러 candidate joint pose 를 순차 적용해서 tool0 (gripper tip) 의 world
position 을 측정 / 큐브까지의 거리 출력.

큐브 위치: build_rover_m0609_scene.py / pickup_demo.py 와 동일하게
(SPAWN_X+0.5, 0, SPAWN_Z-0.30+CUBE_SIZE/2) = (5.5, 0, 0.025).

가장 좋은 pose = tool0 가 큐브 바로 위 (5.5, 0, ~0.1) 근처에 오는 것.

실행:
    isaac-python ~/dev_ws/rover_ws/src/a2_isaac/isaac_manipulation/scripts/find_home_pose.py
"""
from __future__ import annotations

import math
import os
import sys
import tempfile
import time
from pathlib import Path

try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

_TMP_CWD = tempfile.mkdtemp(prefix="isaac_urdf_")
os.chdir(_TMP_CWD)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import omni.kit.app
import omni.kit.commands
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

_carb = carb.settings.get_settings()
for ch in (
    "/log/channels/isaacsim.core.simulation_manager.plugin",
    "/log/channels/omni.physx.tensors.plugin",
):
    try:
        _carb.set(ch, "Error")
    except Exception:
        pass

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.robot_setup.assembler", True
)

from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_setup.assembler import RobotAssembler


A2_ROOT = Path(
    os.environ.get("A2_ISAAC_ROOT") or Path(__file__).resolve().parents[2]
)
MARS_WORLD_USD = A2_ROOT / "isaac_sim/worlds/mars_exploration_world.usd"
ROVER_USD = A2_ROOT / "isaac_sim/assets/rover/Mars_Rover.usd"
M0609_URDF = A2_ROOT / "isaac_sim/assets/doosan-robot2/urdf/m0609_isaac_sim.urdf"
RG2_URDF = A2_ROOT / "isaac_sim/assets/onrobot_rg2/urdf/onrobot_rg2.urdf"

SPAWN_X, SPAWN_Y, SPAWN_Z = 5.0, 0.0, 0.30
M0609_MOUNT_OFFSET_X = 0.15274
M0609_MOUNT_OFFSET_Y = 0.0
M0609_MOUNT_OFFSET_Z = 0.21232

CUBE_SIZE = 0.05
CUBE_POS = (SPAWN_X + 0.5, SPAWN_Y, SPAWN_Z - 0.30 + CUBE_SIZE / 2)
TARGET_TOOL0 = (CUBE_POS[0], CUBE_POS[1], CUBE_POS[2] + 0.10)  # 큐브 위 10cm


# ── candidate pose 들 (필요시 편집해서 재실행) ──────────────────────
# 각 항목: (label, (j1, j2, j3, j4, j5, j6) in degrees)
CANDIDATE_POSES = [
    # 1차 round (참고): horizontal fwd 가 best (dist 0.75)
    ("horizontal (ref)",  (0.0,  90.0,   0.0,  0.0,   0.0,   0.0)),
    # curl-down: 어깨 fwd + 팔꿈치 같은 방향 fold → 팔이 아래로 떨어지길 기대
    ("curl 90/+90",       (0.0,  90.0,  90.0,  0.0,   0.0,   0.0)),
    ("curl 90/+90 w5",    (0.0,  90.0,  90.0,  0.0,  90.0,   0.0)),
    ("curl 90/+90 w-90",  (0.0,  90.0,  90.0,  0.0, -90.0,   0.0)),
    ("curl 60/+90",       (0.0,  60.0,  90.0,  0.0,   0.0,   0.0)),
    ("curl 60/+90 w30",   (0.0,  60.0,  90.0,  0.0,  30.0,   0.0)),
    ("curl 45/+90 w45",   (0.0,  45.0,  90.0,  0.0,  45.0,   0.0)),
    ("curl 45/+135",      (0.0,  45.0, 135.0,  0.0,   0.0,   0.0)),
    ("curl 30/+120",      (0.0,  30.0, 120.0,  0.0,  30.0,   0.0)),
    ("hard curl 60/+135", (0.0,  60.0, 135.0,  0.0,  45.0,   0.0)),
    ("hard curl 90/+135", (0.0,  90.0, 135.0,  0.0,  45.0,   0.0)),
    # 큰 wrist pitch 로 손목만 내리는 방향
    ("horizontal + w90",  (0.0,  90.0,   0.0,  0.0,  90.0,   0.0)),
    ("horizontal + w-90", (0.0,  90.0,   0.0,  0.0, -90.0,   0.0)),
    ("h-fwd + w+45",      (0.0,  60.0,   0.0,  0.0,  90.0,   0.0)),
]


def _import_urdf(urdf_path: str, fix_base: bool) -> str:
    _, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = True
    import_config.import_inertia_tensor = True
    import_config.fix_base = fix_base
    import_config.distance_scale = 1.0
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.default_drive_strength = 1e10
    import_config.default_position_drive_damping = 1e5
    _, artic_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path, import_config=import_config, get_articulation_root=True,
    )
    return artic_path.rsplit("/", 1)[0] or artic_path


def _find_prim_path_by_name(root_path: str, link_name: str):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == link_name:
            return str(prim.GetPath())
    return None


def _assemble(robot_base, robot_base_mount, robot_attach, robot_attach_mount,
              namespace, variant):
    stage = omni.usd.get_context().get_stage()
    root_target = Usd.EditTarget(stage.GetRootLayer())
    assembler = RobotAssembler()
    assembler.begin_assembly(stage, robot_base, robot_base_mount, robot_attach,
                             robot_attach_mount, namespace, variant)
    assembler.assemble()
    assembler.finish_assemble()
    stage.SetEditTarget(root_target)


def _freeze_rover_drives(rover_prim) -> int:
    n = 0
    for prim in Usd.PrimRange(rover_prim):
        for drv_type in ("angular", "linear"):
            drv = UsdPhysics.DriveAPI.Get(prim, drv_type)
            if drv:
                drv.GetTargetPositionAttr().Set(0.0)
                drv.GetTargetVelocityAttr().Set(0.0)
                drv.GetStiffnessAttr().Set(1e8)
                drv.GetDampingAttr().Set(1e6)
                drv.GetMaxForceAttr().Set(1e7)
                n += 1
    return n


def _set_m0609_pose(stage, robot_root: str, joint_angles_deg) -> int:
    targets = {f"joint_{i+1}": float(np.deg2rad(joint_angles_deg[i]))
               for i in range(min(6, len(joint_angles_deg)))}
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_root)):
        if prim.GetName() in targets:
            drv = UsdPhysics.DriveAPI.Get(prim, "angular")
            if drv:
                drv.GetTargetPositionAttr().Set(targets[prim.GetName()])
                n += 1
    return n


def _tool0_world(stage, robot_root: str):
    tp_path = _find_prim_path_by_name(robot_root, "tool0")
    if not tp_path:
        return None
    tp = stage.GetPrimAtPath(tp_path)
    if not tp.IsValid():
        return None
    t = UsdGeom.Xformable(tp).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()).ExtractTranslation()
    return (float(t[0]), float(t[1]), float(t[2]))


def build_scene():
    stage = omni.usd.get_context().get_stage()

    print("[1/5] mars world …")
    mars_prim = stage.DefinePrim("/World/Mars", "Xform")
    mars_prim.GetReferences().AddReference(str(MARS_WORLD_USD))
    for _ in range(8):
        simulation_app.update()
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene") \
        .CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    UsdPhysics.Scene(stage.GetPrimAtPath("/World/PhysicsScene")) \
        .CreateGravityMagnitudeAttr().Set(3.72)

    print("[2/5] rover …")
    rover_path = "/World/Vehicle/rover"
    stage.DefinePrim("/World/Vehicle", "Xform")
    rover_prim = stage.DefinePrim(rover_path, "Xform")
    rover_prim.GetReferences().AddReference(str(ROVER_USD))
    xform = UsdGeom.Xformable(rover_prim)
    xform.ClearXformOpOrder()
    for op_attr in ("xformOp:translate", "xformOp:orient", "xformOp:scale",
                    "xformOp:rotateXYZ", "xformOp:rotateZYX"):
        if rover_prim.GetAttribute(op_attr).IsValid():
            rover_prim.RemoveProperty(op_attr)
    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(SPAWN_X, SPAWN_Y, SPAWN_Z))
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
    xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1, 1, 1))
    for _ in range(8):
        simulation_app.update()
    _freeze_rover_drives(rover_prim)

    print("[3/5] M0609 URDF …")
    m0609_world = (SPAWN_X + M0609_MOUNT_OFFSET_X,
                   SPAWN_Y + M0609_MOUNT_OFFSET_Y,
                   SPAWN_Z + M0609_MOUNT_OFFSET_Z)
    robot_root = _import_urdf(str(M0609_URDF), fix_base=False)
    omni.kit.commands.execute("TransformPrimSRTCommand",
                              path=robot_root,
                              new_translation=Gf.Vec3d(*m0609_world),
                              new_rotation_euler=Gf.Vec3d(0, 0, 0),
                              new_scale=Gf.Vec3d(1, 1, 1))
    for _ in range(8):
        simulation_app.update()

    print("[4/5] RG2 + assemble …")
    gripper_root = _import_urdf(str(RG2_URDF), fix_base=False)
    robot_ee = _find_prim_path_by_name(robot_root, "link_6") or f"{robot_root}/link_6"
    gripper_base = _find_prim_path_by_name(gripper_root, "angle_bracket") \
                   or f"{gripper_root}/angle_bracket"
    _assemble(robot_root, robot_ee, gripper_root, gripper_base, "Gripper", "m0609_rg2")

    print("[5/5] rover ↔ M0609 mount + anchor …")
    m0609_base = _find_prim_path_by_name(robot_root, "base_link") or f"{robot_root}/base_link"
    rover_body = _find_prim_path_by_name(rover_path, "Body") or f"{rover_path}/Body"
    stage.SetEditTarget(Usd.EditTarget(stage.GetRootLayer()))
    mount_path = f"{rover_body}/M0609_Mount"
    mount_prim = stage.DefinePrim(mount_path, "Xform")
    UsdGeom.Xformable(mount_prim).AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(M0609_MOUNT_OFFSET_X, M0609_MOUNT_OFFSET_Y, M0609_MOUNT_OFFSET_Z)
    )
    for _ in range(5):
        simulation_app.update()
    _assemble(robot_root, m0609_base, rover_path, mount_path, "RoverMount", "m0609_on_rover")
    _freeze_rover_drives(rover_prim)

    # rover anchor → 자유낙하 방지
    stage.SetEditTarget(Usd.EditTarget(stage.GetRootLayer()))
    stage.DefinePrim("/World/Joints", "Scope")
    anchor = UsdPhysics.FixedJoint.Define(stage, "/World/Joints/RoverAnchor")
    anchor.CreateBody1Rel().SetTargets([Sdf.Path(rover_body)])
    anchor.CreateLocalPos0Attr().Set(Gf.Vec3f(SPAWN_X, SPAWN_Y, SPAWN_Z))
    anchor.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    anchor.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    anchor.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    anchor.CreateBreakForceAttr().Set(float('inf'))
    anchor.CreateBreakTorqueAttr().Set(float('inf'))

    for _ in range(10):
        simulation_app.update()
    return rover_prim, robot_root


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def main():
    try:
        _probe()
    except Exception as e:
        import traceback
        print(f"\n[FATAL] {e}", flush=True)
        traceback.print_exc()
    finally:
        try:
            simulation_app.close()
        except Exception:
            pass
        os._exit(0)


def _probe():
    world = World(stage_units_in_meters=1.0)
    rover_prim, robot_root = build_scene()

    # ArticulationController 사용 — play 후 USD drive write 는 PhysX 에 안 닿음.
    # World.scene 에 등록 → world.reset 후 articulation handle 활성화.
    m0609_art = SingleArticulation(prim_path=robot_root, name="m0609_art")
    world.scene.add(m0609_art)

    print("\n[World] reset + play …")
    world.reset()
    _freeze_rover_drives(rover_prim)
    stage = omni.usd.get_context().get_stage()
    world.play()
    for _ in range(15):
        world.step(render=True)

    dof_names = list(m0609_art.dof_names) if m0609_art.dof_names is not None else []
    print(f"  [art] DOF count={len(dof_names)}")
    print(f"  [art] DOF names: {dof_names}")
    target_indices = []
    for i in range(6):
        name = f"joint_{i+1}"
        if name in dof_names:
            target_indices.append(dof_names.index(name))
        else:
            print(f"  [WARN] '{name}' not in dof_names")
    if len(target_indices) != 6:
        print(f"[FATAL] joint_1~6 모두 못 찾음 — indices={target_indices}")
        return
    print(f"  [art] joint_1~6 indices = {target_indices}")
    controller = m0609_art.get_articulation_controller()

    print(f"\nTarget tool0 ≈ {TARGET_TOOL0}   (큐브 {CUBE_POS} 의 +10cm 위)")
    print("-" * 92)
    print(f"{'label':24s} {'pose target (deg)':32s} {'joints actual (deg)':28s} {'tool0 (m)':22s} {'dist':>6s}")
    print("-" * 92)

    results = []
    for label, pose in CANDIDATE_POSES:
        if not simulation_app.is_running():
            break
        rad_pose = np.array([np.deg2rad(v) for v in pose[:6]], dtype=np.float32)
        # apply_action: drive target 변경 (fabric write — play 후에도 PhysX 가 읽음)
        action = ArticulationAction(
            joint_positions=rad_pose,
            joint_indices=np.array(target_indices, dtype=np.int32),
        )
        controller.apply_action(action)
        # 추가: set_joint_positions 으로 즉시 텔레포트 (FK 빠르게 측정)
        try:
            cur = m0609_art.get_joint_positions()
            if cur is not None:
                cur = np.array(cur, dtype=np.float32).copy()
                for k, idx in enumerate(target_indices):
                    cur[idx] = rad_pose[k]
                m0609_art.set_joint_positions(cur)
        except Exception as e:
            print(f"  [warn] set_joint_positions failed: {e}")
        # 짧게 step (transform propagation)
        for _ in range(30):
            world.step(render=True)

        tw = _tool0_world(stage, robot_root)
        cur = m0609_art.get_joint_positions()
        actual_deg = (
            tuple(round(float(np.rad2deg(cur[idx])), 0) for idx in target_indices)
            if cur is not None else (None,) * 6
        )
        if tw is None:
            tw_str = "(tool0 NOT FOUND)"
            d = float("inf")
        else:
            tw_str = f"({tw[0]:+.2f},{tw[1]:+.2f},{tw[2]:+.2f})"
            d = _dist(tw, TARGET_TOOL0)
        target_str = ",".join(f"{v:+.0f}" for v in pose)
        actual_str = ",".join(f"{v:+.0f}" if v is not None else "?" for v in actual_deg)
        print(f"{label:24s} ({target_str:30s}) ({actual_str:26s}) {tw_str:22s} {d:6.3f}")
        results.append((d, label, pose, tw))

    print("-" * 78)
    print("\n[BEST 3] (가까운 순)")
    results.sort(key=lambda r: r[0])
    for d, label, pose, tw in results[:3]:
        pose_str = ",".join(f"{v:+.0f}" for v in pose)
        tw_str = "n/a" if tw is None else f"({tw[0]:+.2f},{tw[1]:+.2f},{tw[2]:+.2f})"
        print(f"  dist={d:.3f}m  {label}")
        print(f"     pose = ({pose_str})")
        print(f"     tool0 = {tw_str}")

    print("\n[NEXT] 위 best 의 pose 를 pickup_demo.py HOME_JOINT_DEG 로 넣고,")
    print("       (또는 dist 가 충분히 작은 pose 가 없으면 candidate 리스트를 편집해서")
    print("       재실행) — 1m 이상 떨어지면 신뢰성 낮음. 0.05~0.2m 이면 좋음.\n")

    # 사용자 시각 확인용으로 마지막 best pose 유지 후 잠시 대기
    if results:
        best = min(results, key=lambda r: r[0])
        rad_best = np.array([np.deg2rad(v) for v in best[2][:6]], dtype=np.float32)
        action = ArticulationAction(
            joint_positions=rad_best,
            joint_indices=np.array(target_indices, dtype=np.int32),
        )
        controller.apply_action(action)
        try:
            cur = m0609_art.get_joint_positions()
            if cur is not None:
                cur = np.array(cur, dtype=np.float32).copy()
                for k, idx in enumerate(target_indices):
                    cur[idx] = rad_best[k]
                m0609_art.set_joint_positions(cur)
        except Exception:
            pass
        print(f"[VIEW] best pose ({best[1]}) 유지 중 — 10초 후 종료")
        t0 = time.time()
        while simulation_app.is_running() and (time.time() - t0) < 10.0:
            world.step(render=True)


if __name__ == "__main__":
    main()
