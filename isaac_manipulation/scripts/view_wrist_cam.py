"""Wrist-camera live viewer + Lissajous cyan-cube search.

(1) 그리퍼 (RG2 angle_bracket) 에 달린 손목 카메라를 Isaac Sim 안의 **별도
    viewport 패널** (`wrist_camera`) 으로 띄움. 메인 viewport 는 월드 뷰
    그대로 유지. 패널을 드래그해 떼어내면 OS 타이틀바 가진 독립 창이 됨.
(2) 그리퍼가 HOME pose 기준으로 j1 (base yaw), j2 (shoulder pitch) 를 다른
    주파수의 sin 으로 진동 → Lissajous 궤적으로 X-Y 평면을 훑으며 cyan 큐브
    탐색. wrist 카메라 영상에서 HSV 검출 area 가 threshold 넘으면 'FOUND'
    phase 로 전환, 현재 pose 를 잠시 유지 후 다시 스캔 재개.

* cv2.imshow 는 Isaac Sim 의 cv2 가 headless 빌드라 화면이 안 뜸. cv2 자체는
  HSV 처리에 사용 (imshow 없이). 디스플레이는 wrist_camera viewport 패널.

State machine:
    scan  → cyan 검출 (area ≥ DETECTION_MIN_AREA_PX2) 시 → found
    found → FOUND_HOLD_SEC 초 정지 → scan 재개

실행:
    isaac-python ~/dev_ws/rover_ws/src/a2_isaac/isaac_manipulation/scripts/view_wrist_cam.py
옵션:
    --no-cube        시안 큐브 spawn 생략
    --no-play        Play 자동 시작 안 함 (Spacebar 로 수동)
    --pose 0,0,90,0,90,0  joint_1~6 (deg) base pose (j1/j2 는 여기 기준 진동)
    --cam-xyz x,y,z       angle_bracket 기준 카메라 translate (m)
    --cam-rpy r,p,y       angle_bracket 기준 카메라 RotateXYZ (deg)
"""
from __future__ import annotations

import argparse
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

_ext = omni.kit.app.get_app().get_extension_manager()
_ext.set_extension_enabled_immediate("isaacsim.robot_setup.assembler", True)
# viewport window 확장이 필요 — 별도 viewport 패널 (wrist_camera 창) 생성용
_ext.set_extension_enabled_immediate("omni.kit.viewport.window", True)
_ext.set_extension_enabled_immediate("omni.kit.viewport.utility", True)

from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_setup.assembler import RobotAssembler

_PKG_PARENT = "/home/rokey/dev_ws/rover_ws/src/a2_isaac"
sys.path.insert(0, f"{_PKG_PARENT}/isaac_perception")
from isaac_perception.cyan_detector import CyanDetector


A2_ROOT = Path(_PKG_PARENT)
MARS_WORLD_USD = A2_ROOT / "isaac_sim/worlds/mars_exploration_world.usd"
ROVER_USD = A2_ROOT / "isaac_sim/assets/vehicle/legacy/rover/Mars_Rover.usd"
M0609_URDF = A2_ROOT / "isaac_sim/assets/doosan-robot2/urdf/m0609_isaac_sim.urdf"
RG2_URDF = A2_ROOT / "isaac_sim/assets/onrobot_rg2/urdf/onrobot_rg2.urdf"

SPAWN_X, SPAWN_Y, SPAWN_Z = 5.0, 0.0, 0.30
M0609_MOUNT_OFFSET = (0.15274, 0.0, 0.21232)

CUBE_SIZE = 0.10                      # 88cm 거리에서도 픽셀 충분히
# 큐브 spawn 위치 — spawn 직후 FixedJoint 로 즉시 anchor (낙하/굴러감 없음).
# z=0.4 → 카메라 (z≈0.885) 와 terrain (보통 < 0.3) 사이에 고정. 공중에 떠 있어도
# camera/탐색 테스트엔 무방. terrain 안 묻힘 보장.
CUBE_POS = (SPAWN_X + 0.7, SPAWN_Y, 0.4)
CUBE_COLOR = (0.0, 1.0, 1.0)

CAMERA_RESOLUTION = (640, 480)
# 카메라는 RG2 angle_bracket 의 자식 Xform. angle_bracket 의 +Z 가 그리퍼
# fingers 방향이라 HOME pose 에서 world -Z (아래) 를 가리킴. USD 카메라는 본인
# -Z 방향을 보므로, 카메라의 -Z 를 parent 의 +Z 에 맞추려면 Rx(180°) 로 뒤집어야 함.
# 결과: HOME pose 에서 카메라가 큐브를 위에서 아래로 내려다봄.
CAMERA_LOCAL_TRANSLATE = (0.0, 0.0, 0.05)   # 5cm gripper 쪽으로 (시야에 fingers 살짝)
CAMERA_LOCAL_RPY_DEG = (180.0, 0.0, 0.0)

DEFAULT_HOME_DEG = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)

# 스캔 패턴 (HOME pose 기준 j1, j2 진폭). Lissajous: 두 sin 의 주파수가 다르면
# 시간에 따라 (j1, j2) 가 평면 위 8자/타원 궤적 → tool0 가 X-Y 평면을 훑음.
SCAN_J1_AMP_DEG = 30.0     # base yaw 진폭 (deg). ±30° → 좌우 방향 탐색
SCAN_J2_AMP_DEG = 0.0      # 사용 안 함 (j1 만 sweep). 0 으로 두면 j2 고정.
SCAN_FREQ_J1_HZ = 0.12     # j1 진동 주파수
SCAN_FREQ_J2_HZ = 0.05     # (j2 안 쓰므로 unused)
DETECTION_MIN_AREA_PX2 = 200   # px² — 화성 lighting 에서 cyan 어두워질 수 있어 낮춤
FOUND_HOLD_SEC = 4.0           # FOUND 후 정지 유지 시간
DEBUG_DUMP_EVERY_SEC = 2.0     # 이 초마다 카메라 프레임 + HSV mask 를 /tmp 에 저장
DEBUG_DUMP_DIR = "/tmp/wrist_cam_debug"

# CLI 에서 덮어쓰는 카메라 mount params (build_scene 가 참조). _run() 가 채움.
_CAM_XYZ = CAMERA_LOCAL_TRANSLATE
_CAM_RPY = CAMERA_LOCAL_RPY_DEG


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


def _apply_terrain_collision(stage) -> int:
    """/World/Mars 하위 모든 Mesh 에 CollisionAPI + MeshCollisionAPI 'meshSimplification' 보강.
    mars_exploration_world.usd 의 baked collision 이 신뢰성 떨어져 (큐브가 통과)
    여기서 강제 재적용. 멱등.
    """
    n = 0
    mars = stage.GetPrimAtPath("/World/Mars")
    if not mars.IsValid():
        return 0
    for prim in Usd.PrimRange(mars):
        if prim.GetTypeName() != "Mesh":
            continue
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim)
        if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            UsdPhysics.MeshCollisionAPI.Apply(prim)
        mca = UsdPhysics.MeshCollisionAPI(prim)
        approx = mca.GetApproximationAttr() or mca.CreateApproximationAttr()
        approx.Set("meshSimplification")
        n += 1
    return n


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


def build_scene(spawn_cube: bool):
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
    # /World/Mars 하위 모든 Mesh 에 collision 강제 적용
    n_coll = _apply_terrain_collision(stage)
    print(f"  [PhysX] terrain mesh collision 강제 적용: {n_coll}개 mesh")

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
    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(SPAWN_X, SPAWN_Y, SPAWN_Z))
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
    xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1, 1, 1))
    for _ in range(8):
        simulation_app.update()
    _freeze_rover_drives(rover_prim)

    print("[3/5] M0609 URDF …")
    m0609_world = (SPAWN_X + M0609_MOUNT_OFFSET[0],
                   SPAWN_Y + M0609_MOUNT_OFFSET[1],
                   SPAWN_Z + M0609_MOUNT_OFFSET[2])
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

    print("[5/5] rover ↔ M0609 mount + anchor + camera …")
    m0609_base = _find_prim_path_by_name(robot_root, "base_link") or f"{robot_root}/base_link"
    rover_body = _find_prim_path_by_name(rover_path, "Body") or f"{rover_path}/Body"
    stage.SetEditTarget(Usd.EditTarget(stage.GetRootLayer()))
    mount_path = f"{rover_body}/M0609_Mount"
    mount_prim = stage.DefinePrim(mount_path, "Xform")
    UsdGeom.Xformable(mount_prim).AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*M0609_MOUNT_OFFSET)
    )
    for _ in range(5):
        simulation_app.update()
    _assemble(robot_root, m0609_base, rover_path, mount_path, "RoverMount", "m0609_on_rover")
    _freeze_rover_drives(rover_prim)

    # rover anchor (Z 추락 방지)
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

    cube = None
    if spawn_cube:
        cube = DynamicCuboid(
            prim_path="/World/cyan_cube",
            name="cyan_cube",
            position=np.array(CUBE_POS),
            scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]),
            color=np.array(CUBE_COLOR),
            mass=0.05,
        )
        cp = stage.GetPrimAtPath("/World/cyan_cube")
        has_rb = cp.HasAPI(UsdPhysics.RigidBodyAPI)
        has_co = cp.HasAPI(UsdPhysics.CollisionAPI)
        print(f"  [cube] spawn @ {CUBE_POS}  RigidBody={has_rb}  Collider={has_co}")
        # 즉시 FixedJoint anchor — 낙하/굴러감 방지. 큐브를 spawn 위치에 그대로 고정.
        stage.SetEditTarget(Usd.EditTarget(stage.GetRootLayer()))
        anchor = UsdPhysics.FixedJoint.Define(stage, "/World/Joints/CubeAnchor")
        anchor.CreateBody1Rel().SetTargets([Sdf.Path("/World/cyan_cube")])
        anchor.CreateLocalPos0Attr().Set(Gf.Vec3f(*CUBE_POS))
        anchor.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        anchor.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        anchor.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        anchor.CreateBreakForceAttr().Set(float('inf'))
        anchor.CreateBreakTorqueAttr().Set(float('inf'))
        print(f"  [cube] anchor 즉시 적용 @ {CUBE_POS}  (낙하/굴러감 차단)")

    # wrist camera (RG2 angle_bracket 자식)
    cam_parent = _find_prim_path_by_name(robot_root, "angle_bracket")
    cam_xform_path = f"{cam_parent}/wrist_cam"
    cam_xform = stage.DefinePrim(cam_xform_path, "Xform")
    cx = UsdGeom.Xformable(cam_xform)
    cx.ClearXformOpOrder()
    cx.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*_CAM_XYZ))
    cx.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*_CAM_RPY))
    cam_path = f"{cam_xform_path}/Camera"
    UsdGeom.Camera.Define(stage, cam_path)
    print(f"  [cam] mounted under {cam_parent}")
    print(f"  [cam] local xyz={_CAM_XYZ}, rpy={_CAM_RPY}")

    for _ in range(10):
        simulation_app.update()

    return rover_prim, robot_root, cube, cam_path


def _resolve_joint_indices(art: SingleArticulation):
    """joint_1~6 의 dof index 6개 반환 (못 찾으면 None)."""
    names = list(art.dof_names) if art.dof_names is not None else []
    idx = []
    for i in range(6):
        n = f"joint_{i+1}"
        if n not in names:
            return None
        idx.append(names.index(n))
    return idx


def _drive_joints(art: SingleArticulation, idx, joint_deg):
    """joint_1~6 drive target 만 설정 (텔레포트 없음, 연속 모션용)."""
    rad = np.array([np.deg2rad(d) for d in joint_deg], dtype=np.float32)
    art.get_articulation_controller().apply_action(ArticulationAction(
        joint_positions=rad, joint_indices=np.array(idx, dtype=np.int32),
    ))


def _apply_home_pose(art: SingleArticulation, joint_deg, world: World):
    """joint_1~6 home pose 즉시 텔레포트 + drive target 도 설정 (초기 1회용)."""
    idx = _resolve_joint_indices(art)
    if idx is None:
        print("  [warn] joint_1~6 not all present in dof_names")
        return
    _drive_joints(art, idx, joint_deg)
    try:
        cur = art.get_joint_positions()
        if cur is not None:
            cur = np.array(cur, dtype=np.float32).copy()
            rad = np.array([np.deg2rad(d) for d in joint_deg], dtype=np.float32)
            for k, j in enumerate(idx):
                cur[j] = rad[k]
            art.set_joint_positions(cur)
    except Exception as e:
        print(f"  [warn] set_joint_positions failed: {e}")


def main():
    try:
        _run()
    except Exception as e:
        import traceback
        print(f"\n[FATAL] {e.__class__.__name__}: {e}", flush=True)
        traceback.print_exc()
    finally:
        try:
            simulation_app.close()
        except Exception:
            pass
        os._exit(0)


def _parse_pose(s: str):
    parts = [float(x) for x in s.split(",")]
    if len(parts) != 6:
        raise argparse.ArgumentTypeError("pose 는 j1,j2,j3,j4,j5,j6 (deg) 6 개")
    return tuple(parts)


def _parse_xyz(s: str):
    parts = [float(x) for x in s.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("3개 (x,y,z) 필요")
    return tuple(parts)


def _run():
    global _CAM_XYZ, _CAM_RPY
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-cube", action="store_true", help="시안 큐브 생략")
    ap.add_argument("--no-play", action="store_true", help="auto-play OFF")
    ap.add_argument("--pose", type=_parse_pose, default=DEFAULT_HOME_DEG,
                    help="joint_1~6 home pose (deg), comma-separated")
    ap.add_argument("--cam-xyz", type=_parse_xyz, default=CAMERA_LOCAL_TRANSLATE,
                    help="angle_bracket 기준 카메라 local translate (m)")
    ap.add_argument("--cam-rpy", type=_parse_xyz, default=CAMERA_LOCAL_RPY_DEG,
                    help="angle_bracket 기준 카메라 local RotateXYZ (deg)")
    args = ap.parse_args()
    _CAM_XYZ = args.cam_xyz
    _CAM_RPY = args.cam_rpy

    world = World(stage_units_in_meters=1.0)
    rover_prim, robot_root, cube, cam_path = build_scene(spawn_cube=not args.no_cube)

    art = SingleArticulation(prim_path=robot_root, name="m0609_art")
    world.scene.add(art)

    print("\n[World] reset …")
    world.reset()
    _freeze_rover_drives(rover_prim)
    _apply_home_pose(art, args.pose, world)
    for _ in range(20):
        world.step(render=True)
    print(f"  [home pose] {args.pose} deg applied")

    # 큐브는 build_scene 에서 spawn 직후 즉시 anchor 됐음 → 추가 처리 불필요.
    if cube is not None:
        globals()["_CUBE_SETTLED_POS"] = CUBE_POS
        try:
            pos, _ = cube.get_world_pose()
            print(f"  [cube] world pos = ({float(pos[0]):+.3f}, "
                  f"{float(pos[1]):+.3f}, {float(pos[2]):+.3f})  (anchored)")
        except Exception:
            pass

    # 진단: 카메라의 실제 world 위치 + look 방향 (-Z) + cube 와의 관계
    stage = omni.usd.get_context().get_stage()
    cam_prim = stage.GetPrimAtPath(cam_path)
    if cam_prim.IsValid():
        m = UsdGeom.Xformable(cam_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        cam_pos = m.ExtractTranslation()
        # camera look dir = -Z 축의 world 방향
        look = m.TransformDir(Gf.Vec3d(0, 0, -1))
        print(f"  [cam-world] pos = ({cam_pos[0]:+.3f}, {cam_pos[1]:+.3f}, {cam_pos[2]:+.3f})")
        print(f"  [cam-world] look(-Z) dir = ({look[0]:+.3f}, {look[1]:+.3f}, {look[2]:+.3f})")
        if look[2] < -0.5:
            print("              → DOWNWARD ✓ (큐브를 위에서 봄)")
        elif look[2] > 0.5:
            print("              → UPWARD ✗ (하늘 봄, RPY 뒤집기 필요)")
        else:
            print("              → SIDE ✗ (옆을 봄, RPY 재조정 필요)")
        if not args.no_cube:
            ref = globals().get("_CUBE_SETTLED_POS", CUBE_POS)
            dx, dy, dz = (ref[0] - cam_pos[0],
                          ref[1] - cam_pos[1],
                          ref[2] - cam_pos[2])
            print(f"  [cam→cube] vec = ({dx:+.3f}, {dy:+.3f}, {dz:+.3f})  "
                  f"dist={(dx*dx+dy*dy+dz*dz)**0.5:.3f}m  (settled={ref})")

    # ── wrist_camera viewport 창 생성 (Isaac Sim 네이티브 윈도우) ──────
    try:
        from omni.kit.viewport.utility import create_viewport_window
        vp_win = create_viewport_window(
            name="wrist_camera",
            width=640,
            height=480,
            position_x=800,
            position_y=80,
            camera_path=cam_path,
        )
        if vp_win is None:
            print("  [viewport] create_viewport_window 가 None 반환 — "
                  "omni.kit.viewport.window 확장이 비활성일 수 있음")
        else:
            print(f"  [viewport] 'wrist_camera' 창 생성 (camera={cam_path})")
            print("             → Isaac Sim 내부에 패널로 도킹됨. "
                  "드래그하면 떨어내서 별도 OS 창으로 전환 가능.")
    except Exception as e:
        import traceback
        print(f"  [viewport] 생성 실패: {e.__class__.__name__}: {e}")
        traceback.print_exc()

    # ── 카메라 + HSV detector (탐색용) ──────────────────────────────
    from isaacsim.sensors.camera import Camera
    wrist_cam = Camera(prim_path=cam_path, resolution=CAMERA_RESOLUTION)
    wrist_cam.initialize()
    detector = CyanDetector()
    import cv2  # HSV 처리에만 사용 (imshow 안 함)

    joint_idx = _resolve_joint_indices(art)
    if joint_idx is None:
        print("[FATAL] joint_1~6 못 찾음 — 스캔 불가")
        return
    base_pose = list(args.pose)  # mutable copy
    print(f"\n[SCAN] start — j1 ±{SCAN_J1_AMP_DEG}° @ {SCAN_FREQ_J1_HZ}Hz, "
          f"j2 ±{SCAN_J2_AMP_DEG}° @ {SCAN_FREQ_J2_HZ}Hz  "
          f"(area threshold {DETECTION_MIN_AREA_PX2}px²)")

    if not args.no_play:
        time.sleep(0.5)
        world.play()
        print("  [play] auto-play ON")
    else:
        print("  [play] Spacebar 로 수동 Play")

    print("\n[VIEW] 종료: Isaac Sim 창 닫기 (Ctrl+C 또는 GUI X 버튼)\n")

    os.makedirs(DEBUG_DUMP_DIR, exist_ok=True)
    print(f"  [debug] 카메라 프레임 + mask 를 {DEBUG_DUMP_EVERY_SEC:.1f}초마다 "
          f"{DEBUG_DUMP_DIR}/ 에 저장")

    phase = "scan"
    t0_scan = None             # play 시작 시점 기준 (None → world 가 play 될 때 시작)
    t_found = None
    found_pose = None
    frame_n = 0
    last_log_t = time.time()
    last_dump_t = 0.0
    first_frame_logged = False

    while simulation_app.is_running():
        world.step(render=True)

        # Play 가 켜졌을 때만 스캔 진행. (world.is_playing → reset 직후 한 프레임은 False)
        playing = world.is_playing()
        if playing and t0_scan is None:
            t0_scan = time.time()
            print("  [scan] play detected → t=0")

        # 매 프레임 카메라 detect
        det = None
        bgr = None
        rgba = wrist_cam.get_rgba()
        if rgba is not None and rgba.size > 0:
            bgr = cv2.cvtColor(rgba[..., :3], cv2.COLOR_RGB2BGR)
            det = detector.detect(bgr)
            if not first_frame_logged:
                first_frame_logged = True
                h_img, w_img = bgr.shape[:2]
                center_bgr = bgr[h_img // 2, w_img // 2]
                mean_bgr = bgr.mean(axis=(0, 1))
                # HSV histogram 의 H 분포 — cyan (≈90) 픽셀이 실제로 있는지
                hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
                cyan_zone = ((hsv[..., 0] >= 70) & (hsv[..., 0] <= 110)
                             & (hsv[..., 1] >= 40) & (hsv[..., 2] >= 40))
                cyan_px = int(cyan_zone.sum())
                print(f"\n[FIRST FRAME] shape={bgr.shape} dtype={bgr.dtype}")
                print(f"              center BGR = {tuple(int(v) for v in center_bgr)}")
                print(f"              mean   BGR = ({mean_bgr[0]:.1f}, {mean_bgr[1]:.1f}, {mean_bgr[2]:.1f})")
                print(f"              loose cyan zone pixels = {cyan_px} (H∈70~110, S≥40, V≥40)")
                print(f"              (HSV detector 기본: H 80~100, S 100~255, V 80~255)")
                # 진단용 즉시 저장
                cv2.imwrite(f"{DEBUG_DUMP_DIR}/first_frame.png", bgr)
                cv2.imwrite(f"{DEBUG_DUMP_DIR}/first_mask.png",
                            det.mask if det is not None and det.mask is not None
                            else np.zeros_like(bgr[..., 0]))
                print(f"              저장: {DEBUG_DUMP_DIR}/first_frame.png, first_mask.png\n")

        # state machine
        if phase == "scan" and playing and t0_scan is not None:
            t = time.time() - t0_scan
            j1 = SCAN_J1_AMP_DEG * math.sin(2.0 * math.pi * SCAN_FREQ_J1_HZ * t)
            j2 = base_pose[1] + SCAN_J2_AMP_DEG * math.sin(2.0 * math.pi * SCAN_FREQ_J2_HZ * t)
            scan_pose = (j1, j2, base_pose[2], base_pose[3], base_pose[4], base_pose[5])
            _drive_joints(art, joint_idx, scan_pose)

            if det is not None and det.found and det.area >= DETECTION_MIN_AREA_PX2:
                t_found = time.time()
                found_pose = scan_pose
                phase = "found"
                print(f"\n[FOUND] cyan @ cx={det.cx:.1f} cy={det.cy:.1f} "
                      f"area={det.area:.0f}px²  "
                      f"pose=(j1={j1:+.1f}, j2={j2:+.1f}, …)  t={t:.1f}s")

        elif phase == "found":
            # 발견 위치 유지 (drive target 잠금)
            if found_pose is not None:
                _drive_joints(art, joint_idx, found_pose)
            if time.time() - t_found > FOUND_HOLD_SEC:
                print(f"  [found] hold {FOUND_HOLD_SEC:.1f}s done → resume scan")
                phase = "scan"
                t0_scan = time.time()  # phase counter reset → 새 Lissajous 사이클
                t_found = None

        # 주기적으로 카메라 프레임 + mask 디스크 저장 (진단용)
        frame_n += 1
        now = time.time()
        if bgr is not None and (now - last_dump_t) > DEBUG_DUMP_EVERY_SEC:
            last_dump_t = now
            ts = f"{int((now - (t0_scan or now)) * 10):05d}"
            cv2.imwrite(f"{DEBUG_DUMP_DIR}/frame_{ts}.png", bgr)
            if det is not None and det.mask is not None:
                cv2.imwrite(f"{DEBUG_DUMP_DIR}/mask_{ts}.png", det.mask)

        if now - last_log_t > 1.0:
            last_log_t = now
            det_txt = (f"cyan cx={det.cx:.0f} cy={det.cy:.0f} area={det.area:.0f}"
                       if (det and det.found) else "no cyan")
            # max area 진단 — area < threshold 라 가까스로 놓치는 케이스 보이게
            max_in_mask = 0
            if det is not None and det.mask is not None:
                nz = int((det.mask > 0).sum())
                max_in_mask = nz
            j1_now = j2_now = None
            try:
                jp = art.get_joint_positions()
                if jp is not None:
                    j1_now = float(np.rad2deg(jp[joint_idx[0]]))
                    j2_now = float(np.rad2deg(jp[joint_idx[1]]))
            except Exception:
                pass
            print(f"  [{phase}] j1={j1_now:+.1f}° j2={j2_now:+.1f}°  "
                  f"{det_txt}  mask_nonzero={max_in_mask}px")


if __name__ == "__main__":
    main()
