"""Wrist-camera viewer + Cartesian (posx-style) X-Y planar Lissajous scan.

view_wrist_cam.py 의 posj-style joint Lissajous 와 다르게, **link_6 (EE) 의
Cartesian target XYZ** 를 평면에서 sweep. 매 프레임 Damped Least-Squares
Jacobian IK 로 joint 명령을 풀어 EE 가 Z 와 자세 (카메라 ↓) 를 고정한 채
X-Y 평면을 훑음.

Doosan posx(x,y,z,…) 명령과 사실상 동일한 결과 — controller 대신 우리가 IK 를
직접 풀어 articulation drive target 으로 보냄.

State machine:
    scan  → cyan 검출 (area ≥ DETECTION_MIN_AREA_PX2) → found
    found → FOUND_HOLD_SEC 정지 후 scan 재개

실행:
    isaac-python ~/dev_ws/rover_ws/src/a2_isaac/isaac_manipulation/scripts/view_wrist_cam_posx.py
옵션:
    --no-cube                  시안 큐브 spawn 생략
    --no-play                  Play 자동 시작 안 함
    --pose 0,0,90,0,90,0       초기 seed pose (IK 시작점 + 자세 lock 기준)
    --xy-center 5.55,0.0       스캔 평면 중심 (world XY, m)
    --z 0.95                   카메라 target Z (world, m)
    --xy-amp 0.20,0.15         X, Y Lissajous 진폭 (m)
    --xy-freq 0.10,0.07        X, Y Lissajous 주파수 (Hz)
    --cam-xyz / --cam-rpy      카메라 mount 변경 (angle_bracket 기준)
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
_ext.set_extension_enabled_immediate("omni.kit.viewport.window", True)
_ext.set_extension_enabled_immediate("omni.kit.viewport.utility", True)

from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_setup.assembler import RobotAssembler

_PKG_PARENT = os.environ.get("A2_ISAAC_ROOT") or str(
    Path(__file__).resolve().parents[2]
)
sys.path.insert(0, f"{_PKG_PARENT}/isaac_perception")
from isaac_perception.cyan_detector import CyanDetector


A2_ROOT = Path(_PKG_PARENT)
MARS_WORLD_USD = A2_ROOT / "isaac_sim/worlds/mars_exploration_world.usd"
# rover + M0609 + RG2 가 이미 assembled 된 단일 USD. 내부 구조:
#   /Vehicle/Vehicle/m0609/base_link  ← articulation root
#   /Vehicle/Vehicle/rover/Body       ← anchor 대상
#   /Vehicle/Vehicle/onrobot_rg2ft/angle_bracket  ← 카메라 마운트
VEHICLE_USD = A2_ROOT.parent / "Vehicle.usd"

# rover spawn XY — 사용자 지정 평탄 영역. (5,0) 언덕, (0,0) 돔, (-5,0) 구멍.
SPAWN_X, SPAWN_Y = 4.5, -1.0
# 초기 drop Z — terrain 위로 살짝만 띄워 짧게 떨어트림.
SPAWN_Z_DROP = 0.2
M0609_MOUNT_OFFSET = (0.15274, 0.0, 0.21232)

# 안착 단계 — rover/cube 가 terrain 에 닿아 멈출 때까지 physics step 횟수.
# 60Hz 기준 120 frame ≈ 2 초. 짧은 drop 이라 더 짧아도 OK.
SETTLE_FRAMES = 120
    
CUBE_SIZE = 0.10
CUBE_DROP_XY = (SPAWN_X + 0.7, SPAWN_Y)
CUBE_DROP_Z = 0.3           # cube 도 짧게 떨궈서 terrain 위에 안착
CUBE_COLOR = (0.0, 1.0, 1.0)

CAMERA_RESOLUTION = (640, 480)
CAMERA_LOCAL_TRANSLATE = (0.0, 0.0, 0.05)
CAMERA_LOCAL_RPY_DEG = (180.0, 0.0, 0.0)

DEFAULT_HOME_DEG = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)

# ── posx Cartesian scan params ──────────────────────────────────────────
# 카메라 target world Z. settle 후 rover Body ≈ z=-0.15, M0609 base ≈ z=0.06,
# HOME link_6 ≈ z=0.52. scan z=0.80 이면 arm 을 위로 펴서 camera 가 gripper
# body 위로 올라옴 → cube occlusion 해소. M0609 max reach (~0.92) 안쪽.
DEFAULT_SCAN_Z = 0.80
# 스캔 평면 중심 (world XY). cube drop XY = (SPAWN_X+0.7, 0) 기준 약간 안쪽으로.
DEFAULT_SCAN_XY_CENTER = (SPAWN_X + 0.5, -0.05)
# X, Y Lissajous 진폭 (±m). settle 후 cube 가 약간 굴러갈 수 있어 넉넉히.
DEFAULT_SCAN_XY_AMP = (0.25, 0.20)
# X, Y 주파수 (Hz). 두 주파수가 다르면 Lissajous 궤적.
DEFAULT_SCAN_XY_FREQ = (0.10, 0.07)

# IK gain — 한 step 에 error 의 alpha 만큼 좁힘. 0.3~0.6 권장.
IK_ALPHA = 0.5
# DLS damping — singular 근처에서 step 폭주 방지. 0.05~0.15.
IK_DAMPING = 0.10
# Nullspace 끌어당김 강도 — EE target 을 만족하는 여러 joint 조합 중 HOME pose
# 에 가까운 쪽으로 IK 가 골라쓰게 함. 크면 arm 이 HOME 자세 유지, 0 이면 IK 가
# 매 frame 임의 조합 (wrist twist, base 회전 등) 골라 "구부러지는" 모션 발생.
IK_NULLSPACE_GAIN = 0.6

DETECTION_MIN_AREA_PX2 = 200
FOUND_HOLD_SEC = 4.0
DEBUG_DUMP_EVERY_SEC = 2.0
DEBUG_DUMP_DIR = "/tmp/wrist_cam_posx_debug"

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


def _raycast_terrain_z(x, y, search_from=5.0, max_dist=15.0):
    """Mars terrain top Z at (x, y) via PhysX downward raycast. None if no hit."""
    try:
        from omni.physx import get_physx_scene_query_interface
        sqi = get_physx_scene_query_interface()
        origin = carb.Float3(float(x), float(y), float(search_from))
        direction = carb.Float3(0.0, 0.0, -1.0)
        hit = sqi.raycast_closest(origin, direction, float(max_dist))
        if isinstance(hit, dict) and hit.get("hit", False):
            pos = hit.get("position", None)
            if pos is None:
                return None
            try:
                return float(pos[2])
            except (TypeError, IndexError):
                try:
                    return float(pos.z)
                except AttributeError:
                    return None
    except Exception as e:
        print(f"  [raycast] exception: {e.__class__.__name__}: {e}")
    return None


def _freeze_rover_drives(rover_prim) -> int:
    """Rover 휠/조향 드라이브의 속도를 0 으로 강하게 댐핑.

    중요: stiffness=0 — target_position 으로 끌어당기지 않음. 그렇지 않으면
    Continuous wheel joint 가 target=0 으로 강제 회전돼 rover 가 굴러감.
    velocity=0 + 큰 damping 만으로 회전 저항 (현재 각도 유지).
    """
    n = 0
    for prim in Usd.PrimRange(rover_prim):
        for drv_type in ("angular", "linear"):
            drv = UsdPhysics.DriveAPI.Get(prim, drv_type)
            if drv:
                drv.GetTargetVelocityAttr().Set(0.0)
                drv.GetStiffnessAttr().Set(0.0)        # position-based force 없음
                drv.GetDampingAttr().Set(1e6)          # velocity-resistant
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
    n_coll = _apply_terrain_collision(stage)
    print(f"  [PhysX] terrain mesh collision 강제 적용: {n_coll}개 mesh")

    print("[2/3] Vehicle.usd (rover + M0609 + RG2 통합) …")
    # Vehicle.usd 내부에 큰 offset 이 baked-in. 먼저 outer=(0,0,0) 으로 load 해서
    # rover Body 의 internal offset 읽고, 그것으로 outer translate 를 보정 →
    # rover Body 가 정확히 (SPAWN_X, SPAWN_Y, SPAWN_Z_DROP) 위치에서 출발.
    outer_path = "/World/VehicleHolder"
    veh_path = f"{outer_path}/Vehicle"
    outer_prim = stage.DefinePrim(outer_path, "Xform")
    xform = UsdGeom.Xformable(outer_prim)
    xform.ClearXformOpOrder()
    translate_op = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.0))  # 일단 원점, 측정 후 보정
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
    xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1, 1, 1))
    veh_inner = stage.DefinePrim(veh_path, "Xform")
    veh_inner.GetReferences().AddReference(str(VEHICLE_USD))
    for _ in range(10):
        simulation_app.update()

    # Vehicle.usd 내부 paths (assembly 이미 완료 — RobotAssembler 호출 불필요)
    robot_root = _find_prim_path_by_name(veh_path, "m0609") or f"{veh_path}/m0609"
    rover_subtree = _find_prim_path_by_name(veh_path, "rover") or f"{veh_path}/rover"
    rover_body = _find_prim_path_by_name(veh_path, "Body") or f"{rover_subtree}/Body"
    cam_parent = _find_prim_path_by_name(veh_path, "angle_bracket") \
                 or f"{veh_path}/onrobot_rg2ft/angle_bracket"

    # rover Body 의 internal world pos 측정 → outer translate 보정
    internal_xyz = _read_world_xyz(rover_body)
    if internal_xyz is not None:
        dx = SPAWN_X - internal_xyz[0]
        dy = SPAWN_Y - internal_xyz[1]
        dz = SPAWN_Z_DROP - internal_xyz[2]
        translate_op.Set(Gf.Vec3d(dx, dy, dz))
        print(f"  [vehicle] rover Body internal pos = ({internal_xyz[0]:+.3f},"
              f"{internal_xyz[1]:+.3f},{internal_xyz[2]:+.3f})")
        print(f"  [vehicle] outer translate set to ({dx:+.3f},{dy:+.3f},{dz:+.3f}) → "
              f"rover Body @ ({SPAWN_X:+.2f},{SPAWN_Y:+.2f},{SPAWN_Z_DROP:+.2f})")
        for _ in range(5):
            simulation_app.update()
    print(f"  [vehicle] robot_root   = {robot_root}")
    print(f"  [vehicle] rover Body   = {rover_body}")
    print(f"  [vehicle] rover subtree= {rover_subtree}")
    print(f"  [vehicle] cam parent   = {cam_parent}")

    # rover 휠/스티어 드라이브만 freeze (M0609 joint 들은 건드리지 않음)
    rover_subtree_prim = stage.GetPrimAtPath(rover_subtree)
    if rover_subtree_prim.IsValid():
        n_frozen = _freeze_rover_drives(rover_subtree_prim)
        print(f"  [vehicle] frozen {n_frozen} rover drives")

    stage.SetEditTarget(Usd.EditTarget(stage.GetRootLayer()))
    stage.DefinePrim("/World/Joints", "Scope")
    # rover anchor 는 settle 후 _run() 에서 부착.

    print("[3/3] cube + camera …")

    cube = None
    if spawn_cube:
        cube_drop_pos = (CUBE_DROP_XY[0], CUBE_DROP_XY[1], CUBE_DROP_Z)
        cube = DynamicCuboid(
            prim_path="/World/cyan_cube",
            name="cyan_cube",
            position=np.array(cube_drop_pos),
            scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]),
            color=np.array(CUBE_COLOR),
            mass=0.05,
        )
        cp = stage.GetPrimAtPath("/World/cyan_cube")
        print(f"  [cube] drop @ {cube_drop_pos}  "
              f"RigidBody={cp.HasAPI(UsdPhysics.RigidBodyAPI)}  "
              f"Collider={cp.HasAPI(UsdPhysics.CollisionAPI)}  "
              f"(anchor: settle 후 부착)")

    cam_xform_path = f"{cam_parent}/wrist_cam"
    cam_xform = stage.DefinePrim(cam_xform_path, "Xform")
    cx = UsdGeom.Xformable(cam_xform)
    cx.ClearXformOpOrder()
    cx.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*_CAM_XYZ))
    cx.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*_CAM_RPY))
    cam_path = f"{cam_xform_path}/Camera"
    UsdGeom.Camera.Define(stage, cam_path)
    print(f"  [cam] mounted under {cam_parent}  xyz={_CAM_XYZ} rpy={_CAM_RPY}")

    for _ in range(10):
        simulation_app.update()

    return outer_prim, robot_root, cube, cam_path, rover_body


def _resolve_joint_indices(art: SingleArticulation):
    names = list(art.dof_names) if art.dof_names is not None else []
    idx = []
    for i in range(6):
        n = f"joint_{i+1}"
        if n not in names:
            return None
        idx.append(names.index(n))
    return idx


def _drive_joints_deg(art, idx, joint_deg):
    rad = np.array([np.deg2rad(d) for d in joint_deg], dtype=np.float32)
    art.get_articulation_controller().apply_action(ArticulationAction(
        joint_positions=rad, joint_indices=np.array(idx, dtype=np.int32),
    ))


def _drive_joints_rad(art, idx, joint_rad):
    rad = np.array(joint_rad, dtype=np.float32)
    art.get_articulation_controller().apply_action(ArticulationAction(
        joint_positions=rad, joint_indices=np.array(idx, dtype=np.int32),
    ))


def _apply_home_pose(art, joint_deg, world):
    idx = _resolve_joint_indices(art)
    if idx is None:
        print("  [warn] joint_1~6 not all present in dof_names")
        return
    _drive_joints_deg(art, idx, joint_deg)
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


# ── IK helpers (Damped Least Squares Jacobian) ─────────────────────────

def _get_link_world_pose(link_path):
    """Return (pos[3], quat_wxyz[4]) of a prim in world frame."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(link_path)
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat()
    qxyz = q.GetImaginary()
    return (
        np.array([t[0], t[1], t[2]], dtype=np.float64),
        np.array([q.GetReal(), qxyz[0], qxyz[1], qxyz[2]], dtype=np.float64),
    )


def _read_world_xyz(prim_path):
    """Return (x, y, z) world position of a prim via USD."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = m.ExtractTranslation()
    return (float(t[0]), float(t[1]), float(t[2]))


def _quat_orientation_error(q_target, q_current):
    """3D rotation vector ω such that exp(ω) ≈ q_target * q_current⁻¹ (wxyz).

    Small-angle approx: ω ≈ 2 * (xyz part of q_err). Shortest-path resolved.
    """
    qw_t, qx_t, qy_t, qz_t = q_target
    qw_c, qx_c, qy_c, qz_c = q_current
    qw_e =  qw_t * qw_c + qx_t * qx_c + qy_t * qy_c + qz_t * qz_c
    qx_e = -qw_t * qx_c + qx_t * qw_c - qy_t * qz_c + qz_t * qy_c
    qy_e = -qw_t * qy_c + qx_t * qz_c + qy_t * qw_c - qz_t * qx_c
    qz_e = -qw_t * qz_c - qx_t * qy_c + qy_t * qx_c + qz_t * qw_c
    if qw_e < 0.0:
        qx_e, qy_e, qz_e = -qx_e, -qy_e, -qz_e
    return 2.0 * np.array([qx_e, qy_e, qz_e], dtype=np.float64)


def _get_body_names(art):
    """SingleArticulation API 가 버전마다 달라 — body_names 를 안전하게 추출.

    Isaac Sim 5.1 의 SingleArticulation 은 body_names 를 직접 노출하지 않고
    articulation_view (또는 _articulation_view) 를 통해 접근해야 함.
    """
    for attr in ("body_names",):
        if hasattr(art, attr):
            try:
                v = getattr(art, attr)
                if v is not None:
                    return list(v)
            except Exception:
                pass
    av = None
    for view_attr in ("articulation_view", "_articulation_view"):
        if hasattr(art, view_attr):
            av = getattr(art, view_attr)
            if av is not None:
                break
    if av is not None:
        for attr in ("body_names", "link_names", "_body_names"):
            if hasattr(av, attr):
                try:
                    v = getattr(av, attr)
                    if v is not None:
                        return list(v)
                except Exception:
                    pass
    return None


def _find_ee_body_index(body_names, ee_name="link_6"):
    """body_names 내에서 link_6 의 index. 정확 일치 → '/link_6' suffix → 'link_6' 부분일치 순."""
    if not body_names:
        return None
    if ee_name in body_names:
        return body_names.index(ee_name)
    for i, n in enumerate(body_names):
        if n.endswith(f"/{ee_name}") or n.endswith(ee_name):
            return i
    return None


def _get_jacobians(art):
    """SingleArticulation 은 get_jacobians 를 직접 노출하지 않음.
    articulation_view 우회.
    """
    if hasattr(art, "get_jacobians"):
        try:
            v = art.get_jacobians()
            if v is not None:
                return v
        except Exception:
            pass
    av = None
    for view_attr in ("articulation_view", "_articulation_view"):
        if hasattr(art, view_attr):
            av = getattr(art, view_attr)
            if av is not None:
                break
    if av is not None and hasattr(av, "get_jacobians"):
        try:
            return av.get_jacobians()
        except Exception:
            pass
    return None


def _resolve_jacobian(art, ee_body_index):
    """Return (J_body[6, num_cols], col_offset). col_offset = 6 for floating-base."""
    J = _get_jacobians(art)
    if J is None:
        return None, None
    arr = np.asarray(J)
    if arr.ndim == 4:
        arr = arr[0]
    n_cols = arr.shape[2]
    n_dof = art.num_dof
    if n_cols == n_dof:
        col_offset = 0
    elif n_cols == n_dof + 6:
        col_offset = 6
    else:
        col_offset = max(0, n_cols - n_dof)
    return arr, col_offset


def _ik_dls_step(art, link_path, joint_indices, target_pos, target_quat,
                 ee_body_index, q_rest=None, k_null=IK_NULLSPACE_GAIN,
                 alpha=IK_ALPHA, damping=IK_DAMPING):
    """One DLS Jacobian IK update step with nullspace bias toward q_rest.

    dq = J⁺ · err + (I - J⁺J) · k_null · (q_rest - q_cur)
         ───┬─────   ─────────┬──────────────────────────
            │                 │
       primary task     nullspace bias (EE 안 영향)
    """
    cur_pos, cur_quat = _get_link_world_pose(link_path)
    pos_err = target_pos - cur_pos
    rot_err = _quat_orientation_error(target_quat, cur_quat)
    err = np.concatenate([pos_err, rot_err])

    J_all, col_offset = _resolve_jacobian(art, ee_body_index)
    if J_all is None:
        return None, err
    J_body = J_all[ee_body_index]
    arm_cols = [col_offset + i for i in joint_indices]
    J_arm = J_body[:, arm_cols]                       # (6, n)
    n = len(joint_indices)

    lam2 = damping * damping
    JJT = J_arm @ J_arm.T
    try:
        J_pinv = J_arm.T @ np.linalg.inv(JJT + lam2 * np.eye(6))  # (n, 6)
    except np.linalg.LinAlgError:
        return None, err
    dq_primary = J_pinv @ err                         # (n,)

    cur_q_full = np.array(art.get_joint_positions(), dtype=np.float64)

    if q_rest is not None and k_null > 0:
        q_cur_arm = cur_q_full[joint_indices]
        q_rest_arr = np.asarray(q_rest, dtype=np.float64)
        if q_rest_arr.shape[0] == cur_q_full.shape[0]:
            q_rest_arm = q_rest_arr[joint_indices]
        else:
            q_rest_arm = q_rest_arr[:n]
        bias = k_null * (q_rest_arm - q_cur_arm)
        N = np.eye(n) - J_pinv @ J_arm               # nullspace projector
        dq = dq_primary + N @ bias
    else:
        dq = dq_primary

    new_q_full = cur_q_full.copy()
    for k, j in enumerate(joint_indices):
        new_q_full[j] += alpha * float(dq[k])
    return new_q_full, err


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


def _parse_pose(s):
    parts = [float(x) for x in s.split(",")]
    if len(parts) != 6:
        raise argparse.ArgumentTypeError("pose 는 j1,j2,j3,j4,j5,j6 (deg) 6 개")
    return tuple(parts)


def _parse_n(s, n):
    parts = [float(x) for x in s.split(",")]
    if len(parts) != n:
        raise argparse.ArgumentTypeError(f"{n} 개 값 필요")
    return tuple(parts)


def _run():
    global _CAM_XYZ, _CAM_RPY
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-cube", action="store_true")
    ap.add_argument("--no-play", action="store_true")
    ap.add_argument("--pose", type=_parse_pose, default=DEFAULT_HOME_DEG)
    ap.add_argument("--xy-center", type=lambda s: _parse_n(s, 2),
                    default=DEFAULT_SCAN_XY_CENTER,
                    help="스캔 평면 중심 (world XY, m)")
    ap.add_argument("--z", type=float, default=DEFAULT_SCAN_Z,
                    help="카메라 target world Z (m)")
    ap.add_argument("--xy-amp", type=lambda s: _parse_n(s, 2),
                    default=DEFAULT_SCAN_XY_AMP,
                    help="X, Y Lissajous 진폭 (m)")
    ap.add_argument("--xy-freq", type=lambda s: _parse_n(s, 2),
                    default=DEFAULT_SCAN_XY_FREQ,
                    help="X, Y Lissajous 주파수 (Hz)")
    ap.add_argument("--cam-xyz", type=lambda s: _parse_n(s, 3),
                    default=CAMERA_LOCAL_TRANSLATE)
    ap.add_argument("--cam-rpy", type=lambda s: _parse_n(s, 3),
                    default=CAMERA_LOCAL_RPY_DEG)
    args = ap.parse_args()
    _CAM_XYZ = args.cam_xyz
    _CAM_RPY = args.cam_rpy

    world = World(stage_units_in_meters=1.0)
    rover_prim, robot_root, cube, cam_path, rover_body_path = build_scene(spawn_cube=not args.no_cube)

    art = SingleArticulation(prim_path=robot_root, name="m0609_art")
    world.scene.add(art)

    print("\n[World] reset …")
    world.reset()
    # rover 의 휠/조향 드라이브만 다시 freeze (M0609 joints 는 그대로 둠)
    stage_now = omni.usd.get_context().get_stage()
    rover_subtree_path = rover_body_path.rsplit("/", 1)[0]   # ".../rover"
    rover_subtree_prim_now = stage_now.GetPrimAtPath(rover_subtree_path)
    if rover_subtree_prim_now.IsValid():
        _freeze_rover_drives(rover_subtree_prim_now)
    # HOME pose 는 settle 후에 적용 — 떨어지는 동안 arm 은 URDF 기본 (j=0, 위쪽) 으로 유지

    # ── Drop and settle ──────────────────────────────────────────────
    # rover/cube anchor 없이 떨궈서 Mars terrain 위에 자연스럽게 안착.
    # 안착 후 rover 위치를 읽어 FixedJoint 로 고정.
    print(f"\n[Settle] dropping {SETTLE_FRAMES} frames "
          f"(rover from z={SPAWN_Z_DROP:.2f}, cube from z={CUBE_DROP_Z:.2f}) …")
    if not world.is_playing():
        world.play()
    for k in range(SETTLE_FRAMES):
        world.step(render=True)
        if k % 30 == 0:
            # 실제 physics body 의 위치 = rover_body_path (Body rigid). rover_prim
            # (parent Xform) 은 spawn 좌표 그대로 유지되므로 anchor 기준이 되면 안 됨.
            rxyz = _read_world_xyz(rover_body_path)
            cxyz = _read_world_xyz("/World/cyan_cube") if cube is not None else None
            print(f"  [settle {k:3d}] rover Body z={rxyz[2]:+.3f}"
                  + (f"  cube z={cxyz[2]:+.3f}" if cxyz else ""))

    # 안착 후 rover Body 의 실제 world 위치 캡쳐 + FixedJoint anchor
    stage = omni.usd.get_context().get_stage()
    rover_settled = _read_world_xyz(rover_body_path)
    print(f"  [settle] rover Body settled @ ({rover_settled[0]:+.3f}, "
          f"{rover_settled[1]:+.3f}, {rover_settled[2]:+.3f})")
    if cube is not None:
        cube_settled = _read_world_xyz("/World/cyan_cube")
        print(f"  [settle] cube settled @ ({cube_settled[0]:+.3f}, "
              f"{cube_settled[1]:+.3f}, {cube_settled[2]:+.3f})")

    stage.SetEditTarget(Usd.EditTarget(stage.GetRootLayer()))
    anchor = UsdPhysics.FixedJoint.Define(stage, "/World/Joints/RoverAnchor")
    anchor.CreateBody1Rel().SetTargets([Sdf.Path(rover_body_path)])
    anchor.CreateLocalPos0Attr().Set(Gf.Vec3f(*rover_settled))
    anchor.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    anchor.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    anchor.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    anchor.CreateBreakForceAttr().Set(float("inf"))
    anchor.CreateBreakTorqueAttr().Set(float("inf"))
    print(f"  [anchor] rover FixedJoint @ settled pos")

    # 이제 HOME pose 적용 — rover 가 anchor 됐으므로 arm 움직임에 rover 안 밀림
    _apply_home_pose(art, args.pose, world)
    for _ in range(30):
        world.step(render=True)
    print(f"  [home pose] {args.pose} deg applied after settle")

    # HOME pose 의 joint 값 캡쳐 → IK nullspace bias 의 rest target 으로 사용
    q_home_full = np.array(art.get_joint_positions(), dtype=np.float64)
    print(f"  [home] arm joint values captured for IK nullspace bias "
          f"(k_null={IK_NULLSPACE_GAIN})")

    print(f"  [art] num_dof = {art.num_dof}")
    print(f"  [art] dof_names = {list(art.dof_names) if art.dof_names else None}")
    body_names = _get_body_names(art)
    print(f"  [art] body_names = {body_names}")

    joint_idx = _resolve_joint_indices(art)
    if joint_idx is None:
        print("[FATAL] joint_1~6 dof_names 에 없음")
        return
    print(f"  [art] arm joint dof indices = {joint_idx}")

    if not body_names:
        print("[FATAL] articulation body_names 를 어떤 API 로도 얻지 못함. "
              "art.articulation_view 의 attribute 를 확인 필요.")
        return
    ee_body_index = _find_ee_body_index(body_names, "link_6")
    if ee_body_index is None:
        print(f"[FATAL] body_names 에 'link_6' 매치 없음. 후보: {body_names}")
        return
    print(f"  [art] EE link_6 body index = {ee_body_index}  "
          f"(name='{body_names[ee_body_index]}')")

    link6_path = _find_prim_path_by_name(robot_root, "link_6") or f"{robot_root}/link_6"

    J_all, col_off = _resolve_jacobian(art, ee_body_index)
    if J_all is None:
        print("[FATAL] articulation Jacobian 가져오기 실패")
        return
    print(f"  [art] Jacobian shape = {J_all.shape}  col_offset = {col_off}")

    # ── HOME 자세에서 link_6 / 카메라 world pose 캡쳐 ─────────────────
    l6_home_pos, l6_home_quat = _get_link_world_pose(link6_path)
    cam_home_pos, cam_home_quat = _get_link_world_pose(cam_path)
    cam_z_below_link6 = float(l6_home_pos[2] - cam_home_pos[2])
    print(f"  [home] link_6 world = ({l6_home_pos[0]:+.3f}, "
          f"{l6_home_pos[1]:+.3f}, {l6_home_pos[2]:+.3f})")
    print(f"  [home] camera world = ({cam_home_pos[0]:+.3f}, "
          f"{cam_home_pos[1]:+.3f}, {cam_home_pos[2]:+.3f})")
    print(f"  [home] cam_z_below_link6 = {cam_z_below_link6:+.3f} m")

    # 자세는 HOME 의 link_6 orientation 으로 lock (카메라 ↓)
    target_orient_lock = l6_home_quat.copy()
    print(f"  [home] link_6 quat (lock, wxyz) = "
          f"({target_orient_lock[0]:+.3f}, {target_orient_lock[1]:+.3f}, "
          f"{target_orient_lock[2]:+.3f}, {target_orient_lock[3]:+.3f})")

    if cube is not None:
        try:
            pos, _ = cube.get_world_pose()
            print(f"  [cube] world pos = ({float(pos[0]):+.3f}, "
                  f"{float(pos[1]):+.3f}, {float(pos[2]):+.3f})  (anchored)")
        except Exception:
            pass

    # ── wrist_camera viewport 패널 ────────────────────────────────────
    try:
        from omni.kit.viewport.utility import create_viewport_window
        vp_win = create_viewport_window(
            name="wrist_camera",
            width=640, height=480, position_x=800, position_y=80,
            camera_path=cam_path,
        )
        if vp_win is not None:
            print(f"  [viewport] 'wrist_camera' 창 생성 (camera={cam_path})")
    except Exception as e:
        print(f"  [viewport] 생성 실패: {e.__class__.__name__}: {e}")

    # ── 카메라 + HSV detector ─────────────────────────────────────────
    from isaacsim.sensors.camera import Camera
    wrist_cam = Camera(prim_path=cam_path, resolution=CAMERA_RESOLUTION)
    wrist_cam.initialize()
    # Mars lighting (cream/red 톤) 아래서 cyan 큐브가 desaturate 됨 →
    # 기본 (S≥100) 으로는 0px. S/V 임계 완화 + H 범위 약간 넓힘.
    detector = CyanDetector(
        hsv_lower=(70, 40, 40),
        hsv_upper=(110, 255, 255),
        min_area=80,
    )
    import cv2

    # ── 스캔 파라미터 ─────────────────────────────────────────────────
    Xc, Yc = args.xy_center
    Z_cam = args.z
    Z_link6 = Z_cam + cam_z_below_link6
    AMP_X, AMP_Y = args.xy_amp
    F_X, F_Y = args.xy_freq

    print(f"\n[SCAN posx] center=({Xc:.2f}, {Yc:.2f})  cam Z={Z_cam:.2f}m  "
          f"link_6 Z={Z_link6:.2f}m")
    print(f"            amp=(±{AMP_X:.2f}, ±{AMP_Y:.2f})m  "
          f"freq=({F_X:.2f}, {F_Y:.2f})Hz")
    print(f"            IK alpha={IK_ALPHA}  damping={IK_DAMPING}  "
          f"area threshold={DETECTION_MIN_AREA_PX2}px²")

    if not args.no_play:
        time.sleep(0.5)
        world.play()
        print("  [play] auto-play ON")
    else:
        print("  [play] Spacebar 로 수동 Play")

    print("\n[VIEW] 종료: Isaac Sim 창 닫기 (Ctrl+C 또는 GUI X 버튼)\n")

    os.makedirs(DEBUG_DUMP_DIR, exist_ok=True)
    print(f"  [debug] 카메라 프레임 + mask 를 {DEBUG_DUMP_EVERY_SEC:.1f}초마다 "
          f"{DEBUG_DUMP_DIR}/ 에 저장\n")

    phase = "scan"
    t0_scan = None
    t_found = None
    found_q = None
    last_log_t = time.time()
    last_dump_t = 0.0
    first_frame_logged = False

    while simulation_app.is_running():
        world.step(render=True)

        playing = world.is_playing()
        if playing and t0_scan is None:
            t0_scan = time.time()
            print("  [scan] play detected → t=0")

        det = None
        bgr = None
        rgba = wrist_cam.get_rgba()
        if rgba is not None and rgba.size > 0:
            bgr = cv2.cvtColor(rgba[..., :3], cv2.COLOR_RGB2BGR)
            det = detector.detect(bgr)
            if not first_frame_logged:
                first_frame_logged = True
                hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
                cyan_loose = ((hsv[..., 0] >= 70) & (hsv[..., 0] <= 110)
                              & (hsv[..., 1] >= 40) & (hsv[..., 2] >= 40))
                print(f"[FIRST FRAME] shape={bgr.shape}  "
                      f"loose cyan zone = {int(cyan_loose.sum())}px")
                cv2.imwrite(f"{DEBUG_DUMP_DIR}/first_frame.png", bgr)
                cv2.imwrite(f"{DEBUG_DUMP_DIR}/first_mask.png",
                            det.mask if det is not None and det.mask is not None
                            else np.zeros_like(bgr[..., 0]))

        if phase == "scan" and playing and t0_scan is not None:
            t = time.time() - t0_scan
            tx = Xc + AMP_X * math.sin(2.0 * math.pi * F_X * t)
            ty = Yc + AMP_Y * math.sin(2.0 * math.pi * F_Y * t)
            tz = Z_link6
            target_pos = np.array([tx, ty, tz], dtype=np.float64)

            new_q, err6 = _ik_dls_step(
                art, link6_path, joint_idx,
                target_pos, target_orient_lock, ee_body_index,
                q_rest=q_home_full,
            )
            if new_q is not None:
                _drive_joints_rad(art, joint_idx, new_q[joint_idx])

            if det is not None and det.found and det.area >= DETECTION_MIN_AREA_PX2:
                t_found = time.time()
                cur_q = np.array(art.get_joint_positions(), dtype=np.float64)
                found_q = cur_q[joint_idx].copy()
                phase = "found"
                print(f"\n[FOUND] cyan @ cx={det.cx:.1f} cy={det.cy:.1f} "
                      f"area={det.area:.0f}px²  "
                      f"target=({tx:+.3f}, {ty:+.3f}, {tz:+.3f})  t={t:.1f}s")

        elif phase == "found":
            if found_q is not None:
                _drive_joints_rad(art, joint_idx, found_q)
            if time.time() - t_found > FOUND_HOLD_SEC:
                print(f"  [found] hold {FOUND_HOLD_SEC:.1f}s done → resume scan")
                phase = "scan"
                t0_scan = time.time()
                t_found = None

        now = time.time()
        if bgr is not None and (now - last_dump_t) > DEBUG_DUMP_EVERY_SEC:
            last_dump_t = now
            ts = f"{int((now - (t0_scan or now)) * 10):05d}"
            cv2.imwrite(f"{DEBUG_DUMP_DIR}/frame_{ts}.png", bgr)
            if det is not None and det.mask is not None:
                cv2.imwrite(f"{DEBUG_DUMP_DIR}/mask_{ts}.png", det.mask)

        if now - last_log_t > 1.0:
            last_log_t = now
            l6_now_pos, _ = _get_link_world_pose(link6_path)
            cam_now_pos, _ = _get_link_world_pose(cam_path)
            det_txt = (f"cyan cx={det.cx:.0f} cy={det.cy:.0f} area={det.area:.0f}"
                       if (det and det.found) else "no cyan")
            mask_nz = int((det.mask > 0).sum()) if (det and det.mask is not None) else 0
            if phase == "scan" and t0_scan is not None:
                t = time.time() - t0_scan
                tx = Xc + AMP_X * math.sin(2.0 * math.pi * F_X * t)
                ty = Yc + AMP_Y * math.sin(2.0 * math.pi * F_Y * t)
                pos_e = ((tx - l6_now_pos[0])**2 + (ty - l6_now_pos[1])**2
                         + (Z_link6 - l6_now_pos[2])**2) ** 0.5
                print(f"  [{phase}] tgt=({tx:+.2f},{ty:+.2f},{Z_link6:.2f}) "
                      f"l6=({l6_now_pos[0]:+.2f},{l6_now_pos[1]:+.2f},{l6_now_pos[2]:+.2f}) "
                      f"err={pos_e*1000:.0f}mm  cam=({cam_now_pos[0]:+.2f},"
                      f"{cam_now_pos[1]:+.2f},{cam_now_pos[2]:+.2f})  "
                      f"{det_txt}  mask_nz={mask_nz}px")
            else:
                print(f"  [{phase}] l6=({l6_now_pos[0]:+.2f},{l6_now_pos[1]:+.2f},"
                      f"{l6_now_pos[2]:+.2f})  {det_txt}  mask_nz={mask_nz}px")


if __name__ == "__main__":
    main()
