"""Mars rover + Vehicle.usd + DLS IK based pick & place + wrist camera viewer.

DLS Jacobian IK 로 수동 state machine 구현. cube 좌표는 ground-truth 사용 (vision
은 표시용). CameraViewer 의 별도 OpenCV 윈도우 (wrist_camera + mask) 띄움 —
Isaac Sim 의 네이티브 wrist viewport panel 은 사용 안 함 (중복 방지).

Pipeline:
  Mars + Vehicle drop and settle + rover anchor
  → MOVE_TO_HOME (joints)
  → SEARCH (cyan tracker confirm)
  → APPROACH → DESCEND → GRASP → LIFT → MOVE_TO_GOAL → PLACE → RELEASE → RETREAT
  → DONE

실행:
    isaac-python ~/dev_ws/rover_ws/src/a2_isaac/isaac_manipulation/scripts/pickplace_visual_rover.py
옵션:
    --spawn x,y       Vehicle spawn XY (m)
    --goal-offset dx,dy,dz   pick 위치 기준 place 위치 offset
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

_TMP_CWD = tempfile.mkdtemp(prefix="isaac_pickplace_")
os.chdir(_TMP_CWD)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import cv2
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
_ext.set_extension_enabled_immediate("omni.kit.viewport.window", True)
_ext.set_extension_enabled_immediate("omni.kit.viewport.utility", True)

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleArticulation, SingleGeometryPrim
from isaacsim.core.utils.types import ArticulationAction

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from wrist_camera import WristCamera
from vision_tracker_cyan import CyanCubeTracker
from camera_viewer import CameraViewer
from realsense_mount import attach_realsense_d455


# ── 자산 경로 ────────────────────────────────────────────────────────
_PKG_PARENT = os.environ.get("A2_ISAAC_ROOT") or str(
    Path(__file__).resolve().parents[2]
)
A2_ROOT = Path(_PKG_PARENT)
MARS_WORLD_USD = A2_ROOT / "isaac_sim/worlds/mars_exploration_world.usd"
VEHICLE_USD = A2_ROOT.parent / "Vehicle.usd"

# ── Scene 파라미터 ───────────────────────────────────────────────────
SPAWN_X_DEFAULT, SPAWN_Y_DEFAULT = 4.5, -1.0
SPAWN_Z_DROP = 0.2
SETTLE_FRAMES = 120
CUBE_SIZE = 0.05
CUBE_DROP_OFFSET_XY = (0.7, 0.0)
CUBE_DROP_Z = 0.5
CUBE_COLOR = (0.0, 1.0, 1.0)

CAMERA_RESOLUTION = (640, 480)
# 4일차 reference 와 동일: RealSense D455 USD mesh 를 angle_bracket 에 부착하고
# 그 안의 OmniVision 카메라를 사용 (자체 clipping/aperture 가 올바르게 설정돼 있음).
CAM_OFFSET_T = (0.0, 0.045, 0.05)
CAM_OFFSET_RPY = (0.0, -90.0, 90.0)
CAM_SENSOR_EXTRA_RPY = (0.0, 0.0, 90.0)
OMNIVISION_CAM_NAME = "Camera_OmniVision_OV9782_Color"

# ── Manipulator 파라미터 ─────────────────────────────────────────────
EE_LINK_NAME = "link_6"
GRIPPER_BASE_LINK = "angle_bracket"
GRIPPER_GRASP_LINK = "gripper_body"
GRIP_JOINT_PATH = "/World/grip_fixed_joint"

# 작업 자세 — (0,0,90,0,90,0) 카메라 ↓ 자세
HOME_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
HOME_JOINT_POSITIONS_DEG = np.array([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
HOME_REACHED_JOINT_TOL_DEG = 1.5

# Gripper 제어 (joint 직접 driving, 단순)
GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]
GRIPPER_OPEN_RAD = np.array([0.0, 0.0])
GRIPPER_CLOSED_RAD = np.array([0.6, 0.6])

# ── DLS IK 파라미터 ──────────────────────────────────────────────────
IK_ALPHA = 0.4
IK_DAMPING = 0.10
IK_NULLSPACE_GAIN = 0.6
# Orientation error 가중치 — 자세는 nullspace bias 로 wrist joints 만 lock 하고
# IK 의 primary task 는 position 위주로 (1.0 = position 과 동일 가중).
IK_ORIENTATION_WEIGHT = 1.0
# Per-joint nullspace gain — j1 자유, j2/j3 매우 약한 bias (limit hit 방지),
# j4, j5, j6 (wrist) 는 HOME 유지.
IK_NULL_GAIN_PER_JOINT = np.array([0.0, 0.1, 0.1, 1.5, 1.5, 1.5])
# Joint angle limits — reach 자유롭게 매우 넓게.
IK_JOINT_LIMITS_DEG = [(-120, 120), (-60, 120), (-30, 180), (-120, 120), (-10, 170), (-120, 120)]
IK_POS_TOL = 0.04           # 4cm 이내 → 다음 phase (reach 한계 근처에서 stuck 방지)
IK_MAX_STEPS_PER_PHASE = 400  # ~7초 (60Hz). reach 한계까지 시도.
# Grasp 진입 안전 조건 — EE 가 cube 와 이 거리 안일 때만 grasp 진행. 초과 시 fail.
IK_GRASP_REACH_THRESHOLD = 0.10  # 10cm

# ── State machine 높이 (cube top 기준 상대 m) ───────────────────────
APPROACH_HEIGHT = 0.30      # cube top 위 30cm
# DESCEND 에서 추가 10cm 내려옴 (cube top 위 20cm). 0.15 이하로는 reach 한계
# 근처라 joint limit hit 위험.
GRASP_HEIGHT = 0.20
LIFT_HEIGHT = 0.45

GRIPPER_CLOSE_SETTLE_FRAMES = 30  # 그리퍼 close 후 settle


# ── 유틸 함수 ────────────────────────────────────────────────────────
def _find_prim_path_by_name(root_path, link_name):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == link_name:
            return str(prim.GetPath())
    return None


def _apply_terrain_collision(stage):
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


def _freeze_rover_drives(rover_prim):
    n = 0
    for prim in Usd.PrimRange(rover_prim):
        for drv_type in ("angular", "linear"):
            drv = UsdPhysics.DriveAPI.Get(prim, drv_type)
            if drv:
                drv.GetTargetVelocityAttr().Set(0.0)
                drv.GetStiffnessAttr().Set(0.0)
                drv.GetDampingAttr().Set(1e6)
                drv.GetMaxForceAttr().Set(1e7)
                n += 1
    return n


def _read_world_xyz(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = m.ExtractTranslation()
    return (float(t[0]), float(t[1]), float(t[2]))


def _read_world_pose_mat(prim_path):
    """prim 의 world pose 를 (position[3], rotmat[3,3]) 로 반환."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None, None
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = m.ExtractTranslation()
    pos = np.array([t[0], t[1], t[2]], dtype=np.float64)
    # Gf.Matrix4d 의 회전 부분 추출
    rotmat = np.array([
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ], dtype=np.float64)
    return pos, rotmat


def _get_camera_intrinsics(camera_obj, resolution):
    """(fx, fy, cx, cy) — Isaac Sim Camera 의 focal length + aperture 로 계산."""
    w, h = resolution
    # Isaac Sim Camera API 시도
    try:
        if hasattr(camera_obj, "get_intrinsics_matrix"):
            K = camera_obj.get_intrinsics_matrix()
            if K is not None:
                return float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
    except Exception:
        pass
    # USD prim attribute 로 직접 계산
    try:
        fl = float(camera_obj.get_focal_length())
        hAp = float(camera_obj.get_horizontal_aperture())
        vAp = float(camera_obj.get_vertical_aperture())
    except Exception:
        # 기본값 (Isaac Sim default Camera)
        fl, hAp, vAp = 18.147, 20.955, 15.290
    fx = fl * w / hAp
    fy = fl * h / vAp
    cx = w / 2.0
    cy = h / 2.0
    return fx, fy, cx, cy


def _deproject_pixel_to_world(px, py, depth, intrinsics,
                              cam_world_pos, cam_world_rotmat):
    """픽셀 (px, py) + depth → world XYZ.

    USD/OpenGL camera convention: camera 가 -Z 방향 봄, Y 는 위.
    OpenCV pixel convention: (px, py) origin top-left, py 증가 = 아래.
    """
    fx, fy, cx, cy = intrinsics
    # Camera frame (OpenGL):  X 오른쪽, Y 위, -Z forward
    X_cam = (px - cx) * depth / fx
    Y_cam = -(py - cy) * depth / fy   # py 증가 = 아래 = camera -Y
    Z_cam = -depth                     # camera 가 -Z 봄
    p_cam = np.array([X_cam, Y_cam, Z_cam], dtype=np.float64)
    p_world = cam_world_rotmat @ p_cam + cam_world_pos
    return p_world


def _attach_cube_to_link(stage, joint_path, link_path, cube_path):
    """Cube 를 link 의 **현재 상대 pose** 그대로 FixedJoint 부착 (teleport 없음).

    grasp 시점에서 cube 와 link 의 world 상대 pose 를 캡쳐 → 그 pose 가 그대로
    유지되도록 FixedJoint 생성. 따라서 cube 가 자석에 끌리듯 순간이동 안 함.
    EE 가 cube 위 20cm 에서 grasp 하면 cube 는 그 자리에 그대로 (그리퍼 아래
    20cm 에 매달려 있는 형태).
    """
    if stage.GetPrimAtPath(joint_path).IsValid():
        stage.RemovePrim(joint_path)
    link_prim = stage.GetPrimAtPath(link_path)
    cube_prim = stage.GetPrimAtPath(cube_path)
    if not link_prim.IsValid() or not cube_prim.IsValid():
        return False
    link_xf = UsdGeom.Xformable(link_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    cube_xf = UsdGeom.Xformable(cube_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    rel = cube_xf * link_xf.GetInverse()
    rel_pos = rel.ExtractTranslation()
    rel_rot = rel.ExtractRotationQuat()
    rot_imag = rel_rot.GetImaginary()
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(cube_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(rel_pos))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(
        rel_rot.GetReal(),
        float(rot_imag[0]), float(rot_imag[1]), float(rot_imag[2]),
    ))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    print(f"[grip] cube attached at captured pose (no teleport)")
    return True


def _detach_grip_joint(stage, joint_path):
    if stage.GetPrimAtPath(joint_path).IsValid():
        stage.RemovePrim(joint_path)
        print("[grip] cube detached")


# ── DLS Jacobian IK ─────────────────────────────────────────────────
def _get_link_world_pose(link_path):
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


def _quat_orientation_error(q_target, q_current):
    qw_t, qx_t, qy_t, qz_t = q_target
    qw_c, qx_c, qy_c, qz_c = q_current
    qw_e =  qw_t * qw_c + qx_t * qx_c + qy_t * qy_c + qz_t * qz_c
    qx_e = -qw_t * qx_c + qx_t * qw_c - qy_t * qz_c + qz_t * qy_c
    qy_e = -qw_t * qy_c + qx_t * qz_c + qy_t * qw_c - qz_t * qx_c
    qz_e = -qw_t * qz_c - qx_t * qy_c + qy_t * qx_c + qz_t * qw_c
    if qw_e < 0.0:
        qx_e, qy_e, qz_e = -qx_e, -qy_e, -qz_e
    return 2.0 * np.array([qx_e, qy_e, qz_e], dtype=np.float64)


def _get_jacobians(art):
    if hasattr(art, "get_jacobians"):
        try:
            v = art.get_jacobians()
            if v is not None:
                return v
        except Exception:
            pass
    av = getattr(art, "articulation_view", None) or getattr(art, "_articulation_view", None)
    if av and hasattr(av, "get_jacobians"):
        try:
            return av.get_jacobians()
        except Exception:
            pass
    return None


def _resolve_jacobian(art, ee_body_index):
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
                 alpha=IK_ALPHA, damping=IK_DAMPING,
                 ori_weight=IK_ORIENTATION_WEIGHT,
                 per_joint_null_gain=None):
    cur_pos, cur_quat = _get_link_world_pose(link_path)
    pos_err = target_pos - cur_pos
    rot_err = _quat_orientation_error(target_quat, cur_quat)
    # orientation weight 적용 → 자세가 위치보다 우선
    err = np.concatenate([pos_err, ori_weight * rot_err])

    # err 의 첫 3개 (position) 는 실제 mm 단위 보고용 (외부 caller)
    err_report = np.concatenate([pos_err, rot_err])

    J_all, col_offset = _resolve_jacobian(art, ee_body_index)
    if J_all is None:
        return None, err_report
    J_body = J_all[ee_body_index]
    arm_cols = [col_offset + i for i in joint_indices]
    J_arm = J_body[:, arm_cols]
    # Jacobian 도 orientation 행에 weight 곱
    J_arm_w = J_arm.copy()
    J_arm_w[3:6, :] *= ori_weight
    n = len(joint_indices)
    lam2 = damping * damping
    JJT = J_arm_w @ J_arm_w.T
    try:
        J_pinv = J_arm_w.T @ np.linalg.inv(JJT + lam2 * np.eye(6))
    except np.linalg.LinAlgError:
        return None, err_report
    dq_primary = J_pinv @ err
    cur_q_full = np.array(art.get_joint_positions(), dtype=np.float64)
    if q_rest is not None and k_null > 0:
        q_cur_arm = cur_q_full[joint_indices]
        q_rest_arr = np.asarray(q_rest, dtype=np.float64)
        if q_rest_arr.shape[0] == cur_q_full.shape[0]:
            q_rest_arm = q_rest_arr[joint_indices]
        else:
            q_rest_arm = q_rest_arr[:n]
        # per-joint nullspace gain (wrist 강하게)
        gains = per_joint_null_gain if per_joint_null_gain is not None \
            else np.full(n, k_null)
        bias = gains * (q_rest_arm - q_cur_arm)
        N = np.eye(n) - J_pinv @ J_arm_w
        dq = dq_primary + N @ bias
    else:
        dq = dq_primary
    new_q_full = cur_q_full.copy()
    for k, j in enumerate(joint_indices):
        new_q_full[j] += alpha * float(dq[k])
    # Joint limit clamping (joint_1~6 만)
    if len(joint_indices) == 6:
        for k, j in enumerate(joint_indices):
            lo_deg, hi_deg = IK_JOINT_LIMITS_DEG[k]
            new_q_full[j] = float(np.clip(new_q_full[j],
                                          np.deg2rad(lo_deg), np.deg2rad(hi_deg)))
    return new_q_full, err_report


def _drive_joints_rad(art, idx, joint_rad):
    rad = np.array(joint_rad, dtype=np.float32)
    art.get_articulation_controller().apply_action(ArticulationAction(
        joint_positions=rad, joint_indices=np.array(idx, dtype=np.int32),
    ))


def _get_body_names(art):
    for attr in ("body_names",):
        if hasattr(art, attr):
            try:
                v = getattr(art, attr)
                if v is not None:
                    return list(v)
            except Exception:
                pass
    av = getattr(art, "articulation_view", None) or getattr(art, "_articulation_view", None)
    if av is not None:
        for attr in ("body_names", "_body_names"):
            if hasattr(av, attr):
                try:
                    v = getattr(av, attr)
                    if v is not None:
                        return list(v)
                except Exception:
                    pass
    return None


def _find_ee_body_index(body_names, ee_name="link_6"):
    if not body_names:
        return None
    if ee_name in body_names:
        return body_names.index(ee_name)
    for i, n in enumerate(body_names):
        if n.endswith(f"/{ee_name}") or n.endswith(ee_name):
            return i
    return None


def _resolve_joint_indices(art, names):
    dof_names = list(art.dof_names) if art.dof_names else []
    idx = []
    for n in names:
        if n in dof_names:
            idx.append(dof_names.index(n))
        else:
            for i, dn in enumerate(dof_names):
                if dn.endswith(n):
                    idx.append(i)
                    break
            else:
                raise RuntimeError(f"joint {n} not in dof_names")
    return np.array(idx, dtype=np.int32)


# ── Scene build ──────────────────────────────────────────────────────
def build_scene(spawn_x, spawn_y):
    stage = omni.usd.get_context().get_stage()

    print("[1/3] mars world …")
    mars_prim = stage.DefinePrim("/World/Mars", "Xform")
    mars_prim.GetReferences().AddReference(str(MARS_WORLD_USD))
    for _ in range(8):
        simulation_app.update()
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene") \
        .CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    UsdPhysics.Scene(stage.GetPrimAtPath("/World/PhysicsScene")) \
        .CreateGravityMagnitudeAttr().Set(3.72)
    n_coll = _apply_terrain_collision(stage)
    print(f"  [PhysX] terrain mesh collision 적용: {n_coll}개 mesh")

    print("[2/3] Vehicle.usd …")
    outer_path = "/World/VehicleHolder"
    veh_path = f"{outer_path}/Vehicle"
    outer_prim = stage.DefinePrim(outer_path, "Xform")
    xform = UsdGeom.Xformable(outer_prim)
    xform.ClearXformOpOrder()
    translate_op = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
    xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1, 1, 1))
    veh_inner = stage.DefinePrim(veh_path, "Xform")
    veh_inner.GetReferences().AddReference(str(VEHICLE_USD))
    for _ in range(10):
        simulation_app.update()

    robot_root = _find_prim_path_by_name(veh_path, "m0609") or f"{veh_path}/m0609"
    rover_subtree = _find_prim_path_by_name(veh_path, "rover") or f"{veh_path}/rover"
    rover_body = _find_prim_path_by_name(veh_path, "Body") or f"{rover_subtree}/Body"
    angle_bracket = _find_prim_path_by_name(veh_path, "angle_bracket") \
                    or f"{veh_path}/onrobot_rg2ft/angle_bracket"
    gripper_body = _find_prim_path_by_name(veh_path, GRIPPER_GRASP_LINK) \
                   or f"{veh_path}/onrobot_rg2ft/{GRIPPER_GRASP_LINK}"

    internal = _read_world_xyz(rover_body)
    if internal is not None:
        dx = spawn_x - internal[0]
        dy = spawn_y - internal[1]
        dz = SPAWN_Z_DROP - internal[2]
        translate_op.Set(Gf.Vec3d(dx, dy, dz))
        print(f"  [vehicle] outer translate ({dx:+.3f},{dy:+.3f},{dz:+.3f})")
        for _ in range(5):
            simulation_app.update()

    rover_subtree_prim = stage.GetPrimAtPath(rover_subtree)
    if rover_subtree_prim.IsValid():
        n = _freeze_rover_drives(rover_subtree_prim)
        print(f"  [vehicle] rover drives frozen: {n}")

    # gripper finger drive 강화
    for jn in GRIPPER_JOINTS:
        jpath = _find_prim_path_by_name(veh_path, jn)
        if jpath:
            jprim = stage.GetPrimAtPath(jpath)
            for dt in ("angular", "linear"):
                drv = UsdPhysics.DriveAPI.Get(jprim, dt)
                if drv:
                    drv.GetMaxForceAttr().Set(100.0)
                    drv.GetStiffnessAttr().Set(1000.0)
                    drv.GetDampingAttr().Set(50.0)

    stage.SetEditTarget(Usd.EditTarget(stage.GetRootLayer()))
    stage.DefinePrim("/World/Joints", "Scope")

    print("[3/3] cube + camera …")
    cube_pos = (spawn_x + CUBE_DROP_OFFSET_XY[0],
                spawn_y + CUBE_DROP_OFFSET_XY[1], CUBE_DROP_Z)
    cube_material = PhysicsMaterial(
        prim_path="/World/Physics_Materials/cube_mat",
        static_friction=1.2, dynamic_friction=1.0, restitution=0.0,
    )
    DynamicCuboid(
        prim_path="/World/cyan_cube",
        name="cyan_cube",
        position=np.array(cube_pos),
        scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]),
        color=np.array(CUBE_COLOR),
        mass=0.01,
        physics_material=cube_material,
    )
    print(f"  [cube] drop @ {cube_pos}")

    finger_mat = PhysicsMaterial(
        prim_path="/World/Physics_Materials/finger_mat",
        static_friction=4.0, dynamic_friction=3.0, restitution=0.0,
    )
    for nm in ("left_inner_finger", "right_inner_finger",
               "left_inner_knuckle", "right_inner_knuckle"):
        path = _find_prim_path_by_name(veh_path, nm)
        if path:
            try:
                SingleGeometryPrim(prim_path=path,
                                   name=f"gripper_geom_{nm}").apply_physics_material(finger_mat)
            except Exception:
                pass

    # 4일차 reference 와 동일: RealSense D455 USD mesh 를 angle_bracket 에 부착.
    # 내부의 OmniVision 카메라 prim 을 wrist camera 로 사용.
    realsense_prim_path = attach_realsense_d455(
        parent_prim_path=angle_bracket,
        child_name="realsense_d455",
        translation=CAM_OFFSET_T,
        rpy_deg=CAM_OFFSET_RPY,
    )
    # USD reference 가 해결될 때까지 몇 프레임 대기
    for _ in range(8):
        simulation_app.update()
    # RealSense 내부의 RigidBody/Collision 은 비활성화 (angle_bracket 자체가 rigid)
    for _prim in Usd.PrimRange(stage.GetPrimAtPath(realsense_prim_path)):
        if _prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(_prim).GetRigidBodyEnabledAttr().Set(False)
        if _prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(_prim).GetCollisionEnabledAttr().Set(False)

    # OmniVision 내장 카메라 prim 찾기 + 추가 yaw 회전 (sensor 방향)
    cam_path = _find_prim_path_by_name(realsense_prim_path, OMNIVISION_CAM_NAME) \
               or f"{realsense_prim_path}/{OMNIVISION_CAM_NAME}"
    cam_prim_for_extra_rot = stage.GetPrimAtPath(cam_path)
    if cam_prim_for_extra_rot.IsValid():
        _xf = UsdGeom.Xformable(cam_prim_for_extra_rot)
        _existing = [op.GetOpName() for op in _xf.GetOrderedXformOps()]
        _rot_op = _xf.AddRotateZOp(UsdGeom.XformOp.PrecisionFloat, opSuffix="extra")
        _rot_op.Set(float(CAM_SENSOR_EXTRA_RPY[2]))
        from pxr import Vt
        cam_prim_for_extra_rot.GetAttribute("xformOpOrder").Set(
            Vt.TokenArray(_existing + [_rot_op.GetOpName()])
        )
    print(f"  [cam] RealSense D455 attached @ {realsense_prim_path}")
    print(f"  [cam] OmniVision camera @ {cam_path}")

    for _ in range(10):
        simulation_app.update()

    return {
        "robot_root": robot_root,
        "rover_body": rover_body,
        "rover_subtree": rover_subtree,
        "angle_bracket": angle_bracket,
        "gripper_body": gripper_body,
        "cube_prim": "/World/cyan_cube",
        "cam_path": cam_path,
        "veh_path": veh_path,
    }


# ── State machine 헬퍼 ──────────────────────────────────────────────
class PickPlaceStateMachine:
    """단순화된 pick&place state machine.

    각 상태에서 DLS IK 로 target_pos 추종. err < tol 이면 다음 상태.
    """

    STATES = [
        "MOVE_TO_HOME", "SEARCH", "APPROACH", "DESCEND",
        "GRASP_CLOSE", "ATTACH_LIFT", "MOVE_TO_GOAL",
        "PLACE_DESCEND", "RELEASE", "RETREAT", "DONE",
    ]

    def __init__(self, art, joint_idx, link6_path, ee_body_index, q_home,
                 quat_lock, cube_path, gripper_body_path, angle_bracket_path, goal_xy):
        self.art = art
        self.joint_idx = joint_idx
        self.link6_path = link6_path
        self.ee_body_index = ee_body_index
        self.q_home = q_home
        self.quat_lock = quat_lock
        self.cube_path = cube_path
        self.gripper_body_path = gripper_body_path
        self.angle_bracket_path = angle_bracket_path
        self.goal_xy = goal_xy
        self.state = "MOVE_TO_HOME"
        self.step_count = 0
        self.gripper_close_counter = 0
        self.cube_pick_xyz = None    # SEARCH 에서 캡쳐
        self.lift_z = None           # APPROACH 에서 결정 (cube top + lift)
        self.place_z = None          # cube top + small clearance at goal

    def _ik_to(self, target_pos):
        new_q, err6 = _ik_dls_step(
            self.art, self.link6_path, self.joint_idx,
            target_pos, self.quat_lock, self.ee_body_index,
            q_rest=self.q_home,
            per_joint_null_gain=IK_NULL_GAIN_PER_JOINT,
        )
        if new_q is not None:
            _drive_joints_rad(self.art, self.joint_idx, new_q[self.joint_idx])
        pos_err = float(np.linalg.norm(err6[:3]))
        return pos_err

    def _drive_gripper(self, joint_idx_grip, target):
        _drive_joints_rad(self.art, joint_idx_grip, target)

    def step(self, det, gripper_joint_idx, stage, ee_pos, vision_cube_xyz=None):
        s = self.state
        self.step_count += 1

        if s == "MOVE_TO_HOME":
            _drive_joints_rad(self.art, self.joint_idx, self.q_home[self.joint_idx])
            cur = np.array(self.art.get_joint_positions(), dtype=np.float64)
            err = np.max(np.abs(cur[self.joint_idx] - self.q_home[self.joint_idx]))
            if err < np.deg2rad(HOME_REACHED_JOINT_TOL_DEG):
                print(f"[state] MOVE_TO_HOME → SEARCH")
                self.state = "SEARCH"
                self.step_count = 0

        elif s == "SEARCH":
            # Vision-based: 카메라가 cube 검출 + depth deprojection 결과 (vision_cube_xyz) 가
            # 들어오면 그걸 사용. 검출 안 되면 timeout 후 ground-truth fallback.
            use_vision = (det is not None and det.found
                          and vision_cube_xyz is not None)
            if use_vision:
                cube_xyz = vision_cube_xyz.astype(np.float64).copy()
                src = f"VISION (px=({det.cx:.0f},{det.cy:.0f}), area={det.area:.0f})"
                # 비교용: ground-truth 위치 (로그용)
                gt_xyz = np.array(_read_world_xyz(self.cube_path), dtype=np.float64)
                err_vec = cube_xyz - gt_xyz
                err_str = f"  vs GT err=({err_vec[0]*1000:+.0f},{err_vec[1]*1000:+.0f},{err_vec[2]*1000:+.0f})mm"
            elif self.step_count > 120:
                cube_xyz = np.array(_read_world_xyz(self.cube_path), dtype=np.float64)
                src = "GROUND-TRUTH fallback (vision timeout)"
                err_str = ""
            else:
                return  # 계속 SEARCH 대기

            self.cube_pick_xyz = cube_xyz.copy()
            cube_top = cube_xyz[2] + CUBE_SIZE * 0.5
            self.lift_z = cube_top + LIFT_HEIGHT
            self.place_z = cube_top + GRASP_HEIGHT
            print(f"[state] SEARCH → APPROACH  cube={cube_xyz.round(3)}  "
                  f"lift_z={self.lift_z:.3f}  src={src}{err_str}")
            self.state = "APPROACH"
            self.step_count = 0

        elif s == "APPROACH":
            target = np.array([self.cube_pick_xyz[0], self.cube_pick_xyz[1],
                               self.cube_pick_xyz[2] + APPROACH_HEIGHT])
            pos_err = self._ik_to(target)
            if pos_err < IK_POS_TOL or self.step_count > IK_MAX_STEPS_PER_PHASE:
                print(f"[state] APPROACH → DESCEND  pos_err={pos_err*1000:.0f}mm")
                self.state = "DESCEND"
                self.step_count = 0

        elif s == "DESCEND":
            target = np.array([self.cube_pick_xyz[0], self.cube_pick_xyz[1],
                               self.cube_pick_xyz[2] + GRASP_HEIGHT])
            pos_err = self._ik_to(target)
            # 수렴: pos_err 가 충분히 작을 때만 grasp. timeout 만으로는 grasp 안 함.
            if pos_err < IK_POS_TOL:
                print(f"[state] DESCEND → GRASP_CLOSE  pos_err={pos_err*1000:.0f}mm OK")
                self.state = "GRASP_CLOSE"
                self.step_count = 0
                self.gripper_close_counter = 0
            elif self.step_count > IK_MAX_STEPS_PER_PHASE:
                # reach 한계로 못 내려옴 — pos_err 가 grasp threshold 이내면 grasp,
                # 아니면 ABORT (pipeline 중단)
                if pos_err < IK_GRASP_REACH_THRESHOLD:
                    print(f"[state] DESCEND → GRASP_CLOSE  pos_err={pos_err*1000:.0f}mm "
                          f"(timeout, marginal)")
                    self.state = "GRASP_CLOSE"
                    self.step_count = 0
                    self.gripper_close_counter = 0
                else:
                    print(f"\n[ABORT] DESCEND timed out at pos_err={pos_err*1000:.0f}mm "
                          f"(threshold {IK_GRASP_REACH_THRESHOLD*1000:.0f}mm). "
                          f"Arm 이 cube 까지 못 내려옴 — grasp 안전 거부.")
                    self.state = "DONE"
                    self.step_count = 0

        elif s == "GRASP_CLOSE":
            # 그리퍼 닫기 명령 유지
            self._drive_gripper(gripper_joint_idx, GRIPPER_CLOSED_RAD)
            self.gripper_close_counter += 1
            if self.gripper_close_counter > GRIPPER_CLOSE_SETTLE_FRAMES:
                # cube 를 gripper_body 에 FixedJoint 로 결속 (안정적 lift)
                # angle_bracket 의 +Z 방향이 일정한 finger 방향이라 더 예측 가능
                _attach_cube_to_link(stage, GRIP_JOINT_PATH,
                                     self.angle_bracket_path, self.cube_path)
                print(f"[state] GRASP_CLOSE → ATTACH_LIFT")
                self.state = "ATTACH_LIFT"
                self.step_count = 0

        elif s == "ATTACH_LIFT":
            target = np.array([self.cube_pick_xyz[0], self.cube_pick_xyz[1],
                               self.lift_z])
            pos_err = self._ik_to(target)
            self._drive_gripper(gripper_joint_idx, GRIPPER_CLOSED_RAD)
            if pos_err < IK_POS_TOL or self.step_count > IK_MAX_STEPS_PER_PHASE:
                print(f"[state] ATTACH_LIFT → MOVE_TO_GOAL  pos_err={pos_err*1000:.0f}mm")
                self.state = "MOVE_TO_GOAL"
                self.step_count = 0

        elif s == "MOVE_TO_GOAL":
            target = np.array([self.goal_xy[0], self.goal_xy[1], self.lift_z])
            pos_err = self._ik_to(target)
            self._drive_gripper(gripper_joint_idx, GRIPPER_CLOSED_RAD)
            if pos_err < IK_POS_TOL or self.step_count > IK_MAX_STEPS_PER_PHASE:
                print(f"[state] MOVE_TO_GOAL → PLACE_DESCEND  pos_err={pos_err*1000:.0f}mm")
                self.state = "PLACE_DESCEND"
                self.step_count = 0

        elif s == "PLACE_DESCEND":
            target = np.array([self.goal_xy[0], self.goal_xy[1], self.place_z])
            pos_err = self._ik_to(target)
            self._drive_gripper(gripper_joint_idx, GRIPPER_CLOSED_RAD)
            # PLACE 는 ABORT 안 함 — IK 가 못 닿아도 무조건 release (cube 떨굼)
            if pos_err < IK_POS_TOL or self.step_count > IK_MAX_STEPS_PER_PHASE:
                qual = "OK" if pos_err < IK_POS_TOL else "timeout, release anyway"
                print(f"[state] PLACE_DESCEND → RELEASE  pos_err={pos_err*1000:.0f}mm ({qual})")
                self.state = "RELEASE"
                self.step_count = 0

        elif s == "RELEASE":
            _detach_grip_joint(stage, GRIP_JOINT_PATH)
            self._drive_gripper(gripper_joint_idx, GRIPPER_OPEN_RAD)
            if self.step_count > 30:
                print(f"[state] RELEASE → RETREAT")
                self.state = "RETREAT"
                self.step_count = 0

        elif s == "RETREAT":
            target = np.array([self.goal_xy[0], self.goal_xy[1], self.lift_z])
            pos_err = self._ik_to(target)
            self._drive_gripper(gripper_joint_idx, GRIPPER_OPEN_RAD)
            if pos_err < IK_POS_TOL or self.step_count > IK_MAX_STEPS_PER_PHASE:
                print(f"[state] RETREAT → DONE  pos_err={pos_err*1000:.0f}mm")
                self.state = "DONE"
                self.step_count = 0


# ── Main ─────────────────────────────────────────────────────────────
def _run():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spawn", type=str, default=f"{SPAWN_X_DEFAULT},{SPAWN_Y_DEFAULT}")
    ap.add_argument("--goal-offset", type=str, default="0.0,-0.3,0.0",
                    help="cube pick 위치 기준 place 위치 offset (dx,dy,dz). "
                         "rover 반대 방향으로 두는 게 안전")
    args = ap.parse_args()
    spawn_xy = tuple(float(v) for v in args.spawn.split(","))
    goal_offset = np.array([float(v) for v in args.goal_offset.split(",")])

    world = World(stage_units_in_meters=1.0)
    paths = build_scene(spawn_xy[0], spawn_xy[1])

    art = SingleArticulation(prim_path=paths["robot_root"], name="m0609_art")
    world.scene.add(art)

    print("\n[World] reset …")
    world.reset()
    stage = omni.usd.get_context().get_stage()
    rover_subtree_prim = stage.GetPrimAtPath(paths["rover_subtree"])
    if rover_subtree_prim.IsValid():
        _freeze_rover_drives(rover_subtree_prim)

    print(f"\n[Settle] {SETTLE_FRAMES} frames …")
    if not world.is_playing():
        world.play()
    for k in range(SETTLE_FRAMES):
        world.step(render=True)
        if k % 30 == 0:
            rxyz = _read_world_xyz(paths["rover_body"])
            cxyz = _read_world_xyz(paths["cube_prim"])
            print(f"  [settle {k:3d}] rover z={rxyz[2]:+.3f}  cube z={cxyz[2]:+.3f}")

    rover_settled = _read_world_xyz(paths["rover_body"])
    cube_settled = _read_world_xyz(paths["cube_prim"])
    print(f"  [settle] rover @ ({rover_settled[0]:+.3f},{rover_settled[1]:+.3f},"
          f"{rover_settled[2]:+.3f})")
    print(f"  [settle] cube  @ ({cube_settled[0]:+.3f},{cube_settled[1]:+.3f},"
          f"{cube_settled[2]:+.3f})")

    stage.SetEditTarget(Usd.EditTarget(stage.GetRootLayer()))
    anchor = UsdPhysics.FixedJoint.Define(stage, "/World/Joints/RoverAnchor")
    anchor.CreateBody1Rel().SetTargets([Sdf.Path(paths["rover_body"])])
    anchor.CreateLocalPos0Attr().Set(Gf.Vec3f(*rover_settled))
    anchor.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    anchor.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    anchor.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    anchor.CreateBreakForceAttr().Set(float("inf"))
    anchor.CreateBreakTorqueAttr().Set(float("inf"))
    print("  [anchor] rover FixedJoint applied")

    # Joint indices
    joint_idx = _resolve_joint_indices(art, HOME_JOINT_NAMES)
    gripper_joint_idx = _resolve_joint_indices(art, GRIPPER_JOINTS)
    body_names = _get_body_names(art)
    ee_body_index = _find_ee_body_index(body_names, EE_LINK_NAME)
    link6_path = _find_prim_path_by_name(paths["robot_root"], EE_LINK_NAME) \
                 or f"{paths['robot_root']}/{EE_LINK_NAME}"
    print(f"\n[Joints] arm idx = {joint_idx.tolist()}")
    print(f"  gripper idx = {gripper_joint_idx.tolist()}")
    print(f"  link_6 body idx = {ee_body_index}  (path={link6_path})")

    # HOME pose 적용 + lock orientation 캡쳐
    home_q_arm = np.deg2rad(HOME_JOINT_POSITIONS_DEG)
    _drive_joints_rad(art, joint_idx, home_q_arm)
    try:
        cur = np.array(art.get_joint_positions(), dtype=np.float32).copy()
        for k, j in enumerate(joint_idx):
            cur[j] = home_q_arm[k]
        art.set_joint_positions(cur)
    except Exception:
        pass
    for _ in range(30):
        world.step(render=True)

    q_home_full = np.array(art.get_joint_positions(), dtype=np.float64)
    _, link6_home_quat = _get_link_world_pose(link6_path)
    print(f"  [home] link_6 quat lock = ({link6_home_quat[0]:+.3f},"
          f"{link6_home_quat[1]:+.3f},{link6_home_quat[2]:+.3f},"
          f"{link6_home_quat[3]:+.3f})")

    # 카메라 위치 진단 (RealSense USD 의 내장 OmniVision 카메라 사용 — 자체 orient OK)
    cam_world_xyz = _read_world_xyz(paths["cam_path"])
    if cam_world_xyz:
        print(f"  [cam] camera world pos = ({cam_world_xyz[0]:+.3f},"
              f"{cam_world_xyz[1]:+.3f},{cam_world_xyz[2]:+.3f})")

    # ── Wrist camera (CameraViewer 의 별도 OpenCV 윈도우 — Isaac native viewport 안 띄움) ──
    wrist_cam = WristCamera.from_existing_prim(
        prim_path=paths["cam_path"], resolution=CAMERA_RESOLUTION)
    wrist_cam.initialize()
    tracker = CyanCubeTracker()
    viewer = CameraViewer(enabled=True, show_mask=True)

    # Camera intrinsics (한 번만 계산)
    intrinsics = _get_camera_intrinsics(wrist_cam.camera, CAMERA_RESOLUTION)
    print(f"  [cam] intrinsics fx={intrinsics[0]:.1f} fy={intrinsics[1]:.1f} "
          f"cx={intrinsics[2]:.1f} cy={intrinsics[3]:.1f}")

    # Goal position: cube settled XY + offset
    goal_xy = (cube_settled[0] + goal_offset[0],
               cube_settled[1] + goal_offset[1])
    print(f"\n[Goal] cube → ({goal_xy[0]:.3f}, {goal_xy[1]:.3f})")

    sm = PickPlaceStateMachine(
        art=art, joint_idx=joint_idx, link6_path=link6_path,
        ee_body_index=ee_body_index,
        q_home=q_home_full, quat_lock=link6_home_quat,
        cube_path=paths["cube_prim"],
        gripper_body_path=paths["gripper_body"],
        angle_bracket_path=paths["angle_bracket"],
        goal_xy=goal_xy,
    )

    print("\n[Pipeline 시작]\n")
    last_log_t = time.time()
    was_playing = True

    while simulation_app.is_running():
        world.step(render=True)
        is_playing = world.is_playing()

        # Stop → Play 재진입 시 state machine + grip joint 리셋
        if is_playing and not was_playing:
            print("\n[reset] Play 재진입 — state machine 초기화")
            _detach_grip_joint(stage, GRIP_JOINT_PATH)
            sm.state = "MOVE_TO_HOME"
            sm.step_count = 0
            sm.cube_pick_xyz = None
            sm.lift_z = None
            sm.place_z = None
            sm.gripper_close_counter = 0
            was_playing = True
            continue
        if not is_playing:
            was_playing = False
            continue
        was_playing = True

        # ── 카메라 + 검출 ─────────────────────────────────
        rgb = wrist_cam.get_rgb()
        det = None
        if rgb is not None and rgb.size > 0:
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            det = tracker.detect(bgr)

        ee_pos, _ = _get_link_world_pose(link6_path)

        # ── Vision-driven cube position 추정 (det.found 일 때만) ────────
        vision_cube_xyz = None
        if det is not None and det.found and sm.state == "SEARCH":
            try:
                # depth 가져오기 (Isaac Sim Camera 의 distance_to_image_plane)
                frame_data = wrist_cam.camera.get_current_frame()
                depth_map = frame_data.get("distance_to_image_plane", None)
                if depth_map is not None and depth_map.size > 0:
                    px, py = int(round(det.cx)), int(round(det.cy))
                    H, W = depth_map.shape[:2]
                    if 0 <= px < W and 0 <= py < H:
                        d = float(depth_map[py, px])
                        if np.isfinite(d) and d > 0.01 and d < 100.0:
                            cam_pos, cam_rot = _read_world_pose_mat(paths["cam_path"])
                            if cam_pos is not None:
                                vision_cube_xyz = _deproject_pixel_to_world(
                                    px, py, d, intrinsics, cam_pos, cam_rot)
            except Exception as e:
                print(f"  [vision] deproject error: {e.__class__.__name__}: {e}")

        # ── CameraViewer 갱신 (RGB + bbox + crosshair + state overlay) ──
        key = viewer.update(
            rgb, det,
            state_str=sm.state,
            extra_lines=[
                f"ee=({ee_pos[0]:+.2f},{ee_pos[1]:+.2f},{ee_pos[2]:+.2f})",
                f"step={sm.step_count}",
                (f"cyan area={det.area:.0f} cx={det.cx:.0f} cy={det.cy:.0f}"
                 if det and det.found else "no cyan"),
                (f"vision_xyz=({vision_cube_xyz[0]:+.2f},{vision_cube_xyz[1]:+.2f},"
                 f"{vision_cube_xyz[2]:+.2f})" if vision_cube_xyz is not None else ""),
            ],
        )
        if key == ord('q'):
            print("[viewer] q pressed → exit")
            break

        sm.step(det, gripper_joint_idx, stage, ee_pos,
                vision_cube_xyz=vision_cube_xyz)

        now = time.time()
        if now - last_log_t > 1.0:
            last_log_t = now
            cube_xyz = _read_world_xyz(paths["cube_prim"])
            cam_xyz = _read_world_xyz(paths["cam_path"])
            det_str = (f"cyan area={det.area:.0f}" if det and det.found else "no cyan")
            # 방어: stop/play 직후엔 physics view 미초기화 → joint pos 비어있을 수 있음
            try:
                _raw = art.get_joint_positions()
                cur_q = np.asarray(_raw, dtype=np.float64) if _raw is not None else None
                if cur_q is not None and cur_q.ndim > 0 and cur_q.size >= len(joint_idx):
                    j_deg = [np.rad2deg(cur_q[i]) for i in joint_idx]
                    j_str = "j=[" + ",".join(f"{v:+.0f}" for v in j_deg) + "]"
                else:
                    j_str = "j=N/A"
            except Exception:
                j_str = "j=N/A"
            print(f"  [{sm.state:14s}] ee=({ee_pos[0]:+.2f},{ee_pos[1]:+.2f},{ee_pos[2]:+.2f}) "
                  f"cube=({cube_xyz[0]:+.2f},{cube_xyz[1]:+.2f},{cube_xyz[2]:+.2f}) "
                  f"{j_str} {det_str}")


def main():
    try:
        _run()
    except Exception as e:
        import traceback
        print(f"\n[FATAL] {e.__class__.__name__}: {e}", flush=True)
        traceback.print_exc()
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            simulation_app.close()
        except Exception:
            pass
        os._exit(0)


if __name__ == "__main__":
    main()
