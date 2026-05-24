"""Rover YOLO Demo — terrain_00022 + Vehicle v2 + best.pt 실시간 객체 탐지 + Pick&Place.

흐름:
  AUTOPILOT (nav cam YOLO) → 가장 가까운 mineral 까지 접근 →
  ENGAGE_DISTANCE(0.9m) 진입 시 creep 속도로 감속 → mineral별 STOP_DISTANCE 도달 →
  MANIPULATION (HOME_PRE → WRIST_SERVO: wrist cam bbox XY 보정 →
                APPROACH_DESCEND: 한 번에 GRASP_HEIGHT → GRASP_CLOSE → ATTACH_LIFT →
                JS_PRE → RELEASE (RearBasket) → JS_POST → DONE) →
  RETREAT → 다음 mineral 탐색 (autopilot 재개)

사용:
  isaac-python scripts/rover_yolo_demo.py
  isaac-python scripts/rover_yolo_demo.py --conf 0.3 --interval 2

조작:
  W / S — 전진 / 후진 (autopilot/manipulation override)
  A / D — 좌 / 우 회전
  Space — 정지
  T     — autopilot 토글 (default ON)
  M     — manipulation 강제 abort 및 autopilot 으로 복귀
  ESC   — 종료
  P     — 현재 view + detection screenshot 저장

화면:
  Isaac Sim viewport — 탑뷰
  omni.ui 윈도 1 — nav cam + YOLO bbox + 거리
  omni.ui 윈도 2 — wrist cam + YOLO bbox + 거리 (grasp 진행 가시화)
"""
from __future__ import annotations

import argparse
import os
import sys
import tempfile
import time
from pathlib import Path

try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

# ── argparse pre ──────────────────────────────────────────────────────
_ap = argparse.ArgumentParser(add_help=False)
_ap.add_argument("--model", type=str,
                 default="/home/rokey/dev_ws/rover_ws/src/a2_isaac/isaac_perception/models/mineral_yolo_best.pt")
_ap.add_argument("--conf", type=float, default=0.5)
_ap.add_argument("--iou", type=float, default=0.45)
_ap.add_argument("--interval", type=int, default=2,
                 help="N step 마다 inference (1=매 step, 큰 값 → 적은 부하)")
_ap.add_argument("--resolution", type=str, default="1280x720")
_ap.add_argument("--out", type=str,
                 default="/home/rokey/dev_ws/rover_ws/src/a2_isaac/isaac_perception/runs/mineral/demo_shots")
args, _ = _ap.parse_known_args()

# ── SimulationApp ────────────────────────────────────────────────────
os.chdir(tempfile.mkdtemp(prefix="rover_yolo_demo_"))
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import carb.input
import cv2
import numpy as np
import omni.appwindow
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera


# ── 자산 경로 ──────────────────────────────────────────────────────
PKG_ROOT    = Path("/home/rokey/dev_ws/rover_ws/src/a2_isaac")
TERRAIN_USD = PKG_ROOT / "isaac_sim/worlds/terrain_00022.usd"
VEHICLE_USD = PKG_ROOT / "isaac_sim/assets/vehicle/vehicle_v2.usd"
MINERAL_ASSETS_DIR = PKG_ROOT / "isaac_sim/assets/markers/tier2_mineral"
# vehicle_v2.usd 내부 카메라 경로 (default prim = /Root, 참조 후 /World/Vehicle/Vehicle/...)
NAV_CAM_REL   = "Vehicle/rover/Body/Camera"
WRIST_CAM_REL = "Vehicle/onrobot_rg2ft/angle_bracket/realsense_d455/RSD455/Camera_OmniVision_OV9782_Color"
M0609_REL     = "Vehicle/m0609"
EE_LINK_REL   = "Vehicle/m0609/link_6"
ANGLE_BRACKET_REL = "Vehicle/onrobot_rg2ft/angle_bracket"
LEFT_FINGER_REL   = "Vehicle/onrobot_rg2ft/left_inner_finger"
RIGHT_FINGER_REL  = "Vehicle/onrobot_rg2ft/right_inner_finger"
REAR_BASKET_REL   = "Vehicle/rover/Body/RearBasket"

# 차량 조작 파라미터
LIN_SPEED   = 1.5    # m/s 전진/후진
ANG_SPEED   = 0.8    # rad/s 회전

# ── Mineral spawn 정의 (manual_capture.py 와 동일 좌표) ───────────────
MINERAL_PLACEMENTS = [
    # (cls_id, label, usd_filename, world_xyz, color RGBA)
    (0, "blue_mineral",   "mineral_blue.usd",   (4.5, -1.0, 1.0)),
    (1, "yellow_mineral", "mineral_yellow.usd", (4.5,  1.0, 1.0)),
    (2, "green_gas",      "mineral_red.usd",    (4.5,  0.0, 1.0)),
]
MINERAL_MASS   = 0.01   # kg
MINERAL_RADIUS = 0.05   # m (geom 자체는 sphere; collision approximation 용)

# Terrain 내장 mineral prim 의 런타임 scale (terrain_00022.usd 원본 미수정)
MINERAL_SCALE_PER_CLASS = {
    "blue_mineral":   1.0,
    "yellow_mineral": 1.0,
    "green_gas":      0.5,   # green cube 가 너무 커서 grasp 어려움 → 절반 축소
}

# Invisible Cube sub-prim 만 별도 scale (visible mesh 는 그대로, collision/grasp surface 만 키움)
# blue/yellow 의 cube 가 visible mesh 보다 작아 finger 가 못 잡는 경우 → cube 만 확대해서 grasp 면적 확보.
# 시각적으로는 finger 가 mesh 옆 빈 공간을 잡는 모양이 될 수 있음 (cube > mesh).
CUBE_SCALE_PER_CLASS = {
    "blue_mineral":   1.0,    # original 크기
    "yellow_mineral": 1.0,
    "green_gas":      1.5,
}

# ── Manipulator 파라미터 (pickplace_visual_rover.py 와 동일) ─────────
HOME_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
HOME_JOINT_POSITIONS_DEG = np.array([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
HOME_REACHED_JOINT_TOL_DEG = 1.5

GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]
# finger_joint URDF 범위 [0, 1.18] rad — 0 rad = max open (width 110mm), 1.18 rad = 완전 닫힘
GRIPPER_OPEN_RAD   = np.array([0.0, 0.0])     # width ≈ 110 mm (max open)
GRIPPER_CLOSED_RAD = np.array([0.6, 0.6])     # width ≈ 50 mm
GRIPPER_CLOSE_SETTLE_FRAMES = 30

# DLS IK 파라미터
IK_ALPHA = 0.4
IK_DAMPING = 0.10
IK_NULLSPACE_GAIN = 0.6
IK_ORIENTATION_WEIGHT = 1.0
IK_NULL_GAIN_PER_JOINT = np.array([0.0, 0.05, 0.05, 0.2, 0.2, 0.2])   # wrist 의 home bias 약화 → down-reach 자유
IK_JOINT_LIMITS_DEG = [(-360, 360), (-125, 125), (-150, 150),
                       (-360, 360), (-135, 135), (-360, 360)]         # M0609 spec 로 확장 (down-reach 위해 joint 5 범위 ↑)
IK_POS_TOL = 0.04
IK_MAX_STEPS_PER_PHASE = 400
IK_GRASP_REACH_THRESHOLD = 0.10

# State machine 높이 — TCP(finger midpoint) 기준 mineral 위 상대 m.
# IK 가 TCP 를 target 에 위치시키도록 자동 보정함 (TCP_OFFSET_LOCAL 사용).
APPROACH_HEIGHT      = 0.30   # (legacy, unused)
GRASP_HEIGHT         = 0.20   # (legacy, unused — HOVER_ABOVE_MINERAL 로 대체)
HOVER_ABOVE_MINERAL  = 0.10   # default — class_hint 없거나 lookup 실패 시
HOVER_ABOVE_MINERAL_PER_CLASS = {
    "blue_mineral":   0.05,   # cube 위 4cm — friction grasp 위해 finger 가 cube 안 깊이 들어가게
    "yellow_mineral": 0.05,
    "green_gas":      0.10,   # green cube 는 더 큰 hover 필요 (0.10 이면 finger 가 옆면을 쳐서 튕김)
}
LIFT_HEIGHT          = 0.45   # grasp 후 finger 중심을 mineral 위 45cm 까지 들어올림

# RearBasket release — basket world top 기준 z offset (user 가 0.2 추천)
BASKET_RELEASE_Z_OFFSET = 0.20

GRIP_JOINT_PATH = "/World/grip_fixed_joint"

# ── Joint-space place trajectory (deg) ──────────────────────────────
# ATTACH_LIFT 직후부터 RELEASE 까지의 자세 시퀀스. 단순화된 3-step PRE / 2-step POST.
PLACE_TRAJ_PRE_DEG = [
    [  0.0,  0.0, 90.0, 0.0, 90.0, 0.0],   # HOME (lift 후 정렬)
    [180.0,  0.0, 90.0, 0.0, 90.0, 0.0],   # joint_1: 0 → 180 (베이스 뒤)
    [180.0, 12.5, 90.0, 0.0, 60.0, 0.0],   # joint_2 + joint_5 동시 (어깨 굽힘 + 손목 dump)
]
PLACE_TRAJ_POST_DEG = [
    [180.0,  0.0, 90.0, 0.0, 90.0, 0.0],   # 어깨 + 손목 동시 복귀
    [  0.0,  0.0, 90.0, 0.0, 90.0, 0.0],   # HOME 복귀
]
WAYPOINT_TOL_DEG = 2.5     # 다음 waypoint 진입 조건 (max joint err ≤ 2.5°)
WAYPOINT_TIMEOUT = 250     # ~4초. 못 도달해도 다음으로
WAYPOINT_INTERP_FRAMES = 120  # waypoint 간 선형 보간 frame 수 (60fps 기준 ~2초). 시각적 관찰 가능

# Friction grasp PhysicsMaterial 값 (m0609_pick_place_fixed_target.py 와 동일)
MINERAL_FRICTION = (1.2, 1.0, 0.0)   # static, dynamic, restitution (m0609_pick_place_fixed_target.py default)
FINGER_FRICTION  = (1.8, 1.4, 0.0)


def _create_physics_material(stage, mat_path, sf, df, rest=0.0):
    """USD physics material 생성 (static / dynamic friction + restitution)."""
    mat_prim = stage.DefinePrim(mat_path, "Material")
    phys = UsdPhysics.MaterialAPI.Apply(mat_prim)
    phys.CreateStaticFrictionAttr().Set(float(sf))
    phys.CreateDynamicFrictionAttr().Set(float(df))
    phys.CreateRestitutionAttr().Set(float(rest))
    return UsdShade.Material(mat_prim)


def _bind_physics_material_to_subtree(stage, root_path, mat):
    """root 이하 모든 Mesh prim 에 physics material bind (purpose='physics')."""
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return 0
    n = 0
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() != "Mesh":
            continue
        try:
            binding = UsdShade.MaterialBindingAPI.Apply(prim)
            binding.Bind(mat,
                         bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                         materialPurpose="physics")
            n += 1
        except Exception:
            pass
    return n


def build_scene(stage):
    print("[1/4] terrain 로드 …")
    terrain_prim = stage.DefinePrim("/World/Terrain", "Xform")
    terrain_prim.GetReferences().AddReference(str(TERRAIN_USD))
    for _ in range(20):
        simulation_app.update()

    # [2/4] mineral 은 spawn 하지 않음 — terrain_00022.usd 내장 mineral 들 사용
    # (/World/Terrain/Minerals/blue_* 등). manipulation target 은 nav cam YOLO+depth 로 deproject.
    print("[2/4] mineral 은 terrain 내장 prim 사용 (별도 spawn 안 함)")
    mineral_paths = []  # 미사용 — autopilot 의 nav cam det 만으로 target 결정

    # ── class 별 런타임 scale 적용 (USD 원본 수정 없이 prim xformOp:scale 만 변경) ──
    # prim name prefix → class 매핑 (terrain_00022.usd 의 명명 규칙):
    #   blue_*   → blue_mineral
    #   yellow_* → yellow_mineral
    #   red_*    → green_gas (시각 클래스는 green 이지만 prim 이름은 red)
    _name_to_class = {"blue": "blue_mineral", "yellow": "yellow_mineral", "red": "green_gas"}
    _minerals_root = stage.GetPrimAtPath("/World/Terrain/Minerals")
    _scale_counts = {"blue_mineral": 0, "yellow_mineral": 0, "green_gas": 0}
    if _minerals_root.IsValid():
        for _child in _minerals_root.GetChildren():
            _nm = _child.GetName().lower()
            _cls = None
            for _pref, _c in _name_to_class.items():
                if _nm.startswith(_pref):
                    _cls = _c
                    break
            if _cls is None:
                continue
            _s = float(MINERAL_SCALE_PER_CLASS.get(_cls, 1.0))
            if abs(_s - 1.0) < 1e-6:
                continue   # scale=1 이면 skip
            _xf = UsdGeom.Xformable(_child)
            _ops = list(_xf.GetOrderedXformOps())
            _scale_op = next((o for o in _ops if o.GetOpType() == UsdGeom.XformOp.TypeScale), None)
            if _scale_op is None:
                _scale_op = _xf.AddScaleOp(UsdGeom.XformOp.PrecisionFloat)
                # 새로 추가했으면 op order 재구성 (translate/rotate 뒤에 scale 위치)
                _xf.SetXformOpOrder(list(_xf.GetOrderedXformOps()))
            _scale_op.Set(Gf.Vec3f(_s, _s, _s))
            _scale_counts[_cls] += 1
    print(f"  [mineral scale] applied: {_scale_counts} "
          f"(per_class={MINERAL_SCALE_PER_CLASS})")

    # ── invisible Cube sub-prim 만 별도 scale (visible mesh 그대로, collision/grasp surface 만) ──
    _cube_scale_counts = {"blue_mineral": 0, "yellow_mineral": 0, "green_gas": 0}
    _cube_skipped = 0
    if _minerals_root.IsValid():
        for _child in _minerals_root.GetChildren():
            _nm = _child.GetName().lower()
            _cls = None
            for _pref, _c in _name_to_class.items():
                if _nm.startswith(_pref):
                    _cls = _c
                    break
            if _cls is None:
                continue
            _cs = float(CUBE_SCALE_PER_CLASS.get(_cls, 1.0))
            if abs(_cs - 1.0) < 1e-6:
                continue
            # 하위 트리에서 Cube sub-prim 찾기 (첫 번째 매칭만)
            for _sub in Usd.PrimRange(_child):
                if _sub == _child:
                    continue
                if _sub.GetName().lower() != "cube":
                    continue
                try:
                    _xfsub = UsdGeom.Xformable(_sub)
                    _ops_sub = list(_xfsub.GetOrderedXformOps())
                    _ssub_op = next((o for o in _ops_sub
                                     if o.GetOpType() == UsdGeom.XformOp.TypeScale), None)
                    if _ssub_op is None:
                        _ssub_op = _xfsub.AddScaleOp(UsdGeom.XformOp.PrecisionFloat)
                        _xfsub.SetXformOpOrder(list(_xfsub.GetOrderedXformOps()))
                    _ssub_op.Set(Gf.Vec3f(_cs, _cs, _cs))
                    _cube_scale_counts[_cls] += 1
                except Exception as _e:
                    _cube_skipped += 1
                    print(f"  [cube scale SKIP] {_sub.GetPath()}: "
                          f"{_e.__class__.__name__}: {_e}")
                break
    print(f"  [cube scale] applied: {_cube_scale_counts} "
          f"(per_class={CUBE_SCALE_PER_CLASS}, skipped={_cube_skipped})")

    # ── Friction grasp 용 PhysicsMaterial — mineral 들에 강한 마찰력 부여 ──
    _mineral_mat = _create_physics_material(
        stage, "/World/PhysicsMaterials/mineral_mat",
        sf=MINERAL_FRICTION[0], df=MINERAL_FRICTION[1], rest=MINERAL_FRICTION[2])
    _n_mat = _bind_physics_material_to_subtree(stage, "/World/Terrain/Minerals", _mineral_mat)
    print(f"  [friction] mineral PhysicsMaterial sf={MINERAL_FRICTION[0]} "
          f"df={MINERAL_FRICTION[1]} bound to {_n_mat} Mesh prims")

    print("[3/4] vehicle_v2 로드 …")
    veh_prim_path = "/World/Vehicle"
    veh_root = stage.DefinePrim(veh_prim_path, "Xform")
    veh_root.GetReferences().AddReference(str(VEHICLE_USD))
    # 초기 xform 설정 (terrain 위)
    veh_xf = UsdGeom.Xformable(veh_root)
    veh_xf.ClearXformOpOrder()
    t_op = veh_xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    r_op = veh_xf.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble)
    t_op.Set(Gf.Vec3d(0.0, 0.0, 1.5))
    r_op.Set(0.0)
    for _ in range(20):
        simulation_app.update()

    # 모든 vehicle body 를 dynamic 으로 유지 (pickplace_visual_rover.py 스타일).
    # PhysX articulation 이 정상 동작하려면 root link 가 dynamic 이어야 함.
    # 시작 시 중력으로 ground 까지 settle → RoverAnchor 로 fix → manipulation IK 가능.
    # Terrain mesh 에 CollisionAPI 적용 (rover 가 지면 위에 안착하도록)
    _n_coll = 0
    _terrain_root = stage.GetPrimAtPath("/World/Terrain")
    if _terrain_root.IsValid():
        for prim in Usd.PrimRange(_terrain_root):
            if prim.GetTypeName() != "Mesh":
                continue
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim)
            if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                UsdPhysics.MeshCollisionAPI.Apply(prim)
            mca = UsdPhysics.MeshCollisionAPI(prim)
            approx = mca.GetApproximationAttr() or mca.CreateApproximationAttr()
            approx.Set("meshSimplification")
            _n_coll += 1
    print(f"  terrain mesh collision applied: {_n_coll}")
    # PhysicsScene 의 gravity 설정 — Mars 환경 (3.72 m/s^2)
    if not stage.GetPrimAtPath("/World/PhysicsScene").IsValid():
        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr().Set(3.72)
        print(f"  PhysicsScene 생성 (Mars gravity 3.72 m/s²)")

    print("[4/4] nav camera 경로 확인 …")
    cam_path = f"{veh_prim_path}/{NAV_CAM_REL}"
    cam_prim = stage.GetPrimAtPath(cam_path)
    if not cam_prim.IsValid():
        # default prim 이 /Root 가 아닌 경우 대비
        for prim in Usd.PrimRange(stage.GetPrimAtPath(veh_prim_path)):
            if prim.GetTypeName() == "Camera" and "rover/Body/Camera" in str(prim.GetPath()):
                cam_path = str(prim.GetPath())
                break
    print(f"  nav camera   → {cam_path}")

    # wrist cam (RealSense D455 Color) 도 찾기
    wrist_cam_path = f"{veh_prim_path}/{WRIST_CAM_REL}"
    if not stage.GetPrimAtPath(wrist_cam_path).IsValid():
        # 1) 이름 키워드 매칭 (OmniVision 또는 OV9782 또는 RSD455 하위의 첫 Camera)
        candidates = []
        for prim in Usd.PrimRange(stage.GetPrimAtPath(veh_prim_path)):
            if prim.GetTypeName() != "Camera":
                continue
            p = str(prim.GetPath())
            if any(k in p for k in ("OmniVision", "OV9782", "realsense_d455", "RSD455")):
                candidates.append(p)
        if candidates:
            wrist_cam_path = candidates[0]
            if len(candidates) > 1:
                print(f"  [warn] wrist cam 후보 {len(candidates)} 개 — 첫 번째 사용: {candidates}")
        else:
            # 2) 그래도 못 찾으면 angle_bracket 서브트리의 모든 Camera 나열
            ab_path = f"{veh_prim_path}/{ANGLE_BRACKET_REL}"
            ab_prim = stage.GetPrimAtPath(ab_path)
            ab_cams = []
            if ab_prim.IsValid():
                for prim in Usd.PrimRange(ab_prim):
                    if prim.GetTypeName() == "Camera":
                        ab_cams.append(str(prim.GetPath()))
            print(f"  [ERROR] wrist cam prim 못 찾음. angle_bracket 서브트리 카메라들: {ab_cams or '(없음)'}")
            print(f"          → vehicle_v2.usd 에 D455 reference 가 없을 가능성. "
                  f"build_integrated_vehicle.py 를 다시 실행하거나 런타임 부착 필요.")
    print(f"  wrist camera → {wrist_cam_path}")
    print(f"  wrist cam prim valid? {stage.GetPrimAtPath(wrist_cam_path).IsValid()}")

    # nav cam translate 만 optimal 로 override (orient 는 vehicle_v2 원본 유지)
    cam_prim_obj = stage.GetPrimAtPath(cam_path)
    cam_xf = UsdGeom.Xformable(cam_prim_obj)
    OPTIMAL_T = Gf.Vec3d(0.3, -0.0, -0.1)
    translate_op = None
    for op in cam_xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break
    if translate_op is not None:
        translate_op.Set(OPTIMAL_T)
        print(f"  camera translate override → ({OPTIMAL_T[0]}, {OPTIMAL_T[1]}, {OPTIMAL_T[2]})")
    else:
        # 기존 translate op 없으면 새로 추가하지만 다른 op (orient 등) 는 그대로
        existing_ops = cam_xf.GetOrderedXformOps()
        new_t = cam_xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        new_t.Set(OPTIMAL_T)
        # translate 가 가장 먼저 적용되도록 op order 재구성
        cam_xf.SetXformOpOrder([new_t] + list(existing_ops))
        print(f"  camera translate added → ({OPTIMAL_T[0]}, {OPTIMAL_T[1]}, {OPTIMAL_T[2]})")

    # Top-down free camera (메인 viewport 용) — USD 카메라 default = -Z 향함 → 회전 X
    top_cam_path = "/World/_TopCam"
    UsdGeom.Camera.Define(stage, top_cam_path)
    top_cam = stage.GetPrimAtPath(top_cam_path)
    txf = UsdGeom.Xformable(top_cam)
    txf.ClearXformOpOrder()
    txf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0, 0.0, 40.0))
    top_cam.CreateAttribute("focalLength", Sdf.ValueTypeNames.Float).Set(15.0)
    top_cam.CreateAttribute("clippingRange", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(0.1, 1000.0))
    print(f"  top camera   → {top_cam_path}")

    # Manipulation 경로 (m0609 articulation root + link_6 + angle_bracket + finger + RearBasket)
    m0609_path     = f"{veh_prim_path}/{M0609_REL}"
    link6_path     = f"{veh_prim_path}/{EE_LINK_REL}"
    ab_path        = f"{veh_prim_path}/{ANGLE_BRACKET_REL}"
    left_finger    = f"{veh_prim_path}/{LEFT_FINGER_REL}"
    right_finger   = f"{veh_prim_path}/{RIGHT_FINGER_REL}"
    basket_path    = f"{veh_prim_path}/{REAR_BASKET_REL}"
    rover_subtree  = f"{veh_prim_path}/Vehicle/rover"
    rover_body     = f"{veh_prim_path}/Vehicle/rover/Body"
    # m0609 articulation root 는 base_link
    m0609_root_path = f"{m0609_path}/base_link"
    print(f"  m0609 root   → {m0609_root_path}")
    print(f"  ee link_6    → {link6_path}")
    print(f"  angle bracket→ {ab_path}")
    print(f"  rear basket  → {basket_path}")
    print(f"  rover body   → {rover_body}")

    # ── Friction grasp 용 PhysicsMaterial — finger 양쪽에 강한 마찰력 부여 ──
    _finger_mat = _create_physics_material(
        stage, "/World/PhysicsMaterials/finger_mat",
        sf=FINGER_FRICTION[0], df=FINGER_FRICTION[1], rest=FINGER_FRICTION[2])
    _n_fl = _bind_physics_material_to_subtree(stage, left_finger, _finger_mat)
    _n_fr = _bind_physics_material_to_subtree(stage, right_finger, _finger_mat)
    print(f"  [friction] finger PhysicsMaterial sf={FINGER_FRICTION[0]} "
          f"df={FINGER_FRICTION[1]} bound to L:{_n_fl} R:{_n_fr} Mesh prims")

    paths = {
        "nav_cam":       cam_path,
        "wrist_cam":     wrist_cam_path,
        "top_cam":       top_cam_path,
        "veh_root":      veh_prim_path,
        "m0609_root":    m0609_root_path,
        "link6":         link6_path,
        "angle_bracket": ab_path,
        "left_finger":   left_finger,
        "right_finger":  right_finger,
        "basket":        basket_path,
        "rover_subtree": rover_subtree,
        "rover_body":    rover_body,
        "minerals":      mineral_paths,
    }
    return paths, t_op, r_op


def load_yolo(model_path: str):
    try:
        from ultralytics import YOLO
    except ImportError:
        print("[ERROR] ultralytics 미설치")
        sys.exit(1)
    if not Path(model_path).exists():
        print(f"[ERROR] model not found: {model_path}")
        sys.exit(1)
    print(f"[yolo] loading {model_path}")
    m = YOLO(model_path)
    print(f"[yolo] classes = {m.names}")
    return m


def _sample_depth(depth, cx: int, cy: int, half: int = 3) -> float:
    """bbox 중심점 주변 (2*half+1)x(2*half+1) 영역의 median depth (m).
    노이즈/sky 픽셀 영향을 줄이려 median 사용."""
    if depth is None or depth.size == 0:
        return float("nan")
    H, W = depth.shape[:2]
    x0 = max(0, cx - half); x1 = min(W, cx + half + 1)
    y0 = max(0, cy - half); y1 = min(H, cy + half + 1)
    patch = depth[y0:y1, x0:x1]
    if patch.size == 0:
        return float("nan")
    # inf / NaN 제거
    valid = patch[np.isfinite(patch) & (patch > 0)]
    if valid.size == 0:
        return float("nan")
    return float(np.median(valid))


def annotate(bgr, results, conf_thr: float, depth=None) -> tuple:
    """ultralytics result → bbox + label 그린 BGR 이미지.
    return (annotated_bgr, list of dict(name, conf, dist, cx, cy))"""
    out = bgr.copy()
    det_summary = []
    if results is None or len(results) == 0:
        return out, det_summary
    r = results[0]
    names = r.names
    palette = {
        0: (255, 200, 0),     # blue_mineral
        1: (0, 255, 255),     # yellow_mineral
        2: (0, 255, 0),       # green_gas
    }
    for box in r.boxes:
        conf = float(box.conf)
        if conf < conf_thr:
            continue
        cls = int(box.cls)
        x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
        cx_p = (x1 + x2) // 2
        cy_p = (y1 + y2) // 2
        dist = _sample_depth(depth, cx_p, cy_p)
        color = palette.get(cls, (255, 255, 255))
        cls_name = names[cls]
        if np.isfinite(dist):
            label = f"{cls_name} {conf:.2f}  {dist:.2f}m"
        else:
            label = f"{cls_name} {conf:.2f}"
        det_summary.append({"name": cls_name, "conf": conf, "dist": dist,
                            "cx": cx_p, "cy": cy_p})

        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(out, (x1, max(0, y1 - th - 6)), (x1 + tw + 4, y1), color, -1)
        cv2.putText(out, label, (x1 + 2, max(th, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)
        # bbox 중심에 십자 마크 (depth 샘플링 지점 표시)
        cv2.drawMarker(out, (cx_p, cy_p), color, cv2.MARKER_CROSS, 12, 2)
    return out, det_summary


# ════════════════════════════════════════════════════════════════════════
#  Manipulation 헬퍼 (pickplace_visual_rover.py 에서 인라인 이식)
# ════════════════════════════════════════════════════════════════════════
def _find_prim_path_by_name(root_path, link_name):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == link_name:
            return str(prim.GetPath())
    return None


def _read_world_xyz(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = m.ExtractTranslation()
    return (float(t[0]), float(t[1]), float(t[2]))


def _read_world_pose_mat(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None, None
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = m.ExtractTranslation()
    pos = np.array([t[0], t[1], t[2]], dtype=np.float64)
    rotmat = np.array([
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ], dtype=np.float64)
    return pos, rotmat


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
    err = np.concatenate([pos_err, ori_weight * rot_err])
    err_report = np.concatenate([pos_err, rot_err])

    J_all, col_offset = _resolve_jacobian(art, ee_body_index)
    if J_all is None:
        return None, err_report
    J_body = J_all[ee_body_index]
    arm_cols = [col_offset + i for i in joint_indices]
    J_arm = J_body[:, arm_cols]
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
    if len(joint_indices) == 6:
        for k, j in enumerate(joint_indices):
            lo_deg, hi_deg = IK_JOINT_LIMITS_DEG[k]
            new_q_full[j] = float(np.clip(new_q_full[j],
                                          np.deg2rad(lo_deg), np.deg2rad(hi_deg)))
    return new_q_full, err_report


def _drive_joints_rad(art, idx, joint_rad):
    """Kinematic-safe joint set — articulation 의 root 가 kinematic 일 때도 작동.
    현재 전체 joint pos 를 읽어 indexed joint 만 갱신 후 set_joint_positions 로 직접 적용.
    drive PD 를 거치지 않으므로 즉시 반영 (한 IK step 의 alpha*dq 가 그대로 적용).
    """
    cur = np.array(art.get_joint_positions(), dtype=np.float32)
    for k, j in enumerate(idx):
        cur[j] = float(joint_rad[k])
    try:
        art.set_joint_positions(cur)
    except Exception:
        # fallback: drive PD (root 가 dynamic 인 경우)
        art.get_articulation_controller().apply_action(ArticulationAction(
            joint_positions=np.array(joint_rad, dtype=np.float32),
            joint_indices=np.array(idx, dtype=np.int32),
        ))


def _get_body_names(art):
    if hasattr(art, "body_names"):
        try:
            v = art.body_names
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


def _attach_object_to_link(stage, joint_path, link_path, obj_path):
    """obj 와 link 의 현재 상대 pose 그대로 FixedJoint 부착 (teleport 없음)."""
    if stage.GetPrimAtPath(joint_path).IsValid():
        stage.RemovePrim(joint_path)
    link_prim = stage.GetPrimAtPath(link_path)
    obj_prim = stage.GetPrimAtPath(obj_path)
    if not link_prim.IsValid() or not obj_prim.IsValid():
        return False
    link_xf = UsdGeom.Xformable(link_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    obj_xf = UsdGeom.Xformable(obj_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    rel = obj_xf * link_xf.GetInverse()
    rel_pos = rel.ExtractTranslation()
    rel_rot = rel.ExtractRotationQuat()
    rot_imag = rel_rot.GetImaginary()
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(obj_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(rel_pos))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(
        rel_rot.GetReal(),
        float(rot_imag[0]), float(rot_imag[1]), float(rot_imag[2]),
    ))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    print(f"[grip] attached {obj_path} @ captured pose")
    return True


def _detach_grip_joint(stage, joint_path):
    if stage.GetPrimAtPath(joint_path).IsValid():
        stage.RemovePrim(joint_path)
        print("[grip] detached")


def _hide_prim(stage, prim_path):
    """prim 의 visibility 를 invisible 로 설정 — 시각적으로만 사라짐 (prim 자체는 stage 에 남음)."""
    if not prim_path:
        return False
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return False
    try:
        UsdGeom.Imageable(prim).MakeInvisible()
        return True
    except Exception as e:
        print(f"[visibility] hide 실패 {prim_path}: {e.__class__.__name__}: {e}")
        return False


def _deproject_pixel_to_world(px, py, depth, intrinsics,
                              cam_world_pos, cam_world_rotmat):
    """픽셀 (px, py) + depth → world XYZ.

    USD/OpenGL camera convention: camera 가 -Z 방향 봄, Y 는 위.
    OpenCV pixel convention: (px, py) origin top-left, py 증가 = 아래.
    """
    fx, fy, cx, cy = intrinsics
    X_cam = (px - cx) * depth / fx
    Y_cam = -(py - cy) * depth / fy
    Z_cam = -depth
    p_cam = np.array([X_cam, Y_cam, Z_cam], dtype=np.float64)
    p_world = cam_world_rotmat @ p_cam + cam_world_pos
    return p_world


def _freeze_rover_drives(rover_prim):
    """rover subtree 의 모든 revolute/prismatic joint 에 stiff drive lock 적용.
    기존 drive 가 있으면 강화, 없으면 추가.

    terrain_00022.usd 에 ground mesh collision 없음 → passive rocker/bogie joint 가
    중력으로 droop 함. 모든 joint 에 stiffness 1e7 적용해 suspension 자세 고정."""
    def _safe_set(attr_get, attr_create, value):
        try:
            attr = attr_get()
            if attr and attr.IsValid():
                attr.Set(value)
                return
            attr = attr_create()
            attr.Set(value)
        except Exception:
            pass

    n = 0
    for prim in Usd.PrimRange(rover_prim):
        type_name = prim.GetTypeName()
        if type_name not in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint"):
            continue
        drv_type = "angular" if "Revolute" in type_name else "linear"
        drv = UsdPhysics.DriveAPI.Get(prim, drv_type)
        if not drv:
            try:
                drv = UsdPhysics.DriveAPI.Apply(prim, drv_type)
            except Exception:
                continue
        _safe_set(drv.GetTargetPositionAttr, drv.CreateTargetPositionAttr, 0.0)
        _safe_set(drv.GetTargetVelocityAttr, drv.CreateTargetVelocityAttr, 0.0)
        _safe_set(drv.GetStiffnessAttr,      drv.CreateStiffnessAttr,      1e7)
        _safe_set(drv.GetDampingAttr,        drv.CreateDampingAttr,        1e6)
        _safe_set(drv.GetMaxForceAttr,       drv.CreateMaxForceAttr,       1e8)
        n += 1
    return n


def _get_camera_intrinsics(camera_obj, resolution):
    w, h = resolution
    try:
        if hasattr(camera_obj, "get_intrinsics_matrix"):
            K = camera_obj.get_intrinsics_matrix()
            if K is not None:
                return float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
    except Exception:
        pass
    try:
        fl = float(camera_obj.get_focal_length())
        hAp = float(camera_obj.get_horizontal_aperture())
        vAp = float(camera_obj.get_vertical_aperture())
    except Exception:
        fl, hAp, vAp = 18.147, 20.955, 15.290
    fx = fl * w / hAp
    fy = fl * h / vAp
    cx = w / 2.0
    cy = h / 2.0
    return fx, fy, cx, cy


# ════════════════════════════════════════════════════════════════════════
#  Pick & Place State Machine
# ════════════════════════════════════════════════════════════════════════
class PickPlaceStateMachine:
    """Autopilot 정지 후 활성화. mineral world XYZ 와 basket world XYZ 를 받아
    APPROACH → DESCEND → GRASP → LIFT → MOVE_TO_BASKET → PLACE → RELEASE →
    RETREAT → HOME → DONE 순서로 진행.
    """
    STATES = [
        "IDLE", "HOME_PRE", "WRIST_SERVO", "APPROACH_DESCEND",
        "GRASP_CLOSE", "ATTACH_LIFT", "JS_PRE", "RELEASE", "JS_POST", "DONE",
    ]

    def __init__(self, art, joint_idx, gripper_joint_idx, link6_path,
                 ee_body_index, q_home, quat_lock, angle_bracket_path,
                 basket_path, stage,
                 wrist_cam_path=None, wrist_camera_obj=None, wrist_res_wh=None,
                 tcp_offset_local=None):
        self.art = art
        self.joint_idx = joint_idx
        self.gripper_joint_idx = gripper_joint_idx
        self.link6_path = link6_path
        self.ee_body_index = ee_body_index
        self.q_home = q_home
        self.quat_lock = quat_lock
        self.angle_bracket_path = angle_bracket_path
        self.basket_path = basket_path
        self.stage = stage

        # Wrist cam visual servoing
        self.wrist_cam_path = wrist_cam_path
        self.wrist_camera_obj = wrist_camera_obj
        self.wrist_res_wh = wrist_res_wh
        self.wrist_summary = []          # 외부 (메인 루프) 에서 매 inference 마다 갱신
        self.wrist_intrinsics = None     # lazy 계산
        self.mineral_class_hint = None   # 클래스 매칭 우선순위

        # TCP (finger midpoint) offset in link_6 local frame — IK target 보정용
        # 런타임 HOME 자세 캡쳐 후 외부에서 계산해 넘겨받음 (None 이면 보정 안 함)
        self.tcp_offset_local = (None if tcp_offset_local is None
                                 else np.asarray(tcp_offset_local, dtype=np.float64))

        self.state = "IDLE"
        self.step_count = 0
        self.gripper_close_counter = 0
        self.mineral_xyz = None
        self.mineral_path = None
        self.lift_z = None
        # Joint-space waypoint 진행 인덱스 (PRE 또는 POST trajectory 내에서)
        self.js_wp_idx = 0

    @property
    def busy(self):
        return self.state not in ("IDLE", "DONE")

    def start_pick(self, mineral_xyz, mineral_path=None, class_hint=None):
        self.mineral_xyz = np.asarray(mineral_xyz, dtype=np.float64)
        self.mineral_path = mineral_path  # None 이면 animation only
        self.mineral_class_hint = class_hint
        self.lift_z = self.mineral_xyz[2] + LIFT_HEIGHT
        self.state = "HOME_PRE"
        self.step_count = 0
        self.gripper_close_counter = 0
        print(f"\n[manip] START pick — mineral={self.mineral_xyz.round(3)} "
              f"class={class_hint or '?'} path={mineral_path or 'animation-only'}")

    def _refresh_from_wrist(self):
        """wrist cam YOLO bbox 중심 + depth 로 mineral_xyz 의 XY 갱신.
        매칭 실패 / 카메라 미설정 시 False 반환 (기존 XYZ 유지)."""
        if not self.wrist_summary or self.wrist_cam_path is None:
            return False
        cands = self.wrist_summary
        # 클래스 매칭 우선
        if self.mineral_class_hint:
            same = [d for d in cands if d.get("name") == self.mineral_class_hint]
            if same:
                cands = same
        valid = [d for d in cands
                 if np.isfinite(d.get("dist", float("nan"))) and d["dist"] > 0.03]
        if not valid:
            return False
        # 가장 가까운 것을 target 으로
        target = min(valid, key=lambda d: d["dist"])
        if self.wrist_intrinsics is None and self.wrist_camera_obj is not None:
            try:
                self.wrist_intrinsics = _get_camera_intrinsics(
                    self.wrist_camera_obj, self.wrist_res_wh)
            except Exception:
                return False
        if self.wrist_intrinsics is None:
            return False
        cam_pos, cam_rot = _read_world_pose_mat(self.wrist_cam_path)
        if cam_pos is None:
            return False
        new_xyz = _deproject_pixel_to_world(
            int(target["cx"]), int(target["cy"]), float(target["dist"]),
            self.wrist_intrinsics, cam_pos, cam_rot)
        old_xy = self.mineral_xyz[:2].copy()
        # XY 만 갱신 — Z 는 nav cam / ground-truth 값 유지 (wrist depth noise 큼)
        self.mineral_xyz[0] = float(new_xyz[0])
        self.mineral_xyz[1] = float(new_xyz[1])
        self.lift_z = self.mineral_xyz[2] + LIFT_HEIGHT
        print(f"[manip] wrist refresh: XY {old_xy.round(3)} → "
              f"{self.mineral_xyz[:2].round(3)} "
              f"(class={target.get('name')}, conf={target.get('conf', 0):.2f}, "
              f"wrist_dist={target['dist']:.2f}m)")
        return True

    def abort(self):
        if self.state != "IDLE":
            print(f"[manip] ABORT from {self.state}")
            if self.mineral_path is not None:
                _detach_grip_joint(self.stage, GRIP_JOINT_PATH)
            self.state = "IDLE"
            self.step_count = 0

    def _ik_to(self, target_tcp_world):
        """target_tcp_world 를 TCP(finger midpoint) 가 도달해야 할 world XYZ 로 해석.
        TCP_OFFSET_LOCAL 가 설정돼 있으면 link_6 의 실제 IK target = TCP target - rot*offset."""
        if self.tcp_offset_local is not None:
            _, link6_rot = _read_world_pose_mat(self.link6_path)
            if link6_rot is not None:
                tcp_offset_world = link6_rot @ self.tcp_offset_local
                target_link6 = np.asarray(target_tcp_world, dtype=np.float64) - tcp_offset_world
            else:
                target_link6 = target_tcp_world
        else:
            target_link6 = target_tcp_world
        new_q, err6 = _ik_dls_step(
            self.art, self.link6_path, self.joint_idx,
            target_link6, self.quat_lock, self.ee_body_index,
            q_rest=self.q_home,
            per_joint_null_gain=IK_NULL_GAIN_PER_JOINT,
        )
        if new_q is not None:
            _drive_joints_rad(self.art, self.joint_idx, new_q[self.joint_idx])
        return float(np.linalg.norm(err6[:3]))

    def _drive_gripper(self, target):
        _drive_joints_rad(self.art, self.gripper_joint_idx, target)

    def _drive_to_waypoint_deg(self, target_deg, interp_frames=WAYPOINT_INTERP_FRAMES):
        """6-DOF arm joint 을 target_deg 까지 `interp_frames` frame 에 걸쳐 선형 보간 이동.
        snap 대신 점진적 변화 → 시각적으로 동작이 보임.
        반환: 현재 joint pose 와 FINAL target 의 최대 각도 오차 (deg)."""
        target_arr = np.asarray(target_deg, dtype=np.float64)
        # waypoint 진입 첫 step (step_count == 1) 에 시작 자세 캡쳐
        if self.step_count == 1 or not hasattr(self, "_wp_start_deg"):
            cur = np.array(self.art.get_joint_positions(), dtype=np.float64)
            self._wp_start_deg = np.rad2deg(cur[self.joint_idx])
        # 선형 보간: t ∈ [0, 1]
        t = min(1.0, self.step_count / float(interp_frames))
        interp_target = self._wp_start_deg + t * (target_arr - self._wp_start_deg)
        _drive_joints_rad(self.art, self.joint_idx, np.deg2rad(interp_target))
        # 오차는 FINAL target 기준 — 보간 끝나면 자동으로 작아짐
        cur = np.array(self.art.get_joint_positions(), dtype=np.float64)
        cur_arm_deg = np.rad2deg(cur[self.joint_idx])
        return float(np.max(np.abs(cur_arm_deg - target_arr)))

    def step(self):
        s = self.state
        self.step_count += 1

        if s == "IDLE" or s == "DONE":
            return

        if s == "HOME_PRE":
            _drive_joints_rad(self.art, self.joint_idx, self.q_home[self.joint_idx])
            self._drive_gripper(GRIPPER_OPEN_RAD)
            cur = np.array(self.art.get_joint_positions(), dtype=np.float64)
            err = np.max(np.abs(cur[self.joint_idx] - self.q_home[self.joint_idx]))
            if err < np.deg2rad(HOME_REACHED_JOINT_TOL_DEG) or self.step_count > 200:
                print(f"[manip] HOME_PRE → WRIST_SERVO")
                self.state = "WRIST_SERVO"
                self.step_count = 0

        elif s == "WRIST_SERVO":
            # HOME 자세 유지하면서 wrist cam YOLO 로 mineral XY 보정 시도.
            # 30 frame 안에 검출되면 즉시 진행, 못 잡으면 nav XYZ 그대로 진행.
            _drive_joints_rad(self.art, self.joint_idx, self.q_home[self.joint_idx])
            self._drive_gripper(GRIPPER_OPEN_RAD)
            # green_gas: wrist refresh 결과가 visible mesh 시각 중심과 부정확 → terrain GT XY 그대로 사용
            if self.mineral_class_hint == "green_gas":
                print(f"[manip] WRIST_SERVO → APPROACH_DESCEND (green_gas — wrist refresh skip, terrain GT XY 사용)")
                self.state = "APPROACH_DESCEND"
                self.step_count = 0
            elif self._refresh_from_wrist():
                print(f"[manip] WRIST_SERVO → APPROACH_DESCEND (wrist refreshed)")
                self.state = "APPROACH_DESCEND"
                self.step_count = 0
            elif self.step_count > 30:
                print(f"[manip] WRIST_SERVO → APPROACH_DESCEND "
                      f"(wrist det 없음 — nav XYZ 사용)")
                self.state = "APPROACH_DESCEND"
                self.step_count = 0

        elif s == "APPROACH_DESCEND":
            # TCP(finger midpoint) 가 mineral 위 HOVER_ABOVE_MINERAL 에 오도록 IK — class 별 lookup
            hover_h = HOVER_ABOVE_MINERAL_PER_CLASS.get(
                self.mineral_class_hint, HOVER_ABOVE_MINERAL)
            target = np.array([self.mineral_xyz[0], self.mineral_xyz[1],
                               self.mineral_xyz[2] + hover_h])
            pos_err = self._ik_to(target)
            self._drive_gripper(GRIPPER_OPEN_RAD)
            if pos_err < IK_POS_TOL:
                print(f"[manip] APPROACH_DESCEND → GRASP_CLOSE  "
                      f"pos_err={pos_err*1000:.0f}mm OK")
                self.state = "GRASP_CLOSE"
                self.step_count = 0
                self.gripper_close_counter = 0
            elif self.step_count > IK_MAX_STEPS_PER_PHASE * 2:
                if pos_err < IK_GRASP_REACH_THRESHOLD:
                    print(f"[manip] APPROACH_DESCEND → GRASP_CLOSE  "
                          f"pos_err={pos_err*1000:.0f}mm (marginal)")
                    self.state = "GRASP_CLOSE"
                    self.step_count = 0
                    self.gripper_close_counter = 0
                else:
                    print(f"[manip] ABORT — APPROACH_DESCEND timed out at "
                          f"{pos_err*1000:.0f}mm")
                    self.state = "DONE"
                    self.step_count = 0

        elif s == "GRASP_CLOSE":
            # 진단: finger midpoint world XYZ 와 cube center 의 차이 (첫 진입 step 에만)
            if self.gripper_close_counter == 0:
                try:
                    import os.path as _osp
                    _parent = _osp.dirname(self.angle_bracket_path)
                    _left = _read_world_xyz(f"{_parent}/left_inner_finger")
                    _right = _read_world_xyz(f"{_parent}/right_inner_finger")
                    if _left is not None and _right is not None:
                        _mid = ((_left[0]+_right[0])/2.0,
                                (_left[1]+_right[1])/2.0,
                                (_left[2]+_right[2])/2.0)
                        _cx, _cy, _cz = (float(self.mineral_xyz[0]),
                                         float(self.mineral_xyz[1]),
                                         float(self.mineral_xyz[2]))
                        _dx, _dy, _dz = (_mid[0]-_cx, _mid[1]-_cy, _mid[2]-_cz)
                        print(f"  [grasp diag] finger mid @ ({_mid[0]:+.3f},{_mid[1]:+.3f},{_mid[2]:+.3f})")
                        print(f"  [grasp diag] cube center @ ({_cx:+.3f},{_cy:+.3f},{_cz:+.3f})")
                        print(f"  [grasp diag] Δ (finger - cube) = "
                              f"({_dx*1000:+.0f},{_dy*1000:+.0f},{_dz*1000:+.0f}) mm")
                except Exception as _e:
                    print(f"  [grasp diag] failed: {_e}")
            self._drive_gripper(GRIPPER_CLOSED_RAD)
            self.gripper_close_counter += 1
            if self.gripper_close_counter > GRIPPER_CLOSE_SETTLE_FRAMES:
                # B 롤백: M0609 down-reach 한계로 friction grasp 불가능 → FixedJoint magic grasp 복귀.
                if self.mineral_path is not None:
                    _attach_object_to_link(self.stage, GRIP_JOINT_PATH,
                                           self.angle_bracket_path, self.mineral_path)
                else:
                    print(f"[grip] (mineral_path=None — animation only, no FixedJoint)")
                print(f"[manip] GRASP_CLOSE → ATTACH_LIFT")
                self.state = "ATTACH_LIFT"
                self.step_count = 0

        elif s == "ATTACH_LIFT":
            target = np.array([self.mineral_xyz[0], self.mineral_xyz[1], self.lift_z])
            pos_err = self._ik_to(target)
            self._drive_gripper(GRIPPER_CLOSED_RAD)
            if pos_err < IK_POS_TOL or self.step_count > IK_MAX_STEPS_PER_PHASE:
                print(f"[manip] ATTACH_LIFT → JS_PRE (joint-space dump trajectory)  "
                      f"pos_err={pos_err*1000:.0f}mm")
                self.state = "JS_PRE"
                self.step_count = 0
                self.js_wp_idx = 0

        elif s == "JS_PRE":
            # PLACE_TRAJ_PRE_DEG 의 waypoint 들을 순차 진행 — gripper 는 닫은 채
            if self.js_wp_idx >= len(PLACE_TRAJ_PRE_DEG):
                print(f"[manip] JS_PRE → RELEASE  (all {len(PLACE_TRAJ_PRE_DEG)} waypoints done)")
                self.state = "RELEASE"
                self.step_count = 0
                return
            wp_deg = PLACE_TRAJ_PRE_DEG[self.js_wp_idx]
            err_deg = self._drive_to_waypoint_deg(wp_deg)
            self._drive_gripper(GRIPPER_CLOSED_RAD)
            if err_deg < WAYPOINT_TOL_DEG or self.step_count > WAYPOINT_TIMEOUT:
                qual = "OK" if err_deg < WAYPOINT_TOL_DEG else "timeout"
                print(f"[manip] JS_PRE wp{self.js_wp_idx} {wp_deg} done "
                      f"(err={err_deg:.1f}°, {qual})")
                self.js_wp_idx += 1
                self.step_count = 0

        elif s == "RELEASE":
            # B 롤백: FixedJoint 제거 + gripper 열기. 자세는 마지막 PRE waypoint 유지.
            if self.step_count == 1 and self.mineral_path is not None:
                _detach_grip_joint(self.stage, GRIP_JOINT_PATH)
            self._drive_gripper(GRIPPER_OPEN_RAD)
            self._drive_to_waypoint_deg(PLACE_TRAJ_PRE_DEG[-1])
            # mineral 시각적으로 숨김 (basket 수납 표현)
            if self.step_count == 6 and self.mineral_path is not None:
                if _hide_prim(self.stage, self.mineral_path):
                    print(f"[manip] mineral 수납 완료 — visibility=invisible ({self.mineral_path})")
            if self.step_count > 30:
                print(f"[manip] RELEASE → JS_POST")
                self.state = "JS_POST"
                self.step_count = 0
                self.js_wp_idx = 0

        elif s == "JS_POST":
            # PLACE_TRAJ_POST_DEG 의 waypoint 들 순차 진행 — gripper 열린 채로
            if self.js_wp_idx >= len(PLACE_TRAJ_POST_DEG):
                print(f"[manip] JS_POST → DONE  (all {len(PLACE_TRAJ_POST_DEG)} waypoints done)\n")
                self.state = "DONE"
                self.step_count = 0
                return
            wp_deg = PLACE_TRAJ_POST_DEG[self.js_wp_idx]
            err_deg = self._drive_to_waypoint_deg(wp_deg)
            self._drive_gripper(GRIPPER_OPEN_RAD)
            if err_deg < WAYPOINT_TOL_DEG or self.step_count > WAYPOINT_TIMEOUT:
                qual = "OK" if err_deg < WAYPOINT_TOL_DEG else "timeout"
                print(f"[manip] JS_POST wp{self.js_wp_idx} {wp_deg} done "
                      f"(err={err_deg:.1f}°, {qual})")
                self.js_wp_idx += 1
                self.step_count = 0


def main():
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    res_w, res_h = (int(v) for v in args.resolution.split("x"))

    yolo = load_yolo(args.model)

    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    paths, t_op, r_op = build_scene(stage)
    cam_path       = paths["nav_cam"]
    wrist_cam_path = paths["wrist_cam"]
    top_cam_path   = paths["top_cam"]

    # ── 초기 rover 스폰 위치 (x=5, y=0 — 0,0 이 아닌 +X 5m 지점) ──
    SPAWN_X, SPAWN_Y = 5.0, 0.0

    # ── t_op 을 ground 수준 + spawn xy 로 미리 맞춤 (anchor target 과 mismatch 방지) ──
    import json as _json_pre
    _TERR_DIR_PRE = Path("/home/rokey/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/generated_terrains/terrain_00022")
    try:
        _hm_pre = np.load(_TERR_DIR_PRE / "heightmap.npy")
        _meta_pre = _json_pre.loads((_TERR_DIR_PRE / "meta.json").read_text())
        _res_pre = float(_meta_pre["resolution_m"])
        _ox_pre = float(_meta_pre["origin"]["x"])
        _oy_pre = float(_meta_pre["origin"]["y"])
        # ground_z at (SPAWN_X, SPAWN_Y)
        _col = (SPAWN_X - _ox_pre) / _res_pre
        _row = (SPAWN_Y - _oy_pre) / _res_pre
        _c0 = int(_col); _r0 = int(_row)
        _gz0 = float(_hm_pre[_r0, _c0])
        _init_outer_z = _gz0 + 0.35  # GROUND_CLEARANCE
        t_op.Set(Gf.Vec3d(SPAWN_X, SPAWN_Y, _init_outer_z))
        print(f"  [pre-reset] t_op override → ({SPAWN_X:.2f},{SPAWN_Y:.2f},{_init_outer_z:.3f}) "
              f"(spawn @ x=5 ground+clearance)")
    except Exception as _e:
        print(f"  [pre-reset] heightmap load 실패 ({_e.__class__.__name__}) — t_op 그대로")

    # 메인 viewport = 탑뷰 / 별도 omni.ui 윈도 = vehicle 카메라 + bbox overlay
    try:
        import omni.kit.viewport.utility as vp_util
        vp_main = vp_util.get_active_viewport()
        # 여러 API path 시도 (Isaac Sim 버전마다 다를 수 있음)
        try:
            vp_main.camera_path = top_cam_path
        except Exception:
            pass
        try:
            vp_main.set_active_camera(top_cam_path)
        except Exception:
            pass
        print(f"\n[viewport] main → {top_cam_path} (top view, z=40)")
        print(f"           실제 active: {getattr(vp_main, 'camera_path', '?')}")
    except Exception as e:
        print(f"\n[WARN] main viewport 설정 실패: {e}")

    # omni.ui 윈도: nav cam + bbox 오버레이
    import omni.ui as ui
    yolo_window = ui.Window("YOLO — Nav Cam (rover body)",
                            width=720, height=420,
                            position_x=20, position_y=40,
                            dockPreference=ui.DockPreference.DISABLED)
    img_provider = ui.ByteImageProvider()
    with yolo_window.frame:
        with ui.VStack():
            ui.Label("nav cam (rover body, forward view)",
                     height=20, alignment=ui.Alignment.CENTER)
            ui.ImageWithProvider(img_provider,
                                 fill_policy=ui.IwpFillPolicy.IWP_PRESERVE_ASPECT_FIT)

    # 2nd 윈도: wrist cam (D455 RGB) + bbox 오버레이
    yolo_wrist_window = ui.Window("YOLO — Wrist Cam (D455 RGB)",
                                   width=720, height=420,
                                   position_x=760, position_y=40,
                                   dockPreference=ui.DockPreference.DISABLED)
    img_provider_wrist = ui.ByteImageProvider()
    with yolo_wrist_window.frame:
        with ui.VStack():
            ui.Label("wrist cam (D455 color, top-down)",
                     height=20, alignment=ui.Alignment.CENTER)
            ui.ImageWithProvider(img_provider_wrist,
                                 fill_policy=ui.IwpFillPolicy.IWP_PRESERVE_ASPECT_FIT)
    print(f"[ui] YOLO viewer windows: nav + wrist")

    camera = Camera(prim_path=cam_path, resolution=(res_w, res_h), frequency=30)
    wrist_camera = None
    try:
        wrist_camera = Camera(prim_path=wrist_cam_path, resolution=(res_w, res_h), frequency=30)
    except Exception as e:
        print(f"[ERROR] wrist_camera 생성 실패 ({e.__class__.__name__}: {e}) — "
              f"wrist UI 는 빈 화면으로 유지됨")

    # M0609 articulation 등록 (모든 body dynamic → root 도 dynamic → 정상 init).
    print(f"\n[articulation] register m0609 root = {paths['m0609_root']}")
    art = SingleArticulation(prim_path=paths["m0609_root"], name="m0609_art")
    world.scene.add(art)

    print("[World] reset …")
    world.reset()
    camera.initialize()
    camera.add_distance_to_image_plane_to_frame()         # nav depth
    if wrist_camera is not None:
        try:
            wrist_camera.initialize()
            wrist_camera.add_distance_to_image_plane_to_frame()
            print(f"[wrist_camera] initialized OK @ {wrist_cam_path}")
        except Exception as e:
            print(f"[ERROR] wrist_camera.initialize() 실패 ({e.__class__.__name__}: {e})")
            wrist_camera = None

    # ── Heightmap 으로 초기 ground z 계산 (terrain 자체에 collision 없음 → settle 불가) ──
    import json as _json_init
    TERRAIN_DIR_INIT = Path("/home/rokey/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/generated_terrains/terrain_00022")
    _hm_init = np.load(TERRAIN_DIR_INIT / "heightmap.npy")
    _meta_init = _json_init.loads((TERRAIN_DIR_INIT / "meta.json").read_text())
    _res_init = float(_meta_init["resolution_m"])
    _ox_init = float(_meta_init["origin"]["x"])
    _oy_init = float(_meta_init["origin"]["y"])
    GROUND_CLEARANCE_INIT = 0.35

    def _ground_z_init(x, y, default=0.0):
        Hh, Wh = _hm_init.shape
        col = (x - _ox_init) / _res_init
        row = (y - _oy_init) / _res_init
        if not (0.0 <= col < Wh - 1 and 0.0 <= row < Hh - 1):
            return default
        c0 = int(col); c1 = min(c0 + 1, Wh - 1)
        r0 = int(row); r1 = min(r0 + 1, Hh - 1)
        fc = col - c0; fr = row - r0
        return float((1 - fr) * (1 - fc) * _hm_init[r0, c0] +
                     (1 - fr) *      fc  * _hm_init[r0, c1] +
                          fr  * (1 - fc) * _hm_init[r1, c0] +
                          fr  *      fc  * _hm_init[r1, c1])

    init_xy = (SPAWN_X, SPAWN_Y)  # x=5, y=0 으로 spawn
    init_z  = _ground_z_init(*init_xy) + GROUND_CLEARANCE_INIT
    print(f"  [init pose] target rover Body @ ({init_xy[0]},{init_xy[1]},{init_z:.3f})")

    # ── rover wheel drives freeze (잡기 전에 lock) ──
    rover_subtree_prim = stage.GetPrimAtPath(paths["rover_subtree"])
    if rover_subtree_prim.IsValid():
        n_frozen = _freeze_rover_drives(rover_subtree_prim)
        print(f"  [drives] frozen: {n_frozen}")

    # ── RoverAnchor FixedJoint — world ↔ rover/Body, play 전에 생성해 떨어지지 않게 ──
    stage.SetEditTarget(Usd.EditTarget(stage.GetRootLayer()))
    stage.DefinePrim("/World/Joints", "Scope")
    anchor = UsdPhysics.FixedJoint.Define(stage, "/World/Joints/RoverAnchor")
    anchor.CreateBody1Rel().SetTargets([Sdf.Path(paths["rover_body"])])
    anchor.CreateLocalPos0Attr().Set(Gf.Vec3f(init_xy[0], init_xy[1], init_z))
    anchor.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    anchor.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    anchor.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    anchor.CreateBreakForceAttr().Set(float("inf"))
    anchor.CreateBreakTorqueAttr().Set(float("inf"))
    anchor_pos_attr = anchor.GetLocalPos0Attr()
    anchor_rot_attr = anchor.GetLocalRot0Attr()
    print(f"  [anchor] RoverAnchor FixedJoint applied (no settle — terrain collision 없음)")

    # ── PhysX 시작 + stabilize (anchor snap + articulation init) ──
    if not world.is_playing():
        world.play()
    STABILIZE_FRAMES = 30
    print(f"[Stabilize] {STABILIZE_FRAMES} frames …")
    for k in range(STABILIZE_FRAMES):
        world.step(render=True)
        if k % 10 == 0:
            rxyz = _read_world_xyz(paths["rover_body"])
            print(f"  [stab {k:3d}] rover @ "
                  f"({rxyz[0]:+.3f},{rxyz[1]:+.3f},{rxyz[2]:+.3f})")
    rover_settled = _read_world_xyz(paths["rover_body"]) or (init_xy[0], init_xy[1], init_z)
    print(f"  [stab] rover @ ({rover_settled[0]:+.3f},{rover_settled[1]:+.3f},{rover_settled[2]:+.3f})")

    # ── M0609 joint 인덱스 + EE body index 해상 ────────────────────────
    try:
        arm_joint_idx     = _resolve_joint_indices(art, HOME_JOINT_NAMES)
        gripper_joint_idx = _resolve_joint_indices(art, GRIPPER_JOINTS)
    except Exception as e:
        print(f"[ERROR] joint resolve 실패: {e}")
        arm_joint_idx, gripper_joint_idx = None, None
    body_names = _get_body_names(art)
    ee_body_index = _find_ee_body_index(body_names, "link_6") if body_names else None
    print(f"[articulation] arm_idx={arm_joint_idx.tolist() if arm_joint_idx is not None else None}  "
          f"grip_idx={gripper_joint_idx.tolist() if gripper_joint_idx is not None else None}  "
          f"ee_body_index={ee_body_index}")

    # HOME 자세 적용 + link_6 quat lock 캡쳐 (settle 후 articulation 안정 상태에서)
    home_q_arm = np.deg2rad(HOME_JOINT_POSITIONS_DEG)
    q_home_full = None
    link6_home_quat = None
    if arm_joint_idx is not None and ee_body_index is not None:
        _drive_joints_rad(art, arm_joint_idx, home_q_arm)
        try:
            cur = np.array(art.get_joint_positions(), dtype=np.float32).copy()
            for k, j in enumerate(arm_joint_idx):
                cur[j] = home_q_arm[k]
            art.set_joint_positions(cur)
        except Exception:
            pass
        for _ in range(30):
            world.step(render=True)
        q_home_full = np.array(art.get_joint_positions(), dtype=np.float64)
        _, link6_home_quat = _get_link_world_pose(paths["link6"])
        print(f"[articulation] home applied. link_6 quat lock = "
              f"({link6_home_quat[0]:+.3f},{link6_home_quat[1]:+.3f},"
              f"{link6_home_quat[2]:+.3f},{link6_home_quat[3]:+.3f})")

    # ── TCP(finger midpoint) offset 캡쳐 — HOME 자세에서 link_6 frame 안의 offset ──
    # IK 는 link_6 를 target 에 놓는 게 기본이지만, 실제 grip 은 fingers 사이에서 일어남.
    # link_6 → TCP offset 을 한 번 측정해 IK target 보정에 사용.
    tcp_offset_local = None
    if q_home_full is not None:
        try:
            link6_pos_h, link6_rot_h = _read_world_pose_mat(paths["link6"])
            right_xyz = _read_world_xyz(paths["right_finger"])
            left_xyz  = _read_world_xyz(paths["left_finger"])
            if (link6_pos_h is not None and link6_rot_h is not None
                    and right_xyz is not None and left_xyz is not None):
                tcp_world = (np.array(right_xyz) + np.array(left_xyz)) / 2.0
                offset_world = tcp_world - link6_pos_h
                tcp_offset_local = link6_rot_h.T @ offset_world
                print(f"[TCP calib] link_6 → TCP offset (world @ HOME) = "
                      f"({offset_world[0]*1000:+.1f}, {offset_world[1]*1000:+.1f}, "
                      f"{offset_world[2]*1000:+.1f}) mm")
                print(f"[TCP calib] link_6 → TCP offset (local frame) = "
                      f"({tcp_offset_local[0]*1000:+.1f}, {tcp_offset_local[1]*1000:+.1f}, "
                      f"{tcp_offset_local[2]*1000:+.1f}) mm")
            else:
                print(f"[TCP calib] ⚠ finger/link_6 prim 못 읽음 — TCP 보정 비활성")
        except Exception as e:
            print(f"[TCP calib] 실패 ({e.__class__.__name__}: {e}) — TCP 보정 비활성")

    # State machine (manipulation 가능할 때만 활성화)
    manip_ready = (arm_joint_idx is not None and gripper_joint_idx is not None
                   and ee_body_index is not None and q_home_full is not None)
    if manip_ready:
        sm = PickPlaceStateMachine(
            art=art,
            joint_idx=arm_joint_idx,
            gripper_joint_idx=gripper_joint_idx,
            link6_path=paths["link6"],
            ee_body_index=ee_body_index,
            q_home=q_home_full,
            quat_lock=link6_home_quat,
            angle_bracket_path=paths["angle_bracket"],
            basket_path=paths["basket"],
            stage=stage,
            wrist_cam_path=wrist_cam_path,
            wrist_camera_obj=wrist_camera,
            wrist_res_wh=(res_w, res_h),
            tcp_offset_local=tcp_offset_local,
        )
        print(f"[manip] state machine ready.\n")
    else:
        sm = None
        print(f"[manip] DISABLED — articulation init 실패 (autopilot 만 동작).\n")

    # 키보드 상태
    pressed = set()
    quit_flag = [False]
    snap_flag = [False]
    AUTO_ENABLED = [True]      # T 키로 토글 (autopilot)
    abort_manip_flag = [False] # M 키로 manipulation 강제 abort

    KEY = carb.input.KeyboardInput
    def on_kb(event, *_a, **_k):
        et = event.type
        k = event.input
        if et == carb.input.KeyboardEventType.KEY_PRESS:
            pressed.add(k)
            if k == KEY.ESCAPE:
                quit_flag[0] = True
            elif k == KEY.P:
                snap_flag[0] = True
            elif k == KEY.T:
                AUTO_ENABLED[0] = not AUTO_ENABLED[0]
                print(f"  [autopilot] {'ON' if AUTO_ENABLED[0] else 'OFF'}")
            elif k == KEY.M:
                abort_manip_flag[0] = True
            elif k == KEY.SPACE:
                pressed.discard(KEY.W); pressed.discard(KEY.S)
                pressed.discard(KEY.A); pressed.discard(KEY.D)
        elif et == carb.input.KeyboardEventType.KEY_RELEASE:
            pressed.discard(k)
        return True

    app_window  = omni.appwindow.get_default_app_window()
    input_iface = carb.input.acquire_input_interface()
    sub_id      = input_iface.subscribe_to_keyboard_events(app_window.get_keyboard(), on_kb)

    # world.play() 는 이미 settle 전에 호출됨 — 여기선 보장만
    if not world.is_playing():
        world.play()
    print("\n" + "=" * 60)
    print("  Rover YOLO Demo")
    print("=" * 60)
    print("  W/S    : 전진 / 후진 (수동, autopilot/manipulation override)")
    print("  A/D    : 좌/우 회전 (수동, autopilot/manipulation override)")
    print("  Space  : 정지")
    print("  T      : autopilot 토글 (default ON)")
    print("  M      : manipulation 강제 abort 후 autopilot 복귀")
    print("  P      : 현재 frame + detection 저장")
    print("  ESC    : 종료")
    print()
    print("  [autopilot]    mineral 탐지 시: 화면 중앙 정렬 + 0.6m 까지 접근")
    print("                 미탐지 시: body forward 직진 (yaw=0 이면 월드 +X)")
    print("  [manipulation] 0.6m 도달 시 자동 진입 → pick → joint-space dump → release")
    print("                 ATTACH_LIFT → HOME → joint_1=180° → joint_2=25°+joint_5=55° → RELEASE")
    print(f"  model  : {args.model}")
    print(f"  conf   : {args.conf}")
    print(f"  output : {out_dir}")
    print("=" * 60 + "\n")

    # Isaac Sim 번들 cv2 는 GUI 빌드 X — 디스크에 live 프레임 갱신
    live_path = out_dir / "live.png"
    print(f"  live preview: {live_path}")
    print(f"  → 외부에서: eog {live_path}  (자동 갱신)")
    print()

    # 지면 follower — heightmap 으로 직접 sampling (physx raycast 보다 안정)
    import json as _json
    GROUND_CLEARANCE = 0.35  # 차량 base 위 ground 클리어런스

    TERRAIN_DIR = Path("/home/rokey/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/generated_terrains/terrain_00022")
    _hm = np.load(TERRAIN_DIR / "heightmap.npy")  # (H, W)
    _meta = _json.loads((TERRAIN_DIR / "meta.json").read_text())
    _res = float(_meta["resolution_m"])
    _ox = float(_meta["origin"]["x"])
    _oy = float(_meta["origin"]["y"])
    _Hh, _Wh = _hm.shape
    print(f"  heightmap: {_hm.shape}  origin=({_ox},{_oy})  res={_res}m  z=[{_hm.min():.2f},{_hm.max():.2f}]")

    def ground_z(x: float, y: float, default: float = 0.0) -> float:
        # world (x,y) → heightmap (row, col)
        col = (x - _ox) / _res
        row = (y - _oy) / _res
        if not (0.0 <= col < _Wh - 1 and 0.0 <= row < _Hh - 1):
            return default
        # bilinear
        c0 = int(col); c1 = min(c0 + 1, _Wh - 1)
        r0 = int(row); r1 = min(r0 + 1, _Hh - 1)
        fc = col - c0
        fr = row - r0
        z = ((1 - fr) * (1 - fc) * _hm[r0, c0] +
             (1 - fr) *      fc  * _hm[r0, c1] +
                  fr  * (1 - fc) * _hm[r1, c0] +
                  fr  *      fc  * _hm[r1, c1])
        return float(z)

    # 차량 pose 상태 — settle 된 rover Body 의 world 위치로 시작 (anchor 가 fix 한 곳)
    pos = np.array([rover_settled[0], rover_settled[1], rover_settled[2]], dtype=np.float64)
    yaw = 0.0
    print(f"  initial pos = ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})  (rover settled)")
    snap_idx = 0
    step_i = 0
    last_results = None
    last_fps_t = time.time()
    fps_count  = 0
    fps_disp   = 0.0

    # autopilot 파라미터
    ENGAGE_DISTANCE = 0.9      # m 이내면 forward 축으로 ENGAGE_X_PUSH 만큼 부드럽게 push 후 manipulation 진입
    ENGAGE_X_PUSH   = 0.3      # default — class_hint lookup 실패 시
    ENGAGE_X_PUSH_PER_CLASS = {
        "blue_mineral":   0.5,    # M0609 reach 안에 들어오도록 더 가까이 (이전 0.3 은 reach 한계)
        "yellow_mineral": 0.5,    # M0609 reach 안에 들어오도록 더 가까이
        "green_gas":      0.25,   # green 은 stop 위치 더 멀리 (충돌/푸시 회피)
    }
    ENGAGE_PUSH_FRAMES = 60    # ENGAGE_X_PUSH 를 몇 frame 에 나눠 적용할지 (60fps 기준 60 = 1초)
    AUTO_LIN_SPEED  = 1.0      # autopilot 직진 속도 (engage 전)
    STEER_GAIN = 1.2           # 이미지 중앙 정렬 P-control 게인
    last_target = [None]       # 마지막으로 본 (cx_norm, dist) — 잠시 사라져도 추적 유지
    engage_pushing = [False]   # ENGAGE push 진행 중 flag
    engage_push_class = [None] # push 시작 시 캡쳐된 target class (lookup 키)
    engage_push_target = [None]  # push 시작 시 캡쳐된 target snapshot (det 빠져도 deproject 가능)
    engage_push_frame = [0]    # push 시작 후 경과 frame
    engage_push_yaw = [0.0]    # push 시작 시점 yaw 캡쳐 (push 중에는 steering 정지, 직진)

    # Manipulation 상태 추적 ─────────────────────────────────────────
    picked_set = set()           # 시도한 mineral prim path (또는 cell_key) dedup
    manip_lock_pos = [None]
    manip_lock_yaw = [None]
    nav_cam_intrinsics = [None]

    # Terrain 내장 mineral prim 스캔 — top-level 의 Cube sub-prim 위치 우선 사용.
    # (visible mesh origin 과 collision/grasp 대상인 Cube 위치가 다를 수 있어서)
    # mineral_path 는 그대로 top-level (FixedJoint 부착용), xyz 만 cube 좌표로 대체.
    def _scan_terrain_minerals():
        result = []
        root = stage.GetPrimAtPath("/World/Terrain/Minerals")
        if not root.IsValid():
            return result
        for child in root.GetChildren():
            nm = child.GetName().lower()
            if nm.startswith("blue"):
                color = "blue_mineral"
            elif nm.startswith("yellow"):
                color = "yellow_mineral"
            elif nm.startswith("green") or nm.startswith("red"):
                color = "green_gas"
            else:
                continue
            # Cube sub-prim world XYZ 우선 (visible mesh 와 다른 위치 가능)
            cube_xyz = None
            cube_path = None
            for sub in Usd.PrimRange(child):
                if sub == child:
                    continue
                if sub.GetName().lower() == "cube":
                    cxyz = _read_world_xyz(str(sub.GetPath()))
                    if cxyz is not None:
                        cube_xyz = cxyz
                        cube_path = str(sub.GetPath())
                        break
            top_xyz = _read_world_xyz(str(child.GetPath()))
            # green_gas: visible mesh XY 가 cube XY 와 어긋남 (XY 는 top, Z 는 cube 유지)
            if color == "green_gas" and cube_xyz is not None and top_xyz is not None:
                chosen = (top_xyz[0], top_xyz[1], cube_xyz[2])
                src = "green_gas: top XY + cube Z"
            elif cube_xyz is not None:
                chosen = cube_xyz
                src = "cube"
            elif top_xyz is not None:
                chosen = top_xyz
                src = "top-level (no Cube sub-prim)"
            else:
                continue
            # top ↔ cube offset 진단 (>5mm 면 출력)
            if cube_xyz is not None and top_xyz is not None:
                dx = cube_xyz[0] - top_xyz[0]
                dy = cube_xyz[1] - top_xyz[1]
                dz = cube_xyz[2] - top_xyz[2]
                if max(abs(dx), abs(dy), abs(dz)) > 0.005:
                    print(f"  [cube-offset] {child.GetName():12s} top→cube ΔXYZ = "
                          f"({dx*1000:+.0f},{dy*1000:+.0f},{dz*1000:+.0f}) mm")
            result.append({"path": str(child.GetPath()),
                           "xyz": np.array(chosen, dtype=np.float64),
                           "color": color,
                           "xyz_src": src,
                           "cube_path": cube_path})
        return result

    terrain_minerals = _scan_terrain_minerals()
    print(f"\n[terrain minerals] discovered {len(terrain_minerals)} prims:")
    for m in terrain_minerals:
        print(f"  {m['color']:14s} @ ({m['xyz'][0]:+.2f},{m['xyz'][1]:+.2f},{m['xyz'][2]:+.2f}) "
              f"src={m.get('xyz_src','?')}  {m['path']}")
    print()

    MINERAL_MATCH_RADIUS = 1.5   # m — deproject 결과와 prim 간 거리 허용치

    def _start_manipulation_if_possible(snapshot=None):
        """nav cam YOLO + depth 로 target XYZ 계산 → 가장 가까운 terrain mineral prim 매칭.

        snapshot: push 시작 시점에 캡쳐된 dict {name, cx, cy, dist, cam_pos, cam_rot}.
                  주어지면 현재 det_summary 무시하고 snapshot 으로 deproject (push 끝 시점에 det 가
                  conf drop 으로 빠진 경우에도 manipulation 진입 가능)."""
        if sm is None:
            print(f"  [manip-start FAIL] sm is None")
            return False
        if sm.busy:
            print(f"  [manip-start FAIL] sm busy (state={sm.state})")
            return False
        try:
            nearest = None
            cam_pos, cam_rot = None, None
            if snapshot is not None and snapshot.get('cam_pos') is not None:
                nearest = {
                    'name': snapshot['name'],
                    'cx':   snapshot['cx'],
                    'cy':   snapshot['cy'],
                    'dist': snapshot['dist'],
                }
                cam_pos = snapshot['cam_pos']
                cam_rot = snapshot['cam_rot']
                print(f"  [manip-start] using push snapshot: {nearest['name']} "
                      f"bbox=({nearest['cx']},{nearest['cy']}) dist={nearest['dist']:.2f}m")
            else:
                try:
                    valid = [d for d in det_summary if np.isfinite(d['dist'])]
                except NameError:
                    print(f"  [manip-start FAIL] det_summary NameError")
                    return False
                if not valid:
                    print(f"  [manip-start FAIL] no valid det (with finite dist) and no snapshot")
                    return False
                nearest = min(valid, key=lambda d: d['dist'])
                cam_pos, cam_rot = _read_world_pose_mat(paths["nav_cam"])
                if cam_pos is None:
                    print(f"  [manip-start FAIL] cam_pos None")
                    return False

            if nav_cam_intrinsics[0] is None:
                nav_cam_intrinsics[0] = _get_camera_intrinsics(camera, (res_w, res_h))

            px, py = int(round(nearest['cx'])), int(round(nearest['cy']))
            d = float(nearest['dist'])
            if not (np.isfinite(d) and d > 0.05):
                print(f"  [manip-start FAIL] invalid depth d={d}")
                return False
            target_xyz = _deproject_pixel_to_world(
                px, py, d, nav_cam_intrinsics[0], cam_pos, cam_rot)

            # Terrain mineral prim 역매칭 — XY 평면 거리 기준 (z 는 노이즈 큼)
            mineral_path = None
            mineral_actual_xyz = target_xyz
            closest_info = None
            if terrain_minerals:
                def _xy_dist(m):
                    return float(np.hypot(m['xyz'][0] - target_xyz[0],
                                          m['xyz'][1] - target_xyz[1]))
                closest = min(terrain_minerals, key=_xy_dist)
                dist_xy = _xy_dist(closest)
                closest_info = (closest['path'], closest['color'], dist_xy)
                if dist_xy < MINERAL_MATCH_RADIUS and closest['path'] not in picked_set:
                    mineral_path = closest['path']
                    mineral_actual_xyz = closest['xyz'].copy()  # ground-truth XYZ 사용
                    print(f"  [match] {closest['color']} prim ({dist_xy:.2f}m away from deproject)")
                elif closest['path'] in picked_set:
                    print(f"  [match SKIP] closest prim {closest['path']} 이미 picked_set 에 있음 (dist_xy={dist_xy:.2f}m)")
                else:
                    print(f"  [match MISS] closest {closest['color']} dist_xy={dist_xy:.2f}m > {MINERAL_MATCH_RADIUS}m radius")

            # picked dedup — prim path 있으면 path 로, 없으면 위치 cell 로
            key = mineral_path or (round(target_xyz[0]/0.5),
                                    round(target_xyz[1]/0.5),
                                    round(target_xyz[2]/0.5))
            if key in picked_set:
                print(f"  [manip-start FAIL] key {key} already in picked_set "
                      f"({len(picked_set)} entries). deproject=({target_xyz[0]:+.2f},"
                      f"{target_xyz[1]:+.2f},{target_xyz[2]:+.2f}) class={nearest.get('name')}")
                return False
            picked_set.add(key)
            manip_lock_pos[0] = pos.copy()
            manip_lock_yaw[0] = float(yaw)
            sm.start_pick(mineral_actual_xyz, mineral_path=mineral_path,
                          class_hint=nearest.get('name'))
            src = "terrain prim" if mineral_path else "deproject only"
            print(f"[manip] target = {nearest['name']} @ "
                  f"({mineral_actual_xyz[0]:+.2f},{mineral_actual_xyz[1]:+.2f},"
                  f"{mineral_actual_xyz[2]:+.2f}) ({src}, dist={d:.2f}m)")
            return True
        except Exception as e:
            print(f"[manip] start_pick 진입 중 오류: {e.__class__.__name__}: {e}")
            import traceback; traceback.print_exc()
            return False

    try:
        while simulation_app.is_running() and not quit_flag[0]:
            # ── 차량 이동 ──
            dt = 1.0 / 60.0  # 60fps 기준 추정
            manual = any(k in pressed for k in (KEY.W, KEY.S, KEY.A, KEY.D))

            # M 키로 manipulation abort
            if abort_manip_flag[0]:
                if sm is not None and sm.busy:
                    sm.abort()
                abort_manip_flag[0] = False
                manip_lock_pos[0] = None
                manip_lock_yaw[0] = None

            # ──────────── 우선순위: MANIPULATION > MANUAL > AUTOPILOT ────────────
            in_manip = (sm is not None and sm.busy)

            if in_manip:
                # rover 위치 freeze (manipulation 중 흔들리면 IK 가 어긋남)
                if manip_lock_pos[0] is not None:
                    pos[:] = manip_lock_pos[0]
                if manip_lock_yaw[0] is not None:
                    yaw = manip_lock_yaw[0]
                # manual 입력 시 manipulation abort 옵션 — 안전 위해 그냥 무시
            elif manual:
                if KEY.W in pressed:
                    pos[0] += LIN_SPEED * dt * np.cos(yaw)
                    pos[1] += LIN_SPEED * dt * np.sin(yaw)
                if KEY.S in pressed:
                    pos[0] -= LIN_SPEED * dt * np.cos(yaw)
                    pos[1] -= LIN_SPEED * dt * np.sin(yaw)
                if KEY.A in pressed:
                    yaw += ANG_SPEED * dt
                if KEY.D in pressed:
                    yaw -= ANG_SPEED * dt
            elif AUTO_ENABLED[0]:
                # autopilot: nav cam det_summary 에서 가장 가까운 mineral 추적
                target = None
                try:
                    valid = [d for d in det_summary if np.isfinite(d["dist"])]
                    if valid:
                        target = min(valid, key=lambda d: d["dist"])
                except NameError:
                    pass

                # ── 최우선: ENGAGE push 진행 중이면 target 무시하고 push 직진 계속 ──
                if engage_pushing[0]:
                    push_dist = ENGAGE_X_PUSH_PER_CLASS.get(
                        engage_push_class[0], ENGAGE_X_PUSH)
                    step_amt = push_dist / float(ENGAGE_PUSH_FRAMES)
                    cy_ = np.cos(engage_push_yaw[0])
                    sy_ = np.sin(engage_push_yaw[0])
                    pos[0] += step_amt * cy_
                    pos[1] += step_amt * sy_
                    engage_push_frame[0] += 1
                    if engage_push_frame[0] >= ENGAGE_PUSH_FRAMES:
                        # push 완료 → manipulation 시도 (snapshot 우선)
                        started = _start_manipulation_if_possible(
                            snapshot=engage_push_target[0])
                        print(f"  [engage] X push +{push_dist:.2f}m done "
                              f"(class={engage_push_class[0]}) → manip "
                              f"{'OK' if started else 'FAIL'}")
                        engage_pushing[0] = False
                        engage_push_frame[0] = 0
                        engage_push_class[0] = None
                        engage_push_target[0] = None
                        if not started:
                            # 실패 (이미 picked or target 사라짐) — yaw 비틀고 다음 탐색
                            yaw += 0.3 * dt
                            pos[0] += AUTO_LIN_SPEED * dt * np.cos(yaw)
                            pos[1] += AUTO_LIN_SPEED * dt * np.sin(yaw)
                elif target is not None:
                    cx_norm = (target["cx"] - res_w / 2.0) / (res_w / 2.0)
                    last_target[0] = (cx_norm, target["dist"])
                    if target["dist"] > ENGAGE_DISTANCE:
                        # ENGAGE 밖 — full speed 직진 + 중앙 정렬 steering
                        yaw -= STEER_GAIN * cx_norm * dt
                        pos[0] += AUTO_LIN_SPEED * dt * np.cos(yaw)
                        pos[1] += AUTO_LIN_SPEED * dt * np.sin(yaw)
                    else:
                        # ENGAGE 진입 — target snapshot + push 시작 (yaw freeze + frame counter)
                        if sm is not None and not sm.busy:
                            # push 시작 시점 카메라 pose 캡쳐 (conf drop 대비 snapshot)
                            snap_cam_pos, snap_cam_rot = _read_world_pose_mat(paths["nav_cam"])
                            engage_push_target[0] = {
                                'name': target.get("name"),
                                'cx':   target["cx"],
                                'cy':   target["cy"],
                                'dist': target["dist"],
                                'cam_pos': snap_cam_pos.copy() if snap_cam_pos is not None else None,
                                'cam_rot': snap_cam_rot.copy() if snap_cam_rot is not None else None,
                            }
                            engage_pushing[0] = True
                            engage_push_frame[0] = 0
                            engage_push_yaw[0] = float(yaw)
                            engage_push_class[0] = target.get("name")
                            push_dist = ENGAGE_X_PUSH_PER_CLASS.get(
                                engage_push_class[0], ENGAGE_X_PUSH)
                            print(f"  [engage] start X push +{push_dist:.2f}m forward "
                                  f"over {ENGAGE_PUSH_FRAMES} frames "
                                  f"(class={engage_push_class[0]}, yaw={np.rad2deg(yaw):+.1f}°)")
                else:
                    last_target[0] = None
                    pos[0] += AUTO_LIN_SPEED * dt * np.cos(yaw)
                    pos[1] += AUTO_LIN_SPEED * dt * np.sin(yaw)

            # Manipulation DONE → autopilot 재개 준비
            if sm is not None and sm.state == "DONE":
                print(f"[manip] DONE — autopilot 복귀, picked={len(picked_set)} 개")
                sm.state = "IDLE"
                manip_lock_pos[0] = None
                manip_lock_yaw[0] = None

            # heightmap 기반 ground z 추적 — anchor target z 갱신
            gz = ground_z(pos[0], pos[1], default=pos[2] - GROUND_CLEARANCE)
            pos[2] = gz + GROUND_CLEARANCE

            # RoverAnchor 의 localPos0 / localRot0 갱신 → PhysX 가 rover 를 그 쪽으로 끌고감
            anchor_pos_attr.Set(Gf.Vec3f(float(pos[0]), float(pos[1]), float(pos[2])))
            # yaw → quaternion (Z axis)
            _half = float(yaw) * 0.5
            anchor_rot_attr.Set(Gf.Quatf(float(np.cos(_half)), 0.0, 0.0, float(np.sin(_half))))

            world.step(render=True)
            step_i += 1

            # ── Manipulation state machine tick (physics step 직후) ──
            if sm is not None and sm.busy:
                try:
                    sm.step()
                except Exception as e:
                    print(f"[manip] step error: {e.__class__.__name__}: {e}")
                    sm.abort()
                    manip_lock_pos[0] = None
                    manip_lock_yaw[0] = None

            # ── YOLO inference (interval) — nav + wrist 두 카메라 ──
            if step_i % max(1, args.interval) == 0:
                # === NAV CAM ===
                annotated, det_summary = None, []
                rgba = camera.get_rgba()
                if rgba is not None and rgba.size > 0:
                    bgr = cv2.cvtColor(rgba[..., :3], cv2.COLOR_RGB2BGR)
                    try:
                        depth = camera.get_current_frame()["distance_to_image_plane"]
                    except Exception:
                        depth = None
                    last_results = yolo.predict(bgr, conf=args.conf, iou=args.iou,
                                                verbose=False, imgsz=1280)
                    annotated, det_summary = annotate(bgr, last_results, args.conf, depth)

                # === WRIST CAM ===
                wrist_annotated, wrist_summary = None, []
                if wrist_camera is not None:
                    rgba_w = wrist_camera.get_rgba()
                    if rgba_w is not None and rgba_w.size > 0:
                        bgr_w = cv2.cvtColor(rgba_w[..., :3], cv2.COLOR_RGB2BGR)
                        try:
                            depth_w = wrist_camera.get_current_frame()["distance_to_image_plane"]
                        except Exception:
                            depth_w = None
                        wrist_results = yolo.predict(bgr_w, conf=args.conf, iou=args.iou,
                                                     verbose=False, imgsz=1280)
                        wrist_annotated, wrist_summary = annotate(bgr_w, wrist_results,
                                                                  args.conf, depth_w)
                # state machine 의 wrist visual servoing 용 — inference 한 결과 매번 주입
                if sm is not None:
                    sm.wrist_summary = wrist_summary

                # 둘 다 못 얻으면 UI 갱신 자체 skip (continue 안 함 — physics step 은 계속)
                if annotated is None and wrist_annotated is None:
                    continue

                # FPS 계산
                fps_count += 1
                now = time.time()
                if now - last_fps_t > 1.0:
                    fps_disp = fps_count / (now - last_fps_t)
                    fps_count = 0
                    last_fps_t = now

                # 상태 텍스트 — 가장 가까운 객체 거리도 표시
                n_det = len(det_summary)
                if det_summary:
                    valid_dets = [d for d in det_summary if np.isfinite(d["dist"])]
                    if valid_dets:
                        valid_dets.sort(key=lambda x: x["dist"])
                        nearest = valid_dets[0]
                        near_str = f"  nearest: {nearest['name']} {nearest['dist']:.2f}m"
                    else:
                        near_str = ""
                else:
                    near_str = ""
                n_det_w = len(wrist_summary)
                if sm is not None and sm.busy:
                    mode = f"MANIP:{sm.state}"
                elif AUTO_ENABLED[0]:
                    mode = "AUTO"
                else:
                    mode = "MAN"
                info = (f"[{mode}]  pos=({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f})  "
                        f"yaw={np.rad2deg(yaw):+.0f}deg  nav={n_det}  "
                        f"wrist={n_det_w}  fps={fps_disp:.1f}{near_str}  "
                        f"picked={len(picked_set)}")
                if annotated is not None:
                    cv2.putText(annotated, info, (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

                # UI provider 갱신 helper — Isaac Sim 버전마다 API 다를 수 있어 여러 가지 시도
                def _push(provider, rgba):
                    h_, w_ = rgba.shape[:2]
                    size = [w_, h_]
                    try:
                        provider.set_bytes_data(rgba.tobytes(), size)
                        return True
                    except Exception:
                        pass
                    try:
                        provider.set_data_array(rgba, size)
                        return True
                    except Exception:
                        pass
                    try:
                        provider.set_bytes_data(list(rgba.tobytes()), size)
                        return True
                    except Exception:
                        pass
                    return False

                if annotated is not None:
                    nav_rgba = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGBA)
                    _push(img_provider, nav_rgba)
                    cv2.imwrite(str(live_path), annotated)

                if wrist_annotated is not None:
                    wrist_rgba = cv2.cvtColor(wrist_annotated, cv2.COLOR_BGR2RGBA)
                    _push(img_provider_wrist, wrist_rgba)

                # 콘솔에 탐지 요약 (거리 포함)
                if n_det > 0:
                    items = []
                    for d in det_summary:
                        if np.isfinite(d["dist"]):
                            items.append(f"{d['name']}@{d['dist']:.2f}m({d['conf']:.2f})")
                        else:
                            items.append(f"{d['name']}({d['conf']:.2f})")
                    det_str = ", ".join(items)
                    print(f"  [det] step={step_i} pos=({pos[0]:+.1f},{pos[1]:+.1f}) yaw={np.rad2deg(yaw):+.0f}° | {det_str}")

                if snap_flag[0] and annotated is not None:
                    snap_path = out_dir / f"shot_{snap_idx:04d}.png"
                    cv2.imwrite(str(snap_path), annotated)
                    print(f"  [P] saved {snap_path}")
                    snap_idx += 1
                    snap_flag[0] = False

    except KeyboardInterrupt:
        pass
    finally:
        try:
            input_iface.unsubscribe_to_keyboard_events(app_window.get_keyboard(), sub_id)
        except Exception:
            pass
        try:
            del camera
        except Exception:
            pass
        try:
            del wrist_camera
        except Exception:
            pass
        try:
            world.stop()
        except Exception:
            pass
        time.sleep(0.5)
        simulation_app.close()


if __name__ == "__main__":
    main()
