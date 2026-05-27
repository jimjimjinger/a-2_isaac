"""Cyan cube pickup demo.

시나리오:
  1) rover + M0609 + RG2 씬을 ground level 에서 build (free-fall 없음).
  2) cyan cube 를 그리퍼 도달 범위에 미리 배치.
  3) wrist 카메라 (RG2 angle_bracket 자식) 로 cube 가시성 확인 (HSV).
  4) 단순 scripted sequence: 그리퍼 open → 천천히 cube 위로 내림 →
     그리퍼 close + FixedJoint attach (cube ↔ gripper_body) → lift → 끝.

build_rover_m0609_scene.py 의 빌더 패턴을 재사용 (rover spawn z 만 낮춤).
isaac_perception/cyan_detector.py 를 import 해서 detection visibility 확인.

실행:
    isaac-python ~/dev_ws/rover_ws/src/a2_isaac/isaac_manipulation/scripts/pickup_demo.py
"""
from __future__ import annotations

import argparse
import os
import sys
import tempfile
import time

# stdout line-buffered (tee 캡처용)
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

# URDF importer 가 cwd 에 임시 mesh 를 쓰므로 쓰기 가능 디렉토리로 chdir
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

# 시끄러운 로그 silence
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
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_setup.assembler import RobotAssembler

# isaac_perception 모듈 path 추가 (ROS2 install 없이 직접 import)
_PKG_PARENT = "/home/rokey/dev_ws/rover_ws/src/a2_isaac"
sys.path.insert(0, f"{_PKG_PARENT}/isaac_perception")
from isaac_perception.cyan_detector import CyanDetector


# ────────────────────────────────────────────────────────────────────
# 경로 상수
# ────────────────────────────────────────────────────────────────────
from pathlib import Path

A2_ROOT = Path("/home/rokey/dev_ws/rover_ws/src/a2_isaac")
MARS_WORLD_USD = A2_ROOT / "isaac_sim/worlds/mars_exploration_world.usd"
ROVER_USD = A2_ROOT / "isaac_sim/assets/rover/Mars_Rover.usd"
M0609_URDF = A2_ROOT / "isaac_sim/assets/doosan-robot2/urdf/m0609_isaac_sim.urdf"
RG2_URDF = A2_ROOT / "isaac_sim/assets/onrobot_rg2/urdf/onrobot_rg2.urdf"

# ────────────────────────────────────────────────────────────────────
# 씬 파라미터 — 사용자 조정 포인트
# ────────────────────────────────────────────────────────────────────
# Rover spawn 위치. Body 를 world 에 PhysicsFixedJoint 로 anchor 시켜
# 자유낙하 없이 spawn 위치 그대로 유지. wheels 접지 여부는 시각만 영향.
SPAWN_X = 5.0
SPAWN_Y = 0.0
SPAWN_Z = 0.30

# M0609 base_link 는 rover Body top + offset (이전 spike 검증값)
M0609_MOUNT_OFFSET_X = 0.15274
M0609_MOUNT_OFFSET_Y = 0.0
M0609_MOUNT_OFFSET_Z = 0.21232

# 큐브: 그리퍼 도달 범위에 (M0609 reach ≈ 0.9m). 지면 위에 놓임.
CUBE_SIZE = 0.05  # 5 cm
CUBE_POS = (
    SPAWN_X + 0.5,                          # rover 앞쪽 0.5m
    SPAWN_Y + 0.0,
    SPAWN_Z - 0.30 + CUBE_SIZE / 2,         # 지면 z (rover.Body 아래) + 큐브 절반
)
CUBE_COLOR = (0.0, 1.0, 1.0)                # RGB cyan

# 그리퍼 시퀀스 z 좌표 (world)
APPROACH_Z = CUBE_POS[2] + 0.25              # 위에서 hover
GRASP_Z = CUBE_POS[2] + 0.03                 # 잡기 위해 살짝 위 (그리퍼 두께 보정)
LIFT_Z = CUBE_POS[2] + 0.30                  # 들어올린 최종 높이

# 카메라
CAMERA_RESOLUTION = (640, 480)
CAMERA_LOCAL_TRANSLATE = (0.0, 0.045, 0.05)  # angle_bracket 기준
CAMERA_LOCAL_RPY_DEG = (0.0, -90.0, 90.0)    # gripper 아래로 90° down

# M0609 준비 자세 (find_home_pose.py 로 측정 — dist 0.07m, 큐브 바로 위) — joint_1 ~ joint_6
HOME_JOINT_DEG = (0.0, 90.0, 90.0, 0.0, 0.0, 0.0)
# Lift 시점 자세 (joint_2 90→60 → tool0 z 0.16 → 0.43, 약 27cm 위로)
LIFT_JOINT_DEG = (0.0, 60.0, 90.0, 0.0, 0.0, 0.0)


# ────────────────────────────────────────────────────────────────────
# URDF / Assembler 헬퍼 (build_rover_m0609_scene.py 와 동일)
# ────────────────────────────────────────────────────────────────────
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


# ────────────────────────────────────────────────────────────────────
# 씬 빌드
# ────────────────────────────────────────────────────────────────────
def build_scene():
    stage = omni.usd.get_context().get_stage()

    # ① mars 환경 + PhysicsScene + terrain collision
    print("\n[1/6] mars world …")
    mars_prim = stage.DefinePrim("/World/Mars", "Xform")
    mars_prim.GetReferences().AddReference(str(MARS_WORLD_USD))
    for _ in range(8):
        simulation_app.update()
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene") \
        .CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    UsdPhysics.Scene(stage.GetPrimAtPath("/World/PhysicsScene")) \
        .CreateGravityMagnitudeAttr().Set(3.72)
    # terrain mesh collision 보강 (main 의 terrain 은 이미 baked, idempotent)
    tm = stage.GetPrimAtPath("/World/Mars/Terrain/TerrainMesh")
    if tm.IsValid() and not tm.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(tm)
        UsdPhysics.MeshCollisionAPI.Apply(tm).CreateApproximationAttr().Set("meshSimplification")

    # ② Rover (지면 가까이)
    print("[2/6] rover (ground level) …")
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

    # ③ M0609 (floating, 위치 TransformPrimSRT 로)
    print("[3/6] M0609 URDF …")
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

    # ④ RG2 → M0609 ee
    print("[4/6] RG2 URDF + assemble …")
    gripper_root = _import_urdf(str(RG2_URDF), fix_base=False)
    robot_ee = _find_prim_path_by_name(robot_root, "link_6") or f"{robot_root}/link_6"
    gripper_base = _find_prim_path_by_name(gripper_root, "angle_bracket") \
                   or f"{gripper_root}/angle_bracket"
    _assemble(robot_root, robot_ee, gripper_root, gripper_base, "Gripper", "m0609_rg2")

    # ⑤ rover Body ↔ M0609 mount-point Xform
    print("[5/6] rover ↔ M0609 assemble …")
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

    # ⭐ rover Body 를 world 에 PhysicsFixedJoint 로 anchor → Z 추락 방지.
    # body0 비워두면 world anchor 됨. body1 = rover.Body.
    # localPos1 = (0,0,0) (Body origin), anchor world pos = Body 의 현재 world pos.
    stage.SetEditTarget(Usd.EditTarget(stage.GetRootLayer()))
    anchor_path = "/World/Joints/RoverAnchor"
    stage.DefinePrim("/World/Joints", "Scope")
    anchor = UsdPhysics.FixedJoint.Define(stage, anchor_path)
    anchor.CreateBody1Rel().SetTargets([Sdf.Path(rover_body)])
    anchor.CreateLocalPos0Attr().Set(Gf.Vec3f(SPAWN_X, SPAWN_Y, SPAWN_Z))
    anchor.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    anchor.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    anchor.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    anchor.CreateBreakForceAttr().Set(float('inf'))
    anchor.CreateBreakTorqueAttr().Set(float('inf'))
    print(f"  [anchor] rover.Body world-fixed at ({SPAWN_X}, {SPAWN_Y}, {SPAWN_Z})")

    # ⑥ cyan cube + wrist 카메라
    print("[6/6] cyan cube + wrist camera …")
    cube_prim_path = "/World/cyan_cube"
    cube = DynamicCuboid(
        prim_path=cube_prim_path,
        name="cyan_cube",
        position=np.array(CUBE_POS),
        scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]),
        color=np.array(CUBE_COLOR),
        mass=0.05,
    )
    # cube ↔ world FixedJoint: spawn 위치에 anchor → 자유낙하 없음.
    # close phase 에서 anchor 제거 후 gripper 에 attach.
    cube_anchor_path = "/World/Joints/CubeAnchor"
    cube_anchor = UsdPhysics.FixedJoint.Define(stage, cube_anchor_path)
    cube_anchor.CreateBody1Rel().SetTargets([Sdf.Path(cube_prim_path)])
    cube_anchor.CreateLocalPos0Attr().Set(Gf.Vec3f(*CUBE_POS))
    cube_anchor.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    cube_anchor.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    cube_anchor.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    cube_anchor.CreateBreakForceAttr().Set(float('inf'))
    cube_anchor.CreateBreakTorqueAttr().Set(float('inf'))
    print(f"  [anchor] cyan cube world-fixed at {CUBE_POS}")

    # 카메라: RG2 angle_bracket 자식 Xform 으로 만들고 Camera prim 생성
    cam_parent = _find_prim_path_by_name(robot_root, "angle_bracket")
    cam_xform_path = f"{cam_parent}/wrist_cam"
    cam_xform = stage.DefinePrim(cam_xform_path, "Xform")
    cam_x = UsdGeom.Xformable(cam_xform)
    cam_x.ClearXformOpOrder()
    cam_x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*CAMERA_LOCAL_TRANSLATE))
    cam_x.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*CAMERA_LOCAL_RPY_DEG))
    cam_path = f"{cam_xform_path}/Camera"
    UsdGeom.Camera.Define(stage, cam_path)

    for _ in range(10):
        simulation_app.update()

    return rover_prim, robot_root, cube, cam_path


# ────────────────────────────────────────────────────────────────────
# Pickup 시퀀스 — 단순 scripted (FSM 없이)
# ────────────────────────────────────────────────────────────────────
def _attach_cube_to_gripper(stage, joint_path, gripper_body_path, cube_path):
    """Cube ↔ gripper_body FixedJoint 생성 (lift 안정용)."""
    if stage.GetPrimAtPath(joint_path).IsValid():
        stage.RemovePrim(joint_path)
    link_prim = stage.GetPrimAtPath(gripper_body_path)
    cube_prim = stage.GetPrimAtPath(cube_path)
    if not link_prim.IsValid() or not cube_prim.IsValid():
        return False
    link_xf = UsdGeom.Xformable(link_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    cube_xf = UsdGeom.Xformable(cube_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    rel = cube_xf * link_xf.GetInverse()
    rel_pos = rel.ExtractTranslation()
    rel_rot = rel.ExtractRotationQuat()
    im = rel_rot.GetImaginary()
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(gripper_body_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(cube_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(rel_pos))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(
        rel_rot.GetReal(), float(im[0]), float(im[1]), float(im[2])
    ))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    return True


def main():
    try:
        _run_demo()
    except Exception as e:
        import traceback
        print(f"\n[FATAL] {e.__class__.__name__}: {e}", flush=True)
        traceback.print_exc()
    finally:
        # 어떤 에러든 cleanup → os._exit 으로 atexit(omni.graph) 우회.
        try:
            import cv2
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            simulation_app.close()
        except Exception:
            pass
        os._exit(0)


def _run_demo():
    ap = argparse.ArgumentParser()
    ap.add_argument("--auto-play", action="store_true")
    ap.add_argument("--max-sec", type=float, default=180.0)
    args = ap.parse_args()

    world = World(stage_units_in_meters=1.0)
    rover_prim, robot_root, cube, cam_path = build_scene()

    # ArticulationController 사용 위해 SingleArticulation 등록 (world.reset 전에)
    _init_m0609_articulation(robot_root, world)

    print("\n[World] reset …")
    world.reset()
    _freeze_rover_drives(rover_prim)
    stage = omni.usd.get_context().get_stage()
    # 준비 자세 — apply_action + teleport 으로 즉시 도달
    n_home = _set_m0609_pose(stage, robot_root, HOME_JOINT_DEG, teleport=True)
    print(f"  [home pose] {HOME_JOINT_DEG} deg applied to {n_home} joints")
    for _ in range(20):
        world.step(render=True)

    # 카메라 핸들
    from isaacsim.sensors.camera import Camera
    wrist_cam = Camera(prim_path=cam_path, resolution=CAMERA_RESOLUTION)
    wrist_cam.initialize()

    detector = CyanDetector()

    # OpenCV 디버그 창 — Isaac Sim 의 cv2 가 headless 빌드면 namedWindow 실패.
    # 실패 시 console print 로 fallback.
    import cv2
    WIN_NAME = "Wrist Camera — Cyan Cube Detector"
    cv_window_ok = False
    try:
        cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN_NAME, 960, 540)
        cv2.moveWindow(WIN_NAME, 20, 20)
        cv_window_ok = True
        print(f"  [cv2] window '{WIN_NAME}' created")
    except cv2.error as e:
        print(f"  [cv2] window NOT supported (headless build) — fallback to console print")
        print(f"        ({e.__class__.__name__}: line 1)")
    # cv2 자체는 HSV 처리에 필요하므로 import 는 유지 (단 imshow 만 skip)
    cv_ok = True

    gripper_body_path = _find_prim_path_by_name(robot_root, "gripper_body") \
                        or f"{robot_root}/gripper_body"
    GRIP_JOINT = "/World/cyan_grip_joint"

    # ── 단순 scripted sequence ──────────────────────────────────────
    # phase: 'home_settle' → 'detect' → 'open' → 'descend' → 'close' → 'lift' → 'done'
    phase = "home_settle"
    phase_t0 = time.time()
    print("\n[READY] Spacebar 로 시뮬 시작 (또는 --auto-play)\n")

    if args.auto_play:
        time.sleep(1.0)
        world.play()

    boot_t = time.time()
    was_playing = False

    while simulation_app.is_running():
        world.step(render=True)
        if (time.time() - boot_t) > args.max_sec:
            print("\n[TIMEOUT]\n")
            break
        if not world.is_playing():
            was_playing = False
            continue
        if not was_playing:
            # Play 시작 한번 — drive target 재적용 (world.reset 시 0 으로 reset 됐을 수 있음)
            was_playing = True
            _set_m0609_pose(stage, robot_root, HOME_JOINT_DEG)
            phase = "home_settle"
            phase_t0 = time.time()
            print("[RUN] start — home pose drive re-applied, waiting for arm to settle")

        # 카메라 + detection
        rgba = wrist_cam.get_rgba()
        det = None
        if rgba is not None and rgba.size > 0 and cv_ok:
            bgr = cv2.cvtColor(rgba[..., :3], cv2.COLOR_RGB2BGR)
            det = detector.detect(bgr)

            # ── 시각화 (RGB + mask side-by-side) ──
            vis = bgr.copy()
            h_img, w_img = vis.shape[:2]
            # crosshair (이미지 중앙)
            cx_img, cy_img = w_img // 2, h_img // 2
            cv2.line(vis, (cx_img - 10, cy_img), (cx_img + 10, cy_img), (200, 200, 200), 1)
            cv2.line(vis, (cx_img, cy_img - 10), (cx_img, cy_img + 10), (200, 200, 200), 1)
            # detection bbox + centroid
            if det.found:
                x, y, w, h = det.bbox
                cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(vis, (int(det.cx), int(det.cy)), 6, (0, 0, 255), -1)
                cv2.line(vis, (cx_img, cy_img), (int(det.cx), int(det.cy)),
                         (255, 255, 0), 1)
            # 상태 패널 (좌측 상단)
            status_color = (0, 255, 0) if (det and det.found) else (0, 128, 255)
            cv2.rectangle(vis, (0, 0), (w_img, 60), (30, 30, 30), -1)
            cv2.putText(vis, f"phase: {phase.upper()}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            det_txt = (f"cyan: found  cx={det.cx:.0f} cy={det.cy:.0f} area={det.area:.0f}"
                       if det and det.found else "cyan: not detected")
            cv2.putText(vis, det_txt, (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 2)

            # HSV mask 옆에 붙이기 (옵션)
            mask_vis = None
            if det is not None and det.mask is not None:
                mask_vis = cv2.cvtColor(det.mask, cv2.COLOR_GRAY2BGR)
                cv2.putText(mask_vis, "HSV mask (cyan)", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                combined = np.hstack([vis, mask_vis])
            else:
                combined = vis

            if cv_window_ok:
                try:
                    cv2.imshow(WIN_NAME, combined)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                except cv2.error:
                    cv_window_ok = False  # 한 번 실패하면 더 시도 안 함

        # phase 진행 (시간 기반 단순 시퀀스)
        elapsed = time.time() - phase_t0
        if phase == "home_settle":
            # M0609 이 home pose (0,0,90,0,90,0) 로 도달할 시간. 1e10 drive 라
            # 보통 1~2초 안에 잡힘. 3초 대기 후 detect 진입.
            if elapsed > 3.0:
                # 진단: 실제 joint 위치 + tool0 (gripper tip) world position
                deg_now = _read_joint_deg(stage, robot_root)
                tool0_path = _find_prim_path_by_name(robot_root, "tool0")
                tool0_world = (None, None, None)
                if tool0_path:
                    tp = stage.GetPrimAtPath(tool0_path)
                    if tp.IsValid():
                        t = UsdGeom.Xformable(tp).ComputeLocalToWorldTransform(
                            Usd.TimeCode.Default()).ExtractTranslation()
                        tool0_world = (round(t[0], 3), round(t[1], 3), round(t[2], 3))
                print(f"  [home_settle] joints (deg) = {deg_now}")
                print(f"  [home_settle] tool0 world  = {tool0_world}  "
                      f"(rover origin ≈ ({SPAWN_X}, {SPAWN_Y}, {SPAWN_Z}), "
                      f"cube ≈ {CUBE_POS})")
                print(f"  [home_settle] → detect")
                phase = "detect"; phase_t0 = time.time()
        elif phase == "detect":
            # cube 가 카메라에 보일 때까지 대기 (최대 10초, 보이면 즉시 진행)
            if det is not None and det.found:
                print(f"  [detect] found cyan cx={det.cx:.0f} cy={det.cy:.0f} "
                      f"area={det.area:.0f} → open")
                phase = "open"; phase_t0 = time.time()
            elif elapsed > 10.0:
                print(f"  [detect] timeout 10s — proceeding anyway → open")
                phase = "open"; phase_t0 = time.time()
        elif phase == "open":
            # 그리퍼 open command (단순화: RG2 finger joint 직접)
            _drive_finger(stage, robot_root, target=0.0)
            if elapsed > 1.5:
                print("  [open] → descend")
                phase = "descend"; phase_t0 = time.time()
        elif phase == "descend":
            # cube 바로 위로 천천히 이동 (cube top 위 GRASP_Z 까지). 직접 cube 옆으로
            # 단순화: cube 를 큰 거리에서 텔레포트 → gripper 위치로
            # 대신 여기서는 phase 만 진행 (실제 IK 는 별도 구현 필요).
            if elapsed > 2.0:
                print("  [descend] (simplified — cube teleport in close phase) → close")
                phase = "close"; phase_t0 = time.time()
        elif phase == "close":
            _drive_finger(stage, robot_root, target=0.8)
            # cube ↔ gripper FixedJoint attach + cube↔world anchor 제거
            if elapsed > 0.5:
                # cube anchor 제거 — 그래야 그리퍼 따라 움직일 수 있음
                if stage.GetPrimAtPath("/World/Joints/CubeAnchor").IsValid():
                    stage.RemovePrim("/World/Joints/CubeAnchor")
                    print("  [close] cube anchor removed")
                attached = _attach_cube_to_gripper(stage, GRIP_JOINT,
                                                   gripper_body_path,
                                                   cube.prim_path)
                # LIFT 자세로 drive target 변경 — arm 이 천천히 들어올림
                _set_m0609_pose(stage, robot_root, LIFT_JOINT_DEG)
                print(f"  [close] gripper closed, cube attached={attached}, "
                      f"lift pose {LIFT_JOINT_DEG} deg → lift")
                phase = "lift"; phase_t0 = time.time()
        elif phase == "lift":
            if elapsed > 3.0:
                print("\n[DONE] pickup sequence complete\n")
                phase = "done"
                world.pause()
        # done: 그냥 유지

    # _run_demo 정상 종료 시 main() 의 finally 블록이 cv2/SimApp cleanup + os._exit 처리.


def _drive_finger(stage, robot_root: str, target: float):
    """RG2 finger_joint 의 position drive target 직접 set (0 = open, ~0.8 = close)."""
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_root)):
        if prim.GetName() in ("finger_joint", "right_inner_knuckle_joint"):
            drv = UsdPhysics.DriveAPI.Get(prim, "angular")
            if drv:
                drv.GetTargetPositionAttr().Set(float(target))


def _read_joint_deg(stage, robot_root: str):
    """joint_1 ~ joint_6 의 현재 USD-author 위치(도) 반환. 진단용."""
    out = []
    for i in range(6):
        name = f"joint_{i+1}"
        val = None
        for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_root)):
            if prim.GetName() == name:
                # joint state (실제 PhysX 위치) 는 fabric/articulation 으로 읽어야
                # 하나, 여기서는 simple 하게 USD 의 state:angular:physics:position 읽기.
                attr = prim.GetAttribute("state:angular:physics:position")
                if attr.IsValid() and attr.Get() is not None:
                    val = float(np.rad2deg(attr.Get()))
                break
        out.append(round(val, 1) if val is not None else None)
    return tuple(out)


# 모듈 전역 articulation 핸들 (lazy init — _set_m0609_pose 가 호출되며 채워짐)
_M0609_ART: SingleArticulation | None = None
_M0609_JOINT_IDX: list[int] | None = None


def _init_m0609_articulation(robot_root: str, world: "World"):
    """SingleArticulation 등록 + joint_1~6 index 추출. world.reset() 후에 호출."""
    global _M0609_ART, _M0609_JOINT_IDX
    art = SingleArticulation(prim_path=robot_root, name="m0609_art")
    world.scene.add(art)
    _M0609_ART = art
    return art


def _resolve_m0609_indices():
    """world.reset() 으로 art 가 살아난 뒤 dof_names 가 채워지면 인덱스 캐싱."""
    global _M0609_JOINT_IDX
    if _M0609_ART is None or _M0609_ART.dof_names is None:
        return False
    names = list(_M0609_ART.dof_names)
    idx = []
    for i in range(6):
        n = f"joint_{i+1}"
        if n in names:
            idx.append(names.index(n))
    if len(idx) != 6:
        print(f"[WARN] joint_1~6 인덱스 부족: {idx} (dof={names})")
        return False
    _M0609_JOINT_IDX = idx
    return True


def _set_m0609_pose(stage, robot_root: str, joint_angles_deg, teleport: bool = False) -> int:
    """joint_1~6 position drive target 적용.

    play 후 USD DriveAPI write 는 PhysX 가 안 읽으므로 ArticulationController.apply_action
    을 사용. _init_m0609_articulation + world.reset() 가 선행되어야 함.

    teleport=True 면 set_joint_positions 로 즉시 텔레포트 (settle 대기 0초).
    초기 home pose 적용 같은 경우 유용.
    """
    if _M0609_ART is None:
        print("[WARN] m0609 articulation 미초기화 — _set_m0609_pose 무시")
        return 0
    if _M0609_JOINT_IDX is None and not _resolve_m0609_indices():
        return 0
    rad = np.array([np.deg2rad(joint_angles_deg[i]) for i in range(6)], dtype=np.float32)
    idx_arr = np.array(_M0609_JOINT_IDX, dtype=np.int32)
    controller = _M0609_ART.get_articulation_controller()
    controller.apply_action(ArticulationAction(joint_positions=rad, joint_indices=idx_arr))
    if teleport:
        try:
            cur = _M0609_ART.get_joint_positions()
            if cur is not None:
                cur = np.array(cur, dtype=np.float32).copy()
                for k, j in enumerate(_M0609_JOINT_IDX):
                    cur[j] = rad[k]
                _M0609_ART.set_joint_positions(cur)
        except Exception as e:
            print(f"  [warn] set_joint_positions failed: {e}")
    return 6


if __name__ == "__main__":
    main()
