"""vehicle_v3.usd 빌드 — vehicle_v2.usd 의 외형·물리를 flatten 으로 끌어와
ROS2 Action Graph 를 구워넣은(bake) "액션그래프 내장 자립 로버".

v3 = 고정된 로봇. terrain 에 reference·play 하면 그 자체로 ROS2 인터페이스를
발행/구독한다 — 런타임 그래프 빌더 코드가 필요 없다. 실물 하드웨어처럼.

v3 는 vehicle_v2.usd 를 *reference 하지 않는다* — flatten 으로 외형·물리·관절을
v3 자체에 inline 한 자립(standalone) 파일이다. v2 는 빌드 입력 소스일 뿐.
→ v2 수정은 v3 에 자동 전파되지 않음. v2 수정 후 이 스크립트 재실행 = v3 재bake.

구조:  vehicle_v3.usd  (standalone, defaultPrim=/Root)
         /Root
           ├ Vehicle/...     (v2 에서 flatten 된 외형·물리·센서 prim)
           └ ActionGraph     (이 스크립트가 굽는 그래프)
               · 센서: IMU·joint·카메라 발행
               · 주행: /cmd_vel 구독 → ScriptNode Ackermann → 휠 관절 구동
               · 팔: /arm/joint_command(JointState) 구독 → m0609 관절 위치 제어
               · GT (dev cheat): /ground_truth/odom 발행 — 로버 절대 world pose
                 (ScriptNode 으로 articulation world transform 직접 읽음).
                 졸업 시점엔 이 노드만 빼고 재bake 하면 cheat 제거.

빌드:  <isaac-python> isaac_sim/scripts/build_vehicle_v3.py
산출물: isaac_sim/assets/vehicle/vehicle_v3.usd
"""
import os
import sys

try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.usd
import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension
from pxr import Gf, Sdf, Usd, UsdGeom

enable_extension("isaacsim.ros2.bridge")
app.update()

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC_SIM = os.path.dirname(HERE)
VEHICLE_DIR = os.path.join(ISAAC_SIM, "assets", "vehicle")
V2 = os.path.join(VEHICLE_DIR, "vehicle_v2.usd")
V3 = os.path.join(VEHICLE_DIR, "vehicle_v3.usd")

GRAPH = "/Root/ActionGraph"
# v3 /Root 가 v2 를 reference → v2 의 /Root/Vehicle/... 가 /Root/Vehicle/... 로 합성.
# 그래프·target 을 /Root/ 하위로 author → v3 가 terrain 에 reference 될 때
# 경로가 통째로 remap (/Root → /World/Rover).
IMU       = "/Root/Vehicle/rover/Body/Imu_Sensor"
ARTIC     = "/Root/Vehicle/m0609/base_link"
_D455     = "/Root/Vehicle/onrobot_rg2ft/angle_bracket/realsense_d455/RSD455"
ROVER_CAM = "/Root/Vehicle/rover/Body/Camera"
WRIST_RGB = _D455 + "/Camera_OmniVision_OV9782_Color"
WRIST_DEP = _D455 + "/Camera_Pseudo_Depth"

# Sun 위치 추적용 카메라 — Body 상단, +z(하늘) 향함. T5 localization 의 sun_yaw
# 노드가 /camera/sun/{image_raw, camera_info} 를 받아 절대 방위 추정.
# 외형 invisible 처리(편법) — 기구 충돌 0 (UsdGeom.Camera 는 mesh/collision X).
SUN_CAM = "/Root/Vehicle/rover/Body/SunCamera"
SUN_CAM_XYZ = (0.0, 0.0, 0.5)   # Body 좌표 기준 위치 (m)

# Body 카메라 최적 위치 조정 (2026-05-22 사용자 지정): translate x·z.
# y 는 기존값 유지.
ROVER_CAM_XZ = (0.3, -0.1)

# 지민(T5) RoverAckermannDrive ScriptNode 의 6륜 Ackermann (vehicle_v2_scene.usd
# 에서 덤프). /cmd_vel 의 linear/angular → 휠 4개 조향각 + 6개 구동속도.
ACK_SCRIPT = '''
import math
import omni.graph.core as og

STEER_JOINT_NAMES = ['FL_Steer_Revolute', 'FR_Steer_Revolute', 'RL_Steer_Revolute', 'RR_Steer_Revolute']
DRIVE_JOINT_NAMES = ['FL_Drive_Continuous', 'FR_Drive_Continuous', 'CL_Drive_Continuous', 'CR_Drive_Continuous', 'RL_Drive_Continuous', 'RR_Drive_Continuous']
WHEELBASE_LENGTH = 0.849
MIDDLE_WHEEL_DISTANCE = 0.894
REAR_FRONT_WHEEL_DISTANCE = 0.77
WHEEL_RADIUS = 0.1
OFFSET = -0.0135
STEERING_GAIN = 1.1
# 2.0 절충값 (2026-05-26):
#   1.5 (T5 검증) — 안정적이지만 언덕 등반 부족
#   2.5 — 등반 OK 하지만 통통 튐 + rover 뒤로 기울어짐 (급가속 + slip)
#   2.0 — 등반 도움 + 급가속 완화. GT cheat 모드 사용 (localization 무관).
WHEEL_VELOCITY_GAIN = 2.0


def _component(value, index):
    try:
        return float(value[index])
    except Exception:
        try:
            return float(getattr(value, ("x", "y", "z")[index]))
        except Exception:
            return 0.0


def setup(db):
    pass


def compute(db):
    lin_vel = _component(db.inputs.linearVelocity, 0)
    ang_vel = _component(db.inputs.angularVelocity, 2)

    direction = 1.0 if lin_vel >= 0.0 else -1.0
    if ang_vel > 0.0:
        turn_direction = 1.0
    elif ang_vel < 0.0:
        turn_direction = -1.0
    else:
        turn_direction = 0.0

    lin = abs(lin_vel)
    ang = abs(ang_vel)
    turning_radius = float("inf") if ang == 0.0 else lin / ang
    minimum_radius = MIDDLE_WHEEL_DISTANCE * 0.8

    r_ml = turning_radius - (MIDDLE_WHEEL_DISTANCE / 2.0) * turn_direction
    r_mr = turning_radius + (MIDDLE_WHEEL_DISTANCE / 2.0) * turn_direction
    r_fl = math.hypot(r_ml, WHEELBASE_LENGTH / 2.0)
    r_fr = math.hypot(r_mr, WHEELBASE_LENGTH / 2.0)
    r_rl = math.hypot(r_ml, WHEELBASE_LENGTH / 2.0)
    r_rr = math.hypot(r_mr, WHEELBASE_LENGTH / 2.0)

    if turning_radius < minimum_radius:
        vel_fl = -ang * turn_direction
        vel_fr = ang * turn_direction
        vel_rl = -ang * turn_direction
        vel_rr = ang * turn_direction
        vel_ml = -ang * turn_direction
        vel_mr = ang * turn_direction
    else:
        vel_fl = (r_fl * ang if ang > 0.0 else lin) * direction
        vel_fr = (r_fr * ang if ang > 0.0 else lin) * direction
        vel_rl = (r_rl * ang if ang > 0.0 else lin) * direction
        vel_rr = (r_rr * ang if ang > 0.0 else lin) * direction
        vel_ml = (r_ml * ang if ang > 0.0 else lin) * direction
        vel_mr = (r_mr * ang if ang > 0.0 else lin) * direction

    if turning_radius < minimum_radius:
        theta_fl = -math.pi / 4.0
        theta_fr = math.pi / 4.0
        theta_rl = math.pi / 4.0
        theta_rr = -math.pi / 4.0
    else:
        theta_fl = math.atan2((WHEELBASE_LENGTH / 2.0) - OFFSET, r_fl) * turn_direction
        theta_fr = math.atan2((WHEELBASE_LENGTH / 2.0) - OFFSET, r_fr) * turn_direction
        theta_rl = math.atan2((WHEELBASE_LENGTH / 2.0) + OFFSET, r_rl) * -turn_direction
        theta_rr = math.atan2((WHEELBASE_LENGTH / 2.0) + OFFSET, r_rr) * -turn_direction

    db.outputs.steerJointNames = STEER_JOINT_NAMES
    db.outputs.driveJointNames = DRIVE_JOINT_NAMES
    db.outputs.steeringAngles = [
        STEERING_GAIN * theta_fl,
        STEERING_GAIN * theta_fr,
        STEERING_GAIN * theta_rl,
        STEERING_GAIN * theta_rr,
    ][:len(STEER_JOINT_NAMES)]
    db.outputs.wheelVelocities = [
        WHEEL_VELOCITY_GAIN * vel_fl / (WHEEL_RADIUS * 2.0),
        WHEEL_VELOCITY_GAIN * vel_fr / (WHEEL_RADIUS * 2.0),
        WHEEL_VELOCITY_GAIN * vel_ml / (WHEEL_RADIUS * 2.0),
        WHEEL_VELOCITY_GAIN * vel_mr / (WHEEL_RADIUS * 2.0),
        WHEEL_VELOCITY_GAIN * vel_rl / (WHEEL_RADIUS * 2.0),
        WHEEL_VELOCITY_GAIN * vel_rr / (WHEEL_RADIUS * 2.0),
    ][:len(DRIVE_JOINT_NAMES)]
    db.outputs.execOut = og.ExecutionAttributeState.ENABLED
    return True
'''


# Grasp simulator — supervisor 가 /grasp/command (geometry_msgs/Twist) 으로
# pickup/release 요청 → ScriptNode 가 target 좌표 근처 mineral 찾아 FixedJoint
# attach (arm 따라 mineral 이동). release 시 detach + MakeInvisible.
# 메시지 인코딩 (Twist hijack — OmniGraph 에 String subscriber 없어 우회):
#   pickup x y z   → linear=(x,y,z), angular.x = +1.0
#   release         → linear=(0,0,0), angular.x = -1.0
GRASP_SCRIPT = '''
from pxr import UsdPhysics, UsdGeom, Gf, Sdf, Usd
import omni.usd
import omni.graph.core as og
import math

_state = {"attached_joint_path": None, "attached_obj_path": None,
          "gripper_link_path": None}

GRASP_JOINT_PATH = "/World/grip_fixed_joint"
# vehicle_v3 reference 시 prefix remap (/Root → /World/Rover) 되므로 이름으로 검색.
GRIPPER_LINK_NAME = "right_inner_finger"
SEARCH_RADIUS = 1.5  # m  (supervisor 의 lock_target 좌표 정확도 흡수)


def _component(v, i):
    try:
        return float(v[i])
    except Exception:
        return 0.0


def _find_gripper_link(stage):
    """onrobot 그리퍼의 right_inner_finger prim path 를 traverse 로 검색.
    reference prefix 가 바뀌어도 (/Root → /World/Rover) 정상 동작."""
    for prim in stage.Traverse():
        if prim.GetName() == GRIPPER_LINK_NAME and "onrobot" in str(prim.GetPath()):
            return str(prim.GetPath())
    return None


def _find_nearest_mineral(stage, tx, ty):
    """이름이 'Minerals' 인 어떤 scope (e.g. /World/Minerals,
    /World/MarsScene/Minerals) 의 직속 children 중 (tx,ty) 수평거리 최소 prim.
    hardcode 경로 대신 traverse 로 찾아 terrain reference prefix remap 에 robust.
    """
    cache = UsdGeom.XformCache()
    best_path = None
    best_d2 = SEARCH_RADIUS * SEARCH_RADIUS
    for prim in stage.Traverse():
        parent = prim.GetParent()
        if not parent or parent.GetName() != "Minerals":
            continue
        if not prim.IsValid():
            continue
        imageable = UsdGeom.Imageable(prim)
        try:
            if imageable.ComputeVisibility() == UsdGeom.Tokens.invisible:
                continue
        except Exception:
            pass
        M = cache.GetLocalToWorldTransform(prim)
        p = M.ExtractTranslation()
        dx = p[0] - tx
        dy = p[1] - ty
        d2 = dx * dx + dy * dy
        if d2 < best_d2:
            best_d2 = d2
            best_path = str(prim.GetPath())
    return best_path, math.sqrt(best_d2) if best_path else float("inf")


def _attach(stage, link_path, obj_path):
    """gripper link 와 mineral 사이 FixedJoint snap.

    LocalPos0/1 모두 (0,0,0) 강제 → mineral 의 origin 이 gripper link 의 origin
    위치로 즉시 부착. 이전엔 obj_xf * link_xf.Inverse() 로 *현재 거리* 를 그대로
    LocalPos0 에 저장 → mineral 이 finger 위 26cm 떠있어도 그 거리 유지하며
    공중에서 따라옴 (헛손질 원인). 거리 무관 깔끔한 snap 으로 교체.
    """
    if stage.GetPrimAtPath(GRASP_JOINT_PATH).IsValid():
        stage.RemovePrim(GRASP_JOINT_PATH)
    link_prim = stage.GetPrimAtPath(link_path)
    obj_prim = stage.GetPrimAtPath(obj_path)
    if not link_prim.IsValid() or not obj_prim.IsValid():
        return False
    joint = UsdPhysics.FixedJoint.Define(stage, GRASP_JOINT_PATH)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(obj_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    return True


def _detach(stage):
    if stage.GetPrimAtPath(GRASP_JOINT_PATH).IsValid():
        stage.RemovePrim(GRASP_JOINT_PATH)
    return True


def _set_mineral_collision(stage, obj_path, enabled):
    """attach 직후 mineral collision 일시 off 로 PhysX disjointed body lock 충격
    rover 에 전파 차단. release 시 다시 on 으로 복귀해 다음 사용에 영향 X."""
    if not obj_path:
        return
    prim = stage.GetPrimAtPath(obj_path)
    if not prim or not prim.IsValid():
        return
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        attr = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr()
        if attr:
            attr.Set(bool(enabled))


def _hide(stage, obj_path):
    if not obj_path:
        return False
    prim = stage.GetPrimAtPath(obj_path)
    if not prim or not prim.IsValid():
        return False
    try:
        UsdGeom.Imageable(prim).MakeInvisible()
        return True
    except Exception:
        return False


def setup(db):
    global _state
    _state = {"attached_joint_path": None, "attached_obj_path": None}


def compute(db):
    global _state
    stage = omni.usd.get_context().get_stage()
    mode = _component(db.inputs.angularVelocity, 0)  # angular.x as sign marker

    if mode > 0.5:    # pickup
        # gripper link path — terrain reference prefix 따라 바뀌므로 traverse 검색.
        link_path = _state.get("gripper_link_path") or _find_gripper_link(stage)
        if link_path is None:
            print(f"[grasp] pickup FAILED — '{GRIPPER_LINK_NAME}' prim 없음")
        else:
            _state["gripper_link_path"] = link_path
            tx = _component(db.inputs.linearVelocity, 0)
            ty = _component(db.inputs.linearVelocity, 1)
            obj_path, dist = _find_nearest_mineral(stage, tx, ty)
            if obj_path is None:
                print(f"[grasp] pickup ignored — no mineral near "
                      f"({tx:.2f},{ty:.2f}) within {SEARCH_RADIUS}m")
            else:
                ok = _attach(stage, link_path, obj_path)
                if ok:
                    # snap 충격이 rover 에 전파 안 되도록 mineral collision off.
                    _set_mineral_collision(stage, obj_path, False)
                    _state["attached_joint_path"] = GRASP_JOINT_PATH
                    _state["attached_obj_path"] = obj_path
                    print(f"[grasp] pickup OK — attached {obj_path} to "
                          f"{link_path} (target dist {dist:.2f}m, snapped, "
                          f"collision off)")
                else:
                    print(f"[grasp] pickup FAILED — attach error on {obj_path}")
    elif mode < -0.5:  # release
        obj = _state.get("attached_obj_path")
        _detach(stage)
        if obj:
            # invisible 처리 전 collision 복귀 (다음 grasp 또는 다른 노드 영향 X).
            _set_mineral_collision(stage, obj, True)
            _hide(stage, obj)
            print(f"[grasp] release + hide {obj}")
        _state["attached_joint_path"] = None
        _state["attached_obj_path"] = None
    # mode ≈ 0 → no-op

    db.outputs.execOut = og.ExecutionAttributeState.ENABLED
    return True
'''


# Dev cheat — 로버 articulation 의 절대 world pose 를 in-graph 로 읽어
# /ground_truth/odom 으로 발행. ScriptNode 가 stage traverse 로 articulation
# root 를 찾아 LocalToWorldTransform 계산. terrain 무관(자기 path 의존 X).
GT_SCRIPT = '''
from pxr import UsdGeom
import omni.usd
import omni.graph.core as og


def setup(db):
    pass


def compute(db):
    stage = omni.usd.get_context().get_stage()
    artic = None
    # articulation root prim 찾기 (PhysicsArticulationRootAPI 있는 첫 prim)
    for prim in stage.Traverse():
        if prim.HasAPI("PhysicsArticulationRootAPI"):
            artic = prim
            break
    # fallback: m0609/base_link 이름 패턴
    if artic is None:
        for prim in stage.Traverse():
            if prim.GetName() == "base_link" and "m0609" in str(prim.GetPath()):
                artic = prim
                break
    if artic is None:
        return False
    cache = UsdGeom.XformCache()
    M = cache.GetLocalToWorldTransform(artic)
    t = M.ExtractTranslation()
    q = M.ExtractRotationQuat()
    db.outputs.position = [float(t[0]), float(t[1]), float(t[2])]
    qi = q.GetImaginary()
    # ROS2 quaternion 순서: [x, y, z, w]
    db.outputs.orientation = [float(qi[0]), float(qi[1]), float(qi[2]),
                              float(q.GetReal())]
    db.outputs.execOut = og.ExecutionAttributeState.ENABLED
    return True
'''


def _set_targets(node_path: str, input_name: str, target_path: str) -> None:
    """OmniGraph 노드의 target(relationship) 입력 설정.

    relationship 은 USD reference 시 경로가 remap 되므로, terrain 에 v3 를
    reference 해도 /Root → /World/Rover 로 자동 정합된다 (string 입력은 안 됨).
    """
    stage = omni.usd.get_context().get_stage()
    try:
        from isaacsim.core.utils.prims import set_targets
        set_targets(prim=stage.GetPrimAtPath(node_path),
                    attribute=input_name, target_prim_paths=[target_path])
        return
    except Exception:
        pass
    prim = stage.GetPrimAtPath(node_path)
    rel = prim.GetRelationship(input_name)
    if not rel:
        rel = prim.CreateRelationship(input_name)
    rel.SetTargets([Sdf.Path(target_path)])


def _create_sun_camera(stage) -> None:
    """Body 상단에 +z 향하는 sun 추적 카메라 prim 생성.

    UsdGeom.Camera 는 mesh/collision 가 없어 기구 충돌 발생 0. visibility=invisible
    로 viewport 시각 흔적 제거(사용자 요청). flatten 전에 author 하므로 v3 USD 에
    그대로 baked 된다.
    """
    cam = UsdGeom.Camera.Define(stage, SUN_CAM)
    xf = UsdGeom.Xformable(cam)
    # translate
    xf.AddTranslateOp().Set(Gf.Vec3d(*SUN_CAM_XYZ))
    # rotate — X 축 -90° 회전: 카메라 -z(시선) 가 world +z(하늘) 향함
    xf.AddRotateXOp().Set(-90.0)
    # 카메라 광학 — sun blob 잡기에 충분
    cam.GetFocalLengthAttr().Set(24.0)
    cam.GetHorizontalApertureAttr().Set(20.955)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 100000.0))
    # viewport 시각 흔적 제거 (편법) — ROS 카메라 캡쳐와 무관
    UsdGeom.Imageable(cam.GetPrim()).MakeInvisible()
    print(f"[build_v3] sun camera 생성: {SUN_CAM} translate={SUN_CAM_XYZ}")


def _adjust_camera(stage) -> None:
    """Body 카메라(ROVER_CAM) translate 의 x·z 를 ROVER_CAM_XZ 로 조정.

    y 는 기존값 유지. flatten 전에 author 하면 v3 에 그대로 baked 된다.
    """
    cam = stage.GetPrimAtPath(ROVER_CAM)
    if not cam.IsValid():
        print(f"[build_v3] ⚠ 카메라 prim 없음 — 위치 조정 스킵: {ROVER_CAM}")
        return
    for op in UsdGeom.Xformable(cam).GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            cur = op.Get()
            new = type(cur)(ROVER_CAM_XZ[0], cur[1], ROVER_CAM_XZ[1])
            op.Set(new)
            print(f"[build_v3] 카메라 translate {tuple(cur)} → {tuple(new)}")
            return
    print("[build_v3] ⚠ 카메라 translate op 없음 — 위치 조정 스킵")


def _scale_arm_mass(stage, scale: float = 0.5) -> None:
    """m0609 + onrobot_rg2ft link 들의 physics:mass attr 를 scale 배 적용.

    flatten 전에 호출하므로 변경이 v3 USD 에 그대로 baked 된다. 무게중심을
    내려서 rover 안정화 목적. mass=0 인 visual-only prim 은 자동 skip.
    """
    n = 0
    total_before = 0.0
    total_after = 0.0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if "m0609" not in path and "onrobot" not in path:
            continue
        ma = prim.GetAttribute("physics:mass")
        if not ma.IsValid() or not ma.HasAuthoredValue():
            continue
        m = float(ma.Get())
        if m <= 0.0:
            continue
        new_m = m * scale
        ma.Set(new_m)
        total_before += m
        total_after += new_m
        n += 1
    print(f"[build_v3] arm mass scaled x{scale}: {n} links, "
          f"{total_before:.2f}kg -> {total_after:.2f}kg")


def main() -> None:
    if not os.path.isfile(V2):
        print(f"[build_v3] ✗ vehicle_v2.usd 없음: {V2}")
        app.close()
        sys.exit(1)

    # ── 새 stage 에 v2 를 reference (빌드 합성용 — 최종엔 flatten 으로 inline) ──
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    root = stage.DefinePrim("/Root", "Xform")
    root.GetReferences().AddReference(V2)   # 빌드용 reference (flatten 후 사라짐)
    stage.SetDefaultPrim(root)
    for _ in range(80):
        app.update()
    if not stage.GetPrimAtPath(IMU).IsValid():
        print(f"[build_v3] ✗ v2 합성 실패 — IMU prim 없음: {IMU}")
        app.close()
        sys.exit(1)
    print("[build_v3] vehicle_v2 합성 확인 — 센서 prim 존재")

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                # ── 센서 ──
                ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PubImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
                ("PubJoint", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("RPRover", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RPWristRgb", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RPWristDepth", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RPSun", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CamRoverRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CamRoverDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CamRoverInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("CamWristRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CamWristDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CamWristInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("CamSunRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CamSunInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                # ── 주행 (지민 RoverAckermannDrive 포팅) ──
                ("SubTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("Ackermann", "omni.graph.scriptnode.ScriptNode"),
                ("SteerCtrl", "isaacsim.core.nodes.IsaacArticulationController"),
                ("DriveCtrl", "isaacsim.core.nodes.IsaacArticulationController"),
                # ── 팔 제어 (저수준 관절 명령) ──
                ("SubJointCmd", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArmCtrl", "isaacsim.core.nodes.IsaacArticulationController"),
                # ── GT pose 발행 (dev cheat, 졸업 시 이 두 노드만 제거) ──
                ("ReadGtPose", "omni.graph.scriptnode.ScriptNode"),
                ("PubGtOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                # ── Grasp 시뮬레이터 (T2 standalone 의 FixedJoint + invisible 패턴) ──
                ("SubGrasp", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("GraspScript", "omni.graph.scriptnode.ScriptNode"),
            ],
            keys.CREATE_ATTRIBUTES: [
                # ScriptNode 커스텀 포트 — Ackermann 입출력
                ("Ackermann.inputs:linearVelocity", "vectord[3]"),
                ("Ackermann.inputs:angularVelocity", "vectord[3]"),
                ("Ackermann.outputs:steerJointNames", "token[]"),
                ("Ackermann.outputs:driveJointNames", "token[]"),
                ("Ackermann.outputs:steeringAngles", "double[]"),
                ("Ackermann.outputs:wheelVelocities", "double[]"),
                # ScriptNode 커스텀 포트 — GT pose 출력
                ("ReadGtPose.outputs:position", "vectord[3]"),
                ("ReadGtPose.outputs:orientation", "double[4]"),
                # ScriptNode 커스텀 포트 — Grasp 입력 (SubGrasp Twist 와 연결)
                ("GraspScript.inputs:linearVelocity", "vectord[3]"),
                ("GraspScript.inputs:angularVelocity", "vectord[3]"),
            ],
            keys.SET_VALUES: [
                # 센서
                ("ReadIMU.inputs:readGravity", True),
                ("PubImu.inputs:topicName", "/imu/data"),
                ("PubImu.inputs:frameId", "sim_imu"),
                ("PubImu.inputs:publishAngularVelocity", True),
                ("PubImu.inputs:publishLinearAcceleration", True),
                ("PubImu.inputs:publishOrientation", True),
                ("PubJoint.inputs:topicName", "/joint_states_raw"),
                ("RPRover.inputs:width", 640),
                ("RPRover.inputs:height", 480),
                ("RPWristRgb.inputs:width", 640),
                ("RPWristRgb.inputs:height", 480),
                ("RPWristDepth.inputs:width", 640),
                ("RPWristDepth.inputs:height", 480),
                ("CamRoverRgb.inputs:topicName", "/camera/rover/image_raw"),
                ("CamRoverRgb.inputs:type", "rgb"),
                ("CamRoverRgb.inputs:frameId", "rover_camera"),
                ("CamRoverDepth.inputs:topicName", "/camera/rover/depth"),
                ("CamRoverDepth.inputs:type", "depth"),
                ("CamRoverDepth.inputs:frameId", "rover_camera"),
                ("CamRoverInfo.inputs:topicName", "/camera/rover/camera_info"),
                ("CamRoverInfo.inputs:frameId", "rover_camera"),
                ("CamWristRgb.inputs:topicName", "/camera/wrist/image_raw"),
                ("CamWristRgb.inputs:type", "rgb"),
                ("CamWristRgb.inputs:frameId", "wrist_camera"),
                ("CamWristDepth.inputs:topicName", "/camera/wrist/depth"),
                ("CamWristDepth.inputs:type", "depth"),
                ("CamWristDepth.inputs:frameId", "wrist_camera"),
                ("CamWristInfo.inputs:topicName", "/camera/wrist/camera_info"),
                ("CamWristInfo.inputs:frameId", "wrist_camera"),
                # Sun 카메라 (T5 sun_yaw 노드 입력) — 작게(320×240) 충분
                ("RPSun.inputs:width", 320),
                ("RPSun.inputs:height", 240),
                ("CamSunRgb.inputs:topicName", "/camera/sun/image_raw"),
                ("CamSunRgb.inputs:type", "rgb"),
                ("CamSunRgb.inputs:frameId", "sun_camera"),
                ("CamSunInfo.inputs:topicName", "/camera/sun/camera_info"),
                ("CamSunInfo.inputs:frameId", "sun_camera"),
                # 주행
                ("SubTwist.inputs:topicName", "/cmd_vel"),
                ("Ackermann.inputs:script", ACK_SCRIPT),
                # 팔 — arm_executor_node 가 /arm/joint_command 로 관절 위치 지령
                ("SubJointCmd.inputs:topicName", "/arm/joint_command"),
                # GT pose — dev cheat, 졸업 시 제거
                ("ReadGtPose.inputs:script", GT_SCRIPT),
                ("PubGtOdom.inputs:topicName", "/ground_truth/odom"),
                ("PubGtOdom.inputs:odomFrameId", "world"),
                ("PubGtOdom.inputs:chassisFrameId", "base_link"),
                # Grasp 시뮬레이터 — /grasp/command (Twist hijack)
                ("SubGrasp.inputs:topicName", "/grasp/command"),
                ("GraspScript.inputs:script", GRASP_SCRIPT),
            ],
            keys.CONNECT: [
                # 센서 — IMU
                ("OnTick.outputs:tick", "ReadIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut", "PubImu.inputs:execIn"),
                ("ReadIMU.outputs:angVel", "PubImu.inputs:angularVelocity"),
                ("ReadIMU.outputs:linAcc", "PubImu.inputs:linearAcceleration"),
                ("ReadIMU.outputs:orientation", "PubImu.inputs:orientation"),
                ("ReadIMU.outputs:sensorTime", "PubImu.inputs:timeStamp"),
                # 센서 — 관절
                ("OnTick.outputs:tick", "PubJoint.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime",
                 "PubJoint.inputs:timeStamp"),
                # 센서 — 카메라
                ("OnTick.outputs:tick", "RPRover.inputs:execIn"),
                ("OnTick.outputs:tick", "RPWristRgb.inputs:execIn"),
                ("OnTick.outputs:tick", "RPWristDepth.inputs:execIn"),
                ("RPRover.outputs:execOut", "CamRoverRgb.inputs:execIn"),
                ("RPRover.outputs:renderProductPath",
                 "CamRoverRgb.inputs:renderProductPath"),
                ("RPRover.outputs:execOut", "CamRoverDepth.inputs:execIn"),
                ("RPRover.outputs:renderProductPath",
                 "CamRoverDepth.inputs:renderProductPath"),
                ("RPRover.outputs:execOut", "CamRoverInfo.inputs:execIn"),
                ("RPRover.outputs:renderProductPath",
                 "CamRoverInfo.inputs:renderProductPath"),
                ("RPWristRgb.outputs:execOut", "CamWristRgb.inputs:execIn"),
                ("RPWristRgb.outputs:renderProductPath",
                 "CamWristRgb.inputs:renderProductPath"),
                ("RPWristDepth.outputs:execOut", "CamWristDepth.inputs:execIn"),
                ("RPWristDepth.outputs:renderProductPath",
                 "CamWristDepth.inputs:renderProductPath"),
                ("RPWristDepth.outputs:execOut", "CamWristInfo.inputs:execIn"),
                ("RPWristDepth.outputs:renderProductPath",
                 "CamWristInfo.inputs:renderProductPath"),
                # Sun 카메라
                ("OnTick.outputs:tick", "RPSun.inputs:execIn"),
                ("RPSun.outputs:execOut", "CamSunRgb.inputs:execIn"),
                ("RPSun.outputs:renderProductPath",
                 "CamSunRgb.inputs:renderProductPath"),
                ("RPSun.outputs:execOut", "CamSunInfo.inputs:execIn"),
                ("RPSun.outputs:renderProductPath",
                 "CamSunInfo.inputs:renderProductPath"),
                # 주행 — /cmd_vel → Ackermann → 휠 컨트롤러
                ("OnTick.outputs:tick", "SubTwist.inputs:execIn"),
                ("SubTwist.outputs:execOut", "Ackermann.inputs:execIn"),
                ("SubTwist.outputs:linearVelocity",
                 "Ackermann.inputs:linearVelocity"),
                ("SubTwist.outputs:angularVelocity",
                 "Ackermann.inputs:angularVelocity"),
                ("Ackermann.outputs:execOut", "SteerCtrl.inputs:execIn"),
                ("Ackermann.outputs:steerJointNames",
                 "SteerCtrl.inputs:jointNames"),
                ("Ackermann.outputs:steeringAngles",
                 "SteerCtrl.inputs:positionCommand"),
                ("Ackermann.outputs:execOut", "DriveCtrl.inputs:execIn"),
                ("Ackermann.outputs:driveJointNames",
                 "DriveCtrl.inputs:jointNames"),
                ("Ackermann.outputs:wheelVelocities",
                 "DriveCtrl.inputs:velocityCommand"),
                # 팔 — /arm/joint_command → m0609 관절 (관절명·위치는 메시지가 지정)
                ("OnTick.outputs:tick", "SubJointCmd.inputs:execIn"),
                ("SubJointCmd.outputs:execOut", "ArmCtrl.inputs:execIn"),
                ("SubJointCmd.outputs:jointNames", "ArmCtrl.inputs:jointNames"),
                ("SubJointCmd.outputs:positionCommand",
                 "ArmCtrl.inputs:positionCommand"),
                # GT pose — ScriptNode 가 절대 world pose 읽어 PubOdom 으로
                ("OnTick.outputs:tick", "ReadGtPose.inputs:execIn"),
                ("ReadGtPose.outputs:execOut", "PubGtOdom.inputs:execIn"),
                ("ReadGtPose.outputs:position", "PubGtOdom.inputs:position"),
                ("ReadGtPose.outputs:orientation",
                 "PubGtOdom.inputs:orientation"),
                ("ReadSimTime.outputs:simulationTime",
                 "PubGtOdom.inputs:timeStamp"),
                # Grasp — 메시지 도착 시 ScriptNode 호출 (OnTick 불연결: 메시지 받을 때만 act)
                ("OnTick.outputs:tick", "SubGrasp.inputs:execIn"),
                ("SubGrasp.outputs:execOut", "GraspScript.inputs:execIn"),
                ("SubGrasp.outputs:linearVelocity",
                 "GraspScript.inputs:linearVelocity"),
                ("SubGrasp.outputs:angularVelocity",
                 "GraspScript.inputs:angularVelocity"),
            ],
        },
    )
    # relationship(target) 입력 — 센서 prim · articulation 연결
    _set_targets(f"{GRAPH}/ReadIMU", "inputs:imuPrim", IMU)
    _set_targets(f"{GRAPH}/PubJoint", "inputs:targetPrim", ARTIC)
    _set_targets(f"{GRAPH}/RPRover", "inputs:cameraPrim", ROVER_CAM)
    _set_targets(f"{GRAPH}/RPWristRgb", "inputs:cameraPrim", WRIST_RGB)
    _set_targets(f"{GRAPH}/RPWristDepth", "inputs:cameraPrim", WRIST_DEP)
    _set_targets(f"{GRAPH}/RPSun", "inputs:cameraPrim", SUN_CAM)
    _set_targets(f"{GRAPH}/SteerCtrl", "inputs:targetPrim", ARTIC)
    _set_targets(f"{GRAPH}/DriveCtrl", "inputs:targetPrim", ARTIC)
    _set_targets(f"{GRAPH}/ArmCtrl", "inputs:targetPrim", ARTIC)
    print("[build_v3] Action Graph author 완료 — 센서(IMU·joint·카메라) + "
          "주행(/cmd_vel→Ackermann→휠) + 팔(/arm/joint_command→m0609) + "
          "GT(/ground_truth/odom, dev cheat)")

    # 카메라 위치 조정
    _adjust_camera(stage)
    _create_sun_camera(stage)   # ← 새 sun 추적 카메라 (T5 sun_yaw 노드 입력)

    # m0609 + onrobot_rg2ft mass scaling — 사용자 요청으로 무게중심 안정화.
    # 원래 ~28.5kg → scale=0.5 적용 시 ~14.25kg. arm 토크는 자체 제어라 영향 X.
    _scale_arm_mass(stage, scale=0.5)

    # flatten — v2 reference 를 inline 해 자립(standalone) v3 생성
    if os.path.exists(V3):
        os.remove(V3)
    flat = stage.Flatten(addSourceFileComment=False)
    flat.Export(V3)
    print(f"[build_v3] ✓ standalone v3 저장 완료 (flatten): {V3}")
    app.close()


if __name__ == "__main__":
    main()
