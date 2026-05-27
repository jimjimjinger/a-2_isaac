"""vehicle_v3.usd (액션그래프 내장 로버) 를 terrain 에 올려 구동.

v3 는 ROS2 센서 그래프가 USD 에 내장돼 있다 — 이 런처는 그래프를 짜지 않는다.
terrain 로드 + v3 reference + play 만 한다. 팀 누구든 이 패턴(또는 이 스크립트)
으로 v3 를 띄워 자기 노드를 개발하면 된다 — 실물 로봇처럼.

실행: <isaac-python> isaac_sim/scripts/run_vehicle_v3.py [--terrain terrain_00004] [--headless]
"""
import argparse
import json
import math
import os
import shutil
import sys
import tempfile

try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

_p = argparse.ArgumentParser(description="vehicle_v3 (graph-embedded rover) 런처")
_p.add_argument("--terrain", default="terrain_00004")
_p.add_argument("--headless", action="store_true")
_p.add_argument("--no-chase", action="store_true",
                help="Chase cam(/camera/chase) 비활성화 — GPU 부담 측정/비교용. "
                     "비활성 시 Web HUD chase 슬롯은 offline 표시.")
_p.add_argument("--no-overview", action="store_true",
                help="Overview cam(/camera/overview) 비활성화 — 같은 의도.")
_p.add_argument("--rovers", nargs="*", default=[],
                help="다중 rover 모드 — namespace 리스트 (예: rover_1 rover_2). "
                     "비우면 단일 rover (기존 동작, /World/Rover + T5 sun camera + "
                     "chase/overview cam). 다중 모드는 chase/overview/sun_camera 비활성.")
_p.add_argument("--spawn-spacing", type=float, default=0.0,
                help="다중 rover 시 사이 간격 (m). >0 이면 meta spawn_locations "
                     "무시하고 spawn[0] 기준 X 방향으로 N 미터씩 배치 (A* 회피 검증용).")
_a, _ = _p.parse_known_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": _a.headless})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
app.update()

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC_SIM = os.path.dirname(HERE)
WORLD = f"{ISAAC_SIM}/worlds/{_a.terrain}.usd"
V3 = f"{ISAAC_SIM}/assets/vehicle/vehicle_v3.usd"
TERRAIN_DIR = f"{ISAAC_SIM}/assets/generated_terrains/{_a.terrain}"
ROVER_PRIM = "/World/Rover"
# rover 의 실시간 transform 은 articulation rigid body 인 Body prim 에서만
# 갱신됨. ROVER_PRIM (reference xform root) 은 spawn 시점 그대로 고정.
# build_vehicle_v3.ROVER_CAM 의 경로 패턴 차용: /Root/Vehicle/rover/Body.
ROVER_BODY_PRIM = ROVER_PRIM + "/Vehicle/rover/Body"

# T5 localization (PR #11): vehicle_v3 USD 에 baked 된 좁은 24mm sun camera 를
# runtime 에 wide lens 로 재설정 + 시각용 sun disk/light 를 World 에 추가해
# sun_yaw bearing 추출이 안정적으로 동작하도록 한다.
SUN_CAMERA_PRIM = f"{ROVER_PRIM}/Vehicle/rover/Body/SunCamera"
WORLD_SUN_YAW = 0.929
WORLD_SUN_ELEVATION = math.radians(55.0)
VISUAL_SUN_PRIM = "/World/VisualSun"
VISUAL_SUN_LIGHT = "/World/VisualSunLight"

# World-fixed overview camera — terrain 전체 부감 (50m 아레나 한눈에). vehicle_v3
# 와 무관하게 World prim 에 별도 author. UI 메인 화면 overview 슬롯이 구독.
OVERVIEW_CAM = "/World/OverviewCam"
OVERVIEW_GRAPH = "/World/OverviewGraph"
# 위치: 맵 중심 (0,0) 남쪽 -y 에서 비스듬히 부감. stage 가 Z-up 이라 카메라
# default forward(-z, 즉 아래)에 X 축 +α° 회전을 적용하면 시선이
# (0, sin α, -cos α). 카메라 (0, -45, 55) 에서 (0, 0, 0) 향하려면
# 벡터 = (0, 45, -55), 정규화 = (0, 0.633, -0.774) → α = asin(0.633) ≈ 39.3°.
# 광각 14mm 로 50m 아레나 전체 프레임에 들어옴.
OVERVIEW_XYZ = (0.0, -45.0, 55.0)
OVERVIEW_PITCH_DEG = 39.3
# 16:9 해상도 — Web HUD overview slot 의 가로:세로 비율에 맞춰 letterbox 제거.
OVERVIEW_W = 960
OVERVIEW_H = 540
OVERVIEW_TOPIC = "/camera/overview/image_raw"
OVERVIEW_INFO_TOPIC = "/camera/overview/camera_info"

# Chase camera — rover 뒤쪽 위에서 3인칭 TPS 풍 관망. /camera/chase/image_raw.
# rover transform 을 매 step Python 으로 폴링해서 cam 위치/시선 업데이트.
# vehicle_v3 reference 와 무관한 World prim 으로 author.
CHASE_CAM = "/World/ChaseCam"
CHASE_GRAPH = "/World/ChaseGraph"
CHASE_W = 960
CHASE_H = 540
CHASE_TOPIC = "/camera/chase/image_raw"
CHASE_INFO_TOPIC = "/camera/chase/camera_info"
# rover local frame 기준 chase cam offset (x=뒤, z=위, y=좌우 평행). rover
# 의 forward 가 local +x (Ackermann 주행 + spawn yaw 1.576 검증). 뒤쪽 6m,
# 위쪽 3m, 옆 0 (정후방). target 은 rover origin 보다 약간 위(+1.0) 로 잡아
# 부감 효과.
CHASE_OFFSET_LOCAL = (-6.0, 0.0, 3.0)
CHASE_TARGET_Z_BIAS = 1.0


def _add_overview_camera(stage) -> str:
    """World prim 에 부감 카메라 prim 추가. visibility=invisible 로 viewport
    오버레이 흔적 제거 — ROS 캡쳐와는 무관."""
    cam = UsdGeom.Camera.Define(stage, OVERVIEW_CAM)
    xf = UsdGeom.Xformable(cam)
    xf.AddTranslateOp().Set(Gf.Vec3d(*OVERVIEW_XYZ))
    xf.AddRotateXOp().Set(float(OVERVIEW_PITCH_DEG))
    cam.GetFocalLengthAttr().Set(14.0)
    # 16:9 aperture — 가로 21mm × 세로 11.8mm 비율 유지로 영상 stretching 방지.
    cam.GetHorizontalApertureAttr().Set(20.955)
    cam.GetVerticalApertureAttr().Set(20.955 * (OVERVIEW_H / OVERVIEW_W))
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 100000.0))
    UsdGeom.Imageable(cam.GetPrim()).MakeInvisible()
    print(f"[run_v3] overview camera 생성: {OVERVIEW_CAM} "
          f"@ {OVERVIEW_XYZ} pitch={OVERVIEW_PITCH_DEG}°")
    return OVERVIEW_CAM


def _create_overview_graph() -> None:
    """별도 OmniGraph — RP + ROS2CameraHelper + CameraInfoHelper.
    vehicle_v3 내장 그래프와 분리해 reference 충돌 회피.
    """
    import omni.graph.core as og
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": OVERVIEW_GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("RPOverview", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CamOverviewRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CamOverviewInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("RPOverview.inputs:width", OVERVIEW_W),
                ("RPOverview.inputs:height", OVERVIEW_H),
                ("CamOverviewRgb.inputs:topicName", OVERVIEW_TOPIC),
                ("CamOverviewRgb.inputs:type", "rgb"),
                ("CamOverviewRgb.inputs:frameId", "overview_camera"),
                ("CamOverviewInfo.inputs:topicName", OVERVIEW_INFO_TOPIC),
                ("CamOverviewInfo.inputs:frameId", "overview_camera"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "RPOverview.inputs:execIn"),
                ("RPOverview.outputs:execOut", "CamOverviewRgb.inputs:execIn"),
                ("RPOverview.outputs:renderProductPath",
                 "CamOverviewRgb.inputs:renderProductPath"),
                ("RPOverview.outputs:execOut", "CamOverviewInfo.inputs:execIn"),
                ("RPOverview.outputs:renderProductPath",
                 "CamOverviewInfo.inputs:renderProductPath"),
            ],
        },
    )

    # cameraPrim relationship 연결 — build_vehicle_v3 의 _set_targets 패턴 차용.
    stage = omni.usd.get_context().get_stage()
    try:
        from isaacsim.core.utils.prims import set_targets
        set_targets(
            prim=stage.GetPrimAtPath(f"{OVERVIEW_GRAPH}/RPOverview"),
            attribute="inputs:cameraPrim",
            target_prim_paths=[OVERVIEW_CAM])
    except Exception as e:
        print(f"[run_v3] ⚠ overview cameraPrim 연결 실패: {e}")
        return
    print(f"[run_v3] overview graph 구성 완료 → {OVERVIEW_TOPIC}")


def _add_chase_camera(stage, cam_path: str = CHASE_CAM):
    """rover 뒤에서 따라가는 chase 카메라 prim. 매 step _update_chase_cam 이
    transform op 의 4x4 매트릭스를 갱신해 rover 를 따라간다.
    multi 모드에선 cam_path 로 per-rover 격리 (예: /World/ChaseCam_rover_1)."""
    cam = UsdGeom.Camera.Define(stage, cam_path)
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    transform_op = xf.AddTransformOp()
    cam.GetFocalLengthAttr().Set(20.0)
    cam.GetHorizontalApertureAttr().Set(20.955)
    cam.GetVerticalApertureAttr().Set(20.955 * (CHASE_H / CHASE_W))
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 100000.0))
    UsdGeom.Imageable(cam.GetPrim()).MakeInvisible()
    print(f"[run_v3] chase camera 생성: {cam_path} offset_local={CHASE_OFFSET_LOCAL}")
    return transform_op


def _create_chase_graph(graph_path: str = CHASE_GRAPH,
                        cam_path: str = CHASE_CAM,
                        topic: str = CHASE_TOPIC,
                        info_topic: str = CHASE_INFO_TOPIC,
                        frame_id: str = "chase_camera") -> None:
    """vehicle_v3 / overview 와 분리된 별도 OmniGraph (RP + CamHelper).
    multi 모드에선 graph_path/topic 을 ns 별로 분리 (per-rover chase stream).
    """
    import omni.graph.core as og
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("RPChase", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CamChaseRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CamChaseInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("RPChase.inputs:width", CHASE_W),
                ("RPChase.inputs:height", CHASE_H),
                ("CamChaseRgb.inputs:topicName", topic),
                ("CamChaseRgb.inputs:type", "rgb"),
                ("CamChaseRgb.inputs:frameId", frame_id),
                ("CamChaseInfo.inputs:topicName", info_topic),
                ("CamChaseInfo.inputs:frameId", frame_id),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "RPChase.inputs:execIn"),
                ("RPChase.outputs:execOut", "CamChaseRgb.inputs:execIn"),
                ("RPChase.outputs:renderProductPath",
                 "CamChaseRgb.inputs:renderProductPath"),
                ("RPChase.outputs:execOut", "CamChaseInfo.inputs:execIn"),
                ("RPChase.outputs:renderProductPath",
                 "CamChaseInfo.inputs:renderProductPath"),
            ],
        },
    )
    stage = omni.usd.get_context().get_stage()
    try:
        from isaacsim.core.utils.prims import set_targets
        set_targets(
            prim=stage.GetPrimAtPath(f"{graph_path}/RPChase"),
            attribute="inputs:cameraPrim",
            target_prim_paths=[cam_path])
    except Exception as e:
        print(f"[run_v3] ⚠ chase cameraPrim 연결 실패: {e}")
        return
    print(f"[run_v3] chase graph 구성 완료 → {topic}")


def _find_articulation_root(stage, rover_root: str = ""):
    """PhysicsArticulationRootAPI 가 있는 첫 prim.

    rover_root 가 주어지면 그 prim 의 자손 안에서만 검색 (multi-rover 격리).
    없으면 stage 전체 traverse (단일 모드).
    """
    if rover_root:
        root = stage.GetPrimAtPath(rover_root)
        if not root.IsValid():
            return None
        for prim in Usd.PrimRange(root):
            if prim.HasAPI("PhysicsArticulationRootAPI"):
                return prim
        return None
    for prim in stage.Traverse():
        if prim.HasAPI("PhysicsArticulationRootAPI"):
            return prim
    for prim in stage.Traverse():
        if prim.GetName() == "base_link" and "m0609" in str(prim.GetPath()):
            return prim
    return None


def _update_chase_cam(stage, transform_op, state: dict) -> None:
    """rover 의 world transform 을 읽어 chase cam 위치/시선을 매 step 갱신.

    rover local +x = forward (Ackermann 주행 + spawn yaw=π/2 검증). chase 는
    rover 뒤(-x), 위(+z) 에서 rover 머리 위쪽(+target_z_bias) 을 바라봄.
    USD Camera convention: -Z forward, +Y up, +X right (camera local).

    GT_SCRIPT 와 동일 패턴: articulation root 를 traverse 로 찾고 매 호출마다
    새 XformCache 로 stale 방지. ROVER_PRIM (reference xform root) 은 spawn
    시점 고정이라 사용 불가.
    """
    artic = state.get("artic")
    if artic is None or not artic.IsValid():
        # state 에 "rover_root" 가 있으면 그 안에서만 검색 (multi-rover).
        rover_root = state.get("rover_root", "")
        artic = _find_articulation_root(stage, rover_root)
        state["artic"] = artic
        if artic is None:
            return
    cache = UsdGeom.XformCache()
    m = cache.GetLocalToWorldTransform(artic)
    # ExtractTranslation / row vectors 로 rover 의 world frame 축들.
    rover_pos = m.ExtractTranslation()
    # row 0 = rover local +x in world, row 1 = +y, row 2 = +z.
    rx = Gf.Vec3d(m[0][0], m[0][1], m[0][2])
    ry = Gf.Vec3d(m[1][0], m[1][1], m[1][2])
    rz = Gf.Vec3d(m[2][0], m[2][1], m[2][2])
    ox, oy, oz = CHASE_OFFSET_LOCAL
    eye = Gf.Vec3d(rover_pos) + rx * ox + ry * oy + rz * oz
    target = Gf.Vec3d(rover_pos) + rz * CHASE_TARGET_Z_BIAS

    # look-at 매트릭스 — camera local 의 -z(forward) 가 (target - eye) 향함.
    fwd = (target - eye)
    fwd_len = fwd.GetLength()
    if fwd_len < 1e-6:
        return
    fwd = fwd / fwd_len
    world_up = Gf.Vec3d(0, 0, 1)
    right = Gf.Cross(fwd, world_up)
    right_len = right.GetLength()
    if right_len < 1e-6:
        right = Gf.Vec3d(1, 0, 0)
    else:
        right = right / right_len
    up = Gf.Cross(right, fwd)

    # USD Matrix4d (row-major). camera->world transform:
    #   col 0 = right, col 1 = up, col 2 = -forward, col 3 = eye.
    cam_xform = Gf.Matrix4d(
        right[0], right[1], right[2], 0.0,
        up[0],    up[1],    up[2],    0.0,
        -fwd[0],  -fwd[1],  -fwd[2],  0.0,
        eye[0],   eye[1],   eye[2],   1.0,
    )
    transform_op.Set(cam_xform)


def configure_sun_camera(stage) -> None:
    """Widen the baked v3 sun camera at runtime.

    vehicle_v3.usd was built with a narrow 24 mm lens. The visible sun is a
    bearing target, so it needs the wide view used in the proven feature/local
    setup; otherwise the detector may lock onto bright terrain instead.
    """
    prim = stage.GetPrimAtPath(SUN_CAMERA_PRIM)
    if not prim or not prim.IsValid():
        print(f"[run_v3] ⚠ sun camera 없음: {SUN_CAMERA_PRIM}")
        return
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(0.10, 0.0, 2.50))
    # Match the proven v2 sun-camera mount: optical axis points upward from
    # the rover body, with image axes stable for bearing extraction.
    xform.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, -90.0))
    cam = UsdGeom.Camera(prim)
    cam.GetFocalLengthAttr().Set(6.0)
    cam.GetHorizontalApertureAttr().Set(20.955)
    cam.GetVerticalApertureAttr().Set(15.2908)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10000.0))
    print(f"[run_v3] sun camera wide lens applied: {SUN_CAMERA_PRIM}")


def create_visual_sun(stage) -> None:
    """Create a visible sun disk/sphere for the upward sun camera.

    Isaac DistantLight illuminates the scene but is not a visible object in the
    camera image. T5 sun_yaw needs an actual bright blob, so we add one far
    above the map at the same azimuth as the localization world_sun_yaw.
    """
    distance = 2000.0
    radius = 100.0

    horizontal_distance = distance * math.cos(WORLD_SUN_ELEVATION)
    x = math.cos(WORLD_SUN_YAW) * horizontal_distance
    y = math.sin(WORLD_SUN_YAW) * horizontal_distance
    z = distance * math.sin(WORLD_SUN_ELEVATION)

    sun = UsdGeom.Sphere.Define(stage, Sdf.Path(VISUAL_SUN_PRIM))
    sun.GetRadiusAttr().Set(radius)
    sun.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.86, 0.22)])
    UsdGeom.XformCommonAPI(sun.GetPrim()).SetTranslate(Gf.Vec3d(x, y, z))

    mat = UsdShade.Material.Define(stage, Sdf.Path("/World/VisualSunMaterial"))
    shader = UsdShade.Shader.Define(
        stage,
        Sdf.Path("/World/VisualSunMaterial/PreviewSurface"),
    )
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(1.0, 0.82, 0.12)
    )
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(1.0, 0.82, 0.12)
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(sun.GetPrim()).Bind(mat)

    light = UsdLux.SphereLight.Define(stage, Sdf.Path(VISUAL_SUN_LIGHT))
    light.CreateRadiusAttr(radius)
    light.CreateIntensityAttr(250000.0)
    light.CreateExposureAttr(0.0)
    light.CreateColorAttr(Gf.Vec3f(1.0, 0.86, 0.18))
    UsdGeom.XformCommonAPI(light.GetPrim()).SetTranslate(Gf.Vec3d(x, y, z))

    print(
        f"[run_v3] visual sun: {VISUAL_SUN_PRIM} "
        f"pos=({x:.1f}, {y:.1f}, {z:.1f}), "
        f"yaw={WORLD_SUN_YAW:.3f} rad, "
        f"elev={math.degrees(WORLD_SUN_ELEVATION):.1f} deg, "
        f"radius={radius:.1f}"
    )



# ─── 단일 rover (기존 동작 호환) ──────────────────────────────────────
DEFAULT_ROVER_PRIM = "/World/Rover"


# ─── ScriptNode 소스 — rover root 를 hardcode 한 버전 ────────────────
# build_vehicle_v3.py 의 GT_SCRIPT / GRASP_SCRIPT 와 동일 동작이지만 stage
# 전역 traverse 대신 rover root prim 아래만 검색 → 두 rover 가 같은 stage
# 에 있어도 서로 간섭하지 않음. {ROOT}, {JOINT} 는 rover 별로 다르게 박힘.

GT_SCRIPT_NSAWARE_TPL = '''
from pxr import UsdGeom, Usd
import omni.usd
import omni.graph.core as og


def setup(db):
    pass


def _find_rover_root(node_path):
    """ScriptNode prim path 에서 rover root 추출.
    예: /World/Rover_1/ActionGraph/ReadGtPose → /World/Rover_1
    """
    parts = node_path.split("/")
    if len(parts) < 3:
        return None
    return "/" + parts[1] + "/" + parts[2]


def compute(db):
    stage = omni.usd.get_context().get_stage()
    # 자신 prim path 에서 rover root 동적 추출 — script content 가 공유돼도 인스턴스별 다른 path
    my_path = ""
    try:
        my_path = db.node.get_prim_path()
    except Exception:
        try:
            my_path = str(db.node.get_prim())
        except Exception:
            return False
    rover_root = _find_rover_root(my_path)
    if not rover_root:
        return False
    root_prim = stage.GetPrimAtPath(rover_root)
    if not root_prim or not root_prim.IsValid():
        return False
    artic = None
    for prim in Usd.PrimRange(root_prim):
        if prim.HasAPI("PhysicsArticulationRootAPI"):
            artic = prim
            break
    if artic is None:
        for prim in Usd.PrimRange(root_prim):
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
    db.outputs.orientation = [float(qi[0]), float(qi[1]), float(qi[2]),
                              float(q.GetReal())]
    db.outputs.execOut = og.ExecutionAttributeState.ENABLED
    return True
'''


GRASP_SCRIPT_NSAWARE_TPL = '''
from pxr import UsdPhysics, UsdGeom, Gf, Sdf, Usd
import omni.usd
import omni.graph.core as og
import math

_state = {"attached_joint_path": None, "attached_obj_path": None,
          "gripper_link_path": None}

ROVER_ROOT = "__ROVER_ROOT__"
GRASP_JOINT_PATH = "__GRASP_JOINT_PATH__"
GRIPPER_LINK_NAME = "right_inner_finger"
SEARCH_RADIUS = 2.5  # m  WRIST_T_LINK6 best-effort 누적 IK 오차 흡수용 cheat (2026-05-27)


def _component(v, i):
    try:
        return float(v[i])
    except Exception:
        return 0.0


def _find_gripper_link(stage):
    root = stage.GetPrimAtPath(ROVER_ROOT)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetName() == GRIPPER_LINK_NAME and "onrobot" in str(prim.GetPath()):
            return str(prim.GetPath())
    return None


def _find_arm_base(stage):
    """로봇팔 articulation root (m0609 base_link) — snap distance 기준점.
    IK 자세 폭주로 finger 가 텔레포트해도 base_link 는 안정.
    """
    root = stage.GetPrimAtPath(ROVER_ROOT)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.HasAPI("PhysicsArticulationRootAPI"):
            return str(prim.GetPath())
    for prim in Usd.PrimRange(root):
        if prim.GetName() == "base_link" and "m0609" in str(prim.GetPath()):
            return str(prim.GetPath())
    return None


def _find_nearest_mineral(stage, tx, ty):
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
    if stage.GetPrimAtPath(GRASP_JOINT_PATH).IsValid():
        stage.RemovePrim(GRASP_JOINT_PATH)
    link_prim = stage.GetPrimAtPath(link_path)
    obj_prim = stage.GetPrimAtPath(obj_path)
    if not link_prim.IsValid() or not obj_prim.IsValid():
        return False
    joint = UsdPhysics.FixedJoint.Define(stage, GRASP_JOINT_PATH)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(obj_path)])
    # Body0 = right_inner_finger frame origin (knuckle 회전축) 에 mineral attach.
    # 시연용으로 lever arm = 0 유지 → cargo basket 으로 swing 시 mineral inertia
    # 가 회전축 위에 있어 안정.
    # 졸업 과제 (list_to_fix): 우리 main 의 vehicle_v3.usd 의 right_inner_finger
    # frame 축 정의 검사 + finger tip 까지 정확한 offset 측정해 시각 개선.
    # T2 제안값 (0.06, 0, 0.14) 시도 → 좌표계 mismatch 의심 + cargo 회전 시
    # lever arm 효과로 oscillation 관찰 (2026-05-26 시연 검증). 원복.
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
    if not obj_path:
        return
    prim = stage.GetPrimAtPath(obj_path)
    if not prim or not prim.IsValid():
        return
    for child in Usd.PrimRange(prim):
        if child.HasAPI(UsdPhysics.CollisionAPI):
            ca = UsdPhysics.CollisionAPI(child)
            enabled_attr = ca.GetCollisionEnabledAttr()
            if not enabled_attr:
                enabled_attr = ca.CreateCollisionEnabledAttr()
            enabled_attr.Set(bool(enabled))


def _hide(stage, obj_path):
    prim = stage.GetPrimAtPath(obj_path)
    if prim and prim.IsValid():
        imageable = UsdGeom.Imageable(prim)
        imageable.MakeInvisible()


def setup(db):
    pass


def compute(db):
    lin = db.inputs.linearVelocity
    ang = db.inputs.angularVelocity
    mode = _component(ang, 0)  # +1 pickup, -1 release
    if mode > 0.5:
        stage = omni.usd.get_context().get_stage()
        gripper_path = _state.get("gripper_link_path") or _find_gripper_link(stage)
        if not gripper_path:
            print("[grasp] gripper link 못 찾음")
            return False
        _state["gripper_link_path"] = gripper_path
        # arm_base (articulation root) — snap 거리 측정 기준. finger 와 달리
        # IK 자세 폭주에 영향 안 받음 (rover body 와 함께 stable).
        arm_base_path = _state.get("arm_base_path") or _find_arm_base(stage)
        if not arm_base_path:
            print("[grasp] arm_base 못 찾음 — m0609 articulation root 없음")
            return False
        _state["arm_base_path"] = arm_base_path
        # supervisor 가 보낸 추정 mineral 좌표 (참고용 log).
        req_x = _component(lin, 0)
        req_y = _component(lin, 1)
        # ── snap 판정: arm_base GT 기준 nearest mineral ──────────────────
        # finger world transform 은 PhysX articulation 폭주 시 텔레포트 가능
        # (2026-05-27 rover_1 시연에서 gripper GT (19.78,20.95) 폭주 관찰).
        # arm_base 는 articulation root 라 stable. SEARCH_RADIUS 2.5m 안에
        # mineral 이 들어오면 nearest 를 그리퍼 finger 로 snap (attach 대상은
        # 그리퍼 그대로 — 시각적으로 그리퍼가 광물 들고 있는 모습 유지).
        arm_prim = stage.GetPrimAtPath(arm_base_path)
        bx, by = req_x, req_y
        if arm_prim and arm_prim.IsValid():
            cache = UsdGeom.XformCache()
            M = cache.GetLocalToWorldTransform(arm_prim)
            apos = M.ExtractTranslation()
            bx = float(apos[0])
            by = float(apos[1])
        mineral_path, dist = _find_nearest_mineral(stage, bx, by)
        if not mineral_path:
            print(f"[grasp] pickup ignored — no mineral near arm_base "
                  f"({bx:.2f},{by:.2f}) within {SEARCH_RADIUS}m "
                  f"(requested=({req_x:.2f},{req_y:.2f}))")
            return True
        if _attach(stage, gripper_path, mineral_path):
            _state["attached_joint_path"] = GRASP_JOINT_PATH
            _state["attached_obj_path"] = mineral_path
            _set_mineral_collision(stage, mineral_path, False)
            print(f"[grasp] pickup OK — attached {mineral_path} to {gripper_path} "
                  f"(arm_base GT=({bx:.2f},{by:.2f}), "
                  f"requested=({req_x:.2f},{req_y:.2f}), "
                  f"target dist {dist:.2f}m, snapped, collision off)")
        return True
    elif mode < -0.5:
        stage = omni.usd.get_context().get_stage()
        obj = _state.get("attached_obj_path")
        _detach(stage)
        if obj:
            _hide(stage, obj)
            print(f"[grasp] release + hide {obj}")
        _state["attached_joint_path"] = None
        _state["attached_obj_path"] = None
        return True
    return True
'''


# ─── helpers ─────────────────────────────────────────────────────────

def _ns_to_prim_name(ns: str) -> str:
    """네임스페이스 string 을 prim 이름으로 정규화 (예: 'rover_1' → 'Rover_1')."""
    s = ns.strip("/").strip()
    if not s:
        return "Rover"
    # Sdf path component 는 영문/숫자/_ 만 안전
    safe = "".join(c if (c.isalnum() or c == "_") else "_" for c in s)
    return safe[:1].upper() + safe[1:]


def _spawn_for(spots, idx, fallback=(0.0, 0.0, 1.0)):
    if not spots or idx >= len(spots):
        return fallback
    s = spots[idx]
    return (float(s["x"]), float(s["y"]), float(s["z"]) + 0.3)


def _close_spawn_for(spots, idx, spacing):
    """검증용 — spawn_locations[0] 기준 X 방향 spacing m 씩 떨어진 자리.

    spawn_locations[0] 의 z 사용 (해당 지점 terrain 높이) 으로 wrong-z 묻힘 방지.
    """
    base = (0.0, 0.0, 1.0)
    if spots:
        s0 = spots[0]
        base = (float(s0["x"]), float(s0["y"]), float(s0["z"]) + 0.3)
    return (base[0] + idx * spacing, base[1], base[2])


def _load_rover(stage, world, prim_path: str, spawn,
                usd_source: str = V3) -> None:
    """vehicle_v3 USD 를 prim_path 에 reference + 위치 설정.

    usd_source 가 V3 가 아니면 (다중 rover 시 per-rover copy) 별도 파일이라
    USD prototype 공유가 일어나지 않아 OmniGraph 데이터 분리 자연 보장.
    """
    add_reference_to_stage(usd_path=usd_source, prim_path=prim_path)
    prim = stage.GetPrimAtPath(prim_path)
    # 안전망 — 혹시라도 instance 화 됐으면 강제 해제
    try:
        prim.SetInstanceable(False)
    except Exception:
        pass
    xf = UsdGeom.Xformable(prim)
    top = None
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            top = op
            break
    if top is None:
        top = xf.AddTranslateOp()
    top.Set(Gf.Vec3d(*spawn))
    print(f"[run_v3] reference {os.path.basename(usd_source)} @ {prim_path}  spawn={spawn}")


def _per_rover_usd_copies(namespaces) -> dict:
    """rover 별 vehicle_v3 USD 복사본 경로 dict 반환 {ns: usd_path}.

    같은 파일을 여러 prim path 에 reference 하면 USD/OmniGraph 가 prototype
    공유로 데이터가 섞이는 문제 차단 — 각 rover 가 자기 전용 USD 파일을 봄.
    """
    out_dir = tempfile.mkdtemp(prefix="a2_isaac_rovers_")
    copies = {}
    for ns in namespaces:
        dst = os.path.join(out_dir, f"vehicle_v3_{ns}.usd")
        shutil.copyfile(V3, dst)
        copies[ns] = dst
        print(f"[run_v3] per-rover USD copy: {V3.split('/')[-1]} → {dst}")
    return copies


def _patch_topic_names(stage, rover_root: str, ns: str) -> int:
    """rover_root 아래 모든 OmniGraph 노드의 *:topicName 속성에 /{ns} prefix 추가.

    이미 prefix 가 박혀 있으면 (재실행 등) skip. ns 가 비어 있으면 아무것도 안 함.
    return: 변경한 attribute 개수
    """
    if not ns:
        return 0
    ns_norm = ns.strip("/").strip()
    if not ns_norm:
        return 0
    prefix = f"/{ns_norm}"
    root = stage.GetPrimAtPath(rover_root)
    if not root.IsValid():
        return 0
    patched = 0
    for prim in Usd.PrimRange(root):
        for attr in prim.GetAttributes():
            name = attr.GetName()
            if not (name.endswith(":topicName") or name == "topicName"):
                continue
            cur = attr.Get()
            if not isinstance(cur, str) or not cur:
                continue
            # 이미 prefix 적용돼 있으면 skip
            if cur.startswith(prefix + "/") or cur == prefix:
                continue
            # leading slash 보장
            if not cur.startswith("/"):
                cur = "/" + cur
            new_topic = prefix + cur
            attr.Set(new_topic)
            patched += 1
    return patched


def _patch_script_nodes(stage, rover_root: str, ns: str) -> int:
    """rover_root 아래 GT/GRASP ScriptNode 의 inputs:script 를 rover-scoped 버전으로 교체.

    return: 패치한 ScriptNode 개수
    """
    ns_norm = (ns.strip("/").strip() or "rover")
    joint_path = f"/World/grip_fixed_joint_{ns_norm}"
    root = stage.GetPrimAtPath(rover_root)
    if not root.IsValid():
        return 0
    patched = 0
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() != "OmniGraphNode":
            continue
        name = prim.GetName()
        script_attr = prim.GetAttribute("inputs:script")
        if not script_attr or not script_attr.IsValid():
            continue
        if name == "ReadGtPose":
            # GT_SCRIPT 는 self-introspection (placeholder 없음). 모든 rover 동일 content,
            # 런타임에 자기 prim path 로 rover root 동적 결정.
            new_src = GT_SCRIPT_NSAWARE_TPL
            script_attr.Set(new_src)
            check = script_attr.Get()
            ok = (isinstance(check, str) and "_find_rover_root" in check)
            print(f"[run_v3]     {script_attr.GetPath()} ← ReadGtPose ({len(new_src)} chars, self-introspect={'OK' if ok else 'FAIL'})")
            patched += 1
        elif name == "GraspScript":
            new_src = (GRASP_SCRIPT_NSAWARE_TPL
                       .replace("__ROVER_ROOT__", rover_root)
                       .replace("__GRASP_JOINT_PATH__", joint_path))
            script_attr.Set(new_src)
            check = script_attr.Get()
            ok = (isinstance(check, str) and rover_root in check and joint_path in check)
            print(f"[run_v3]     {script_attr.GetPath()} ← GraspScript ({len(new_src)} chars, ROVER_ROOT+JOINT={'OK' if ok else 'FAIL'})")
            patched += 1
    return patched


def main() -> None:
    for f in (WORLD, V3):
        if not os.path.isfile(f):
            print(f"[run_v3] ✗ 파일 없음: {f}")
            app.close()
            sys.exit(1)

    world = World(stage_units_in_meters=1.0)
    add_reference_to_stage(usd_path=WORLD, prim_path="/World/MarsScene")
    print(f"[run_v3] 씬 로드: {WORLD}")

    # spawn 좌표 (terrain meta.json) — 단일/다중 모드 공통
    spots = []
    meta = os.path.join(TERRAIN_DIR, "meta.json")
    if os.path.isfile(meta):
        with open(meta) as f:
            spots = json.load(f).get("spawn_locations") or []

    stage = omni.usd.get_context().get_stage()
    chase_xform_op = None
    # multi 모드의 per-rover chase cam — main loop 에서 모두 갱신.
    # 각 항목: {"op": Sdf TransformOp, "state": dict(rover_root=...)}.
    multi_chase_cams: list = []

    if _a.rovers:
        # ── 다중 rover (T2 패턴) ──
        # sun_camera/visual_sun 은 단일 검증만 마쳤어 multi 미가동.
        # chase cam 은 per-rover 로 생성 (Web HUD active rover 카메라 슬롯용).
        usd_copies = _per_rover_usd_copies(_a.rovers)
        spacing = float(_a.spawn_spacing)
        if spacing > 0.0:
            print(f"[run_v3] close-spawn 모드: 간격 {spacing:.2f}m (A* 회피 검증)")
        for i, ns in enumerate(_a.rovers):
            prim_path = f"/World/{_ns_to_prim_name(ns)}"
            if spacing > 0.0:
                spawn_xyz = _close_spawn_for(spots, i, spacing)
            else:
                spawn_xyz = _spawn_for(spots, i, fallback=(i * 3.0, 0.0, 1.0))
            _load_rover(stage, world, prim_path, spawn_xyz,
                        usd_source=usd_copies[ns])
            for _ in range(5):
                app.update()
            n_topics = _patch_topic_names(stage, prim_path, ns)
            n_scripts = _patch_script_nodes(stage, prim_path, ns)
            print(f"[run_v3]   patched {n_topics} topicName attrs, "
                  f"{n_scripts} ScriptNode(s) → namespace /{ns}")
            # per-rover chase cam + OmniGraph (--no-chase 시 skip)
            if not _a.no_chase:
                ns_norm = ns.strip("/").strip()
                cam_path = f"/World/ChaseCam_{ns_norm}"
                graph_path = f"/World/ChaseGraph_{ns_norm}"
                topic = f"/{ns_norm}/camera/chase/image_raw"
                info_topic = f"/{ns_norm}/camera/chase/camera_info"
                op = _add_chase_camera(stage, cam_path=cam_path)
                _create_chase_graph(graph_path=graph_path, cam_path=cam_path,
                                    topic=topic, info_topic=info_topic,
                                    frame_id=f"{ns_norm}_chase_camera")
                multi_chase_cams.append({
                    "op": op,
                    "state": {"rover_root": prim_path},
                })
    else:
        # ── 단일 rover (기존 동작) ──
        spawn = (0.0, 0.0, 1.0)
        spawn_yaw = 0.0
        if spots:
            s = spots[0]
            spawn = (float(s["x"]), float(s["y"]), float(s["z"]) + 0.3)
            spawn_yaw = float(s.get("yaw", 0.0))

        # v3 reference — 그래프가 USD 에 내장돼 있어 그대로 따라온다.
        add_reference_to_stage(usd_path=V3, prim_path=ROVER_PRIM)
        # T5 localization (PR #11): sun camera lens 와이드 + 시각 sun disk 추가.
        configure_sun_camera(stage)
        create_visual_sun(stage)
        xf = UsdGeom.Xformable(stage.GetPrimAtPath(ROVER_PRIM))
        top = None
        rot = None
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                top = op
            elif op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                rot = op
        if top is None:
            top = xf.AddTranslateOp()
        top.Set(Gf.Vec3d(*spawn))
        if abs(spawn_yaw) > 1e-9:
            if rot is None:
                rot = xf.AddRotateXYZOp()
            rot.Set(Gf.Vec3f(0.0, 0.0, math.degrees(spawn_yaw)))
        print(f"[run_v3] vehicle_v3 spawn: {spawn}, "
              f"yaw={math.degrees(spawn_yaw):.1f} deg")

        # World-fixed overview camera + 자체 OmniGraph 추가.
        if not _a.no_overview:
            _add_overview_camera(stage)
            _create_overview_graph()
        else:
            print("[run_v3] overview cam 비활성 (--no-overview)")
        # rover 뒤에서 따라가는 chase 카메라
        if not _a.no_chase:
            chase_xform_op = _add_chase_camera(stage)
            _create_chase_graph()
        else:
            print("[run_v3] chase cam 비활성 (--no-chase)")

    for _ in range(20):
        app.update()
    world.reset()
    world.play()
    if _a.rovers:
        nss = ", ".join(f"/{n}" for n in _a.rovers)
        print(f"[run_v3] ready — {len(_a.rovers)}대 vehicle namespaces: {nss}")
    else:
        print("[run_v3] ready — v3 내장 Action Graph 가 센서 토픽 발행 중 "
              "(/imu/data /joint_states_raw /camera/* /camera/overview/* /camera/chase/*)")

    step = 0
    chase_state: dict = {}
    # GPU/RTF 부담 측정용 — 600 step 사이 wall-clock 도 함께 출력.
    import time
    last_log_wall = time.time()
    try:
        while app.is_running():
            world.step(render=True)
            # 단일 모드 chase cam.
            if chase_xform_op is not None:
                _update_chase_cam(stage, chase_xform_op, chase_state)
            # multi 모드 per-rover chase cams.
            for entry in multi_chase_cams:
                _update_chase_cam(stage, entry["op"], entry["state"])
            if step % 600 == 0:
                now = time.time()
                dt = now - last_log_wall
                last_log_wall = now
                # 600 step / dt = 실시간 step rate. 60Hz sim 이면 ~60 step/s
                # = 600 step / 10s 가 정상. 그보다 길면 RTF<1 = lag.
                rate = (600.0 / dt) if dt > 0 else 0.0
                print(f"[run_v3] running... step {step}  "
                      f"({rate:.1f} step/s, 600-step wall={dt:.2f}s)")
            step += 1
    except KeyboardInterrupt:
        pass
    finally:
        app.close()


if __name__ == "__main__":
    main()
