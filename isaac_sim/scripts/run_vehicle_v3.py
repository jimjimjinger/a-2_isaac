"""vehicle_v3.usd (액션그래프 내장 로버) 를 terrain 에 올려 구동.

v3 는 ROS2 센서 그래프가 USD 에 내장돼 있다 — 이 런처는 그래프를 짜지 않는다.
terrain 로드 + v3 reference + play 만 한다. 팀 누구든 이 패턴(또는 이 스크립트)
으로 v3 를 띄워 자기 노드를 개발하면 된다 — 실물 로봇처럼.

실행: <isaac-python> isaac_sim/scripts/run_vehicle_v3.py [--terrain terrain_00004] [--headless]
"""
import argparse
import json
import os
import sys

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
_a, _ = _p.parse_known_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": _a.headless})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
app.update()

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, UsdGeom

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


def _add_chase_camera(stage):
    """rover 뒤에서 따라가는 chase 카메라 prim. 매 step _update_chase_cam 이
    transform op 의 4x4 매트릭스를 갱신해 rover 를 따라간다."""
    cam = UsdGeom.Camera.Define(stage, CHASE_CAM)
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    # 단일 TransformOp 으로 look-at 매트릭스 직접 작성 — 매 frame Set 만 호출.
    transform_op = xf.AddTransformOp()
    cam.GetFocalLengthAttr().Set(20.0)
    cam.GetHorizontalApertureAttr().Set(20.955)
    cam.GetVerticalApertureAttr().Set(20.955 * (CHASE_H / CHASE_W))
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 100000.0))
    UsdGeom.Imageable(cam.GetPrim()).MakeInvisible()
    print(f"[run_v3] chase camera 생성: {CHASE_CAM} offset_local={CHASE_OFFSET_LOCAL}")
    return transform_op


def _create_chase_graph() -> None:
    """vehicle_v3 / overview 와 분리된 별도 OmniGraph (RP + CamHelper)."""
    import omni.graph.core as og
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": CHASE_GRAPH, "evaluator_name": "execution"},
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
                ("CamChaseRgb.inputs:topicName", CHASE_TOPIC),
                ("CamChaseRgb.inputs:type", "rgb"),
                ("CamChaseRgb.inputs:frameId", "chase_camera"),
                ("CamChaseInfo.inputs:topicName", CHASE_INFO_TOPIC),
                ("CamChaseInfo.inputs:frameId", "chase_camera"),
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
            prim=stage.GetPrimAtPath(f"{CHASE_GRAPH}/RPChase"),
            attribute="inputs:cameraPrim",
            target_prim_paths=[CHASE_CAM])
    except Exception as e:
        print(f"[run_v3] ⚠ chase cameraPrim 연결 실패: {e}")
        return
    print(f"[run_v3] chase graph 구성 완료 → {CHASE_TOPIC}")


def _find_articulation_root(stage):
    """build_vehicle_v3.GT_SCRIPT 와 동일 패턴 — PhysicsArticulationRootAPI 가
    있는 첫 prim. 없으면 m0609/base_link 이름 패턴 fallback."""
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
        artic = _find_articulation_root(stage)
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


def main() -> None:
    for f in (WORLD, V3):
        if not os.path.isfile(f):
            print(f"[run_v3] ✗ 파일 없음: {f}")
            app.close()
            sys.exit(1)

    world = World(stage_units_in_meters=1.0)
    add_reference_to_stage(usd_path=WORLD, prim_path="/World/MarsScene")
    print(f"[run_v3] 씬 로드: {WORLD}")

    # 검증된 spawn 위치 (terrain meta.json)
    spawn = (0.0, 0.0, 1.0)
    meta = os.path.join(TERRAIN_DIR, "meta.json")
    if os.path.isfile(meta):
        with open(meta) as f:
            spots = json.load(f).get("spawn_locations") or []
        if spots:
            s = spots[0]
            spawn = (float(s["x"]), float(s["y"]), float(s["z"]) + 0.3)

    # v3 reference — 그래프가 USD 에 내장돼 있어 그대로 따라온다.
    add_reference_to_stage(usd_path=V3, prim_path=ROVER_PRIM)
    stage = omni.usd.get_context().get_stage()
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(ROVER_PRIM))
    top = None
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            top = op
            break
    if top is None:
        top = xf.AddTranslateOp()
    top.Set(Gf.Vec3d(*spawn))
    print(f"[run_v3] vehicle_v3 spawn: {spawn}")

    # World-fixed overview camera + 자체 OmniGraph 추가. UI 메인 화면 overview
    # 슬롯이 /camera/overview/image_raw 토픽을 구독. vehicle_v3 reference 이후
    # 별개 layer 에 author 하므로 v3 USD 와 무관.
    if not _a.no_overview:
        _add_overview_camera(stage)
        _create_overview_graph()
    else:
        print("[run_v3] overview cam 비활성 (--no-overview)")
    # rover 뒤에서 따라가는 chase 카메라 — main loop 에서 매 step 위치 갱신.
    chase_xform_op = None
    if not _a.no_chase:
        chase_xform_op = _add_chase_camera(stage)
        _create_chase_graph()
    else:
        print("[run_v3] chase cam 비활성 (--no-chase)")

    for _ in range(20):
        app.update()
    world.reset()
    world.play()
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
            # chase cam 비활성 시 transform_op = None → skip.
            if chase_xform_op is not None:
                _update_chase_cam(stage, chase_xform_op, chase_state)
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
