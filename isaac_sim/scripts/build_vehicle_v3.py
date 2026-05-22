"""vehicle_v3.usd 빌드 — vehicle_v2.usd(asset)에 ROS2 센서 Action Graph 를
구워넣은(bake) "액션그래프 내장 로버".

v3 = 고정된 로봇. terrain 에 reference 하면 그 자체로 ROS2 센서 인터페이스를
발행한다 — 런타임 그래프 빌더 코드가 필요 없다. 실물 하드웨어처럼 "주어진 것".

구조:  vehicle_v3.usd
         └ /Root  (reference → ./vehicle_v2.usd, defaultPrim)
              ├ Vehicle/...        (v2 에서 합성된 외형·물리·센서 prim)
              └ ActionGraph        (이 스크립트가 굽는 센서 그래프)

이번 단계: 센서(IMU·joint·카메라). 주행(/cmd_vel)·odom 의 in-graph 화는 다음
단계(v3 재bake) — 현재 Python 루프 의존이라 그대로는 못 굽는다.

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
from pxr import Sdf, Usd

enable_extension("isaacsim.ros2.bridge")
app.update()

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC_SIM = os.path.dirname(HERE)
VEHICLE_DIR = os.path.join(ISAAC_SIM, "assets", "vehicle")
V2 = os.path.join(VEHICLE_DIR, "vehicle_v2.usd")
V3 = os.path.join(VEHICLE_DIR, "vehicle_v3.usd")

GRAPH = "/Root/ActionGraph"
# v3 /Root 가 v2 를 reference → v2 의 /Root/Vehicle/... 가 /Root/Vehicle/... 로 합성.
# 그래프와 target 을 /Root/ 하위로 author 해야 v3 가 terrain 에 reference 될 때
# 경로가 통째로 remap 된다 (/Root → /World/Rover).
IMU       = "/Root/Vehicle/rover/Body/Imu_Sensor"
ARTIC     = "/Root/Vehicle/m0609/base_link"
_D455     = "/Root/Vehicle/onrobot_rg2ft/angle_bracket/realsense_d455/RSD455"
ROVER_CAM = "/Root/Vehicle/rover/Body/Camera"
WRIST_RGB = _D455 + "/Camera_OmniVision_OV9782_Color"
WRIST_DEP = _D455 + "/Camera_Pseudo_Depth"


def _set_targets(node_path: str, input_name: str, target_path: str) -> None:
    """OmniGraph 노드의 target(relationship) 입력 설정."""
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


def main() -> None:
    if not os.path.isfile(V2):
        print(f"[build_v3] ✗ vehicle_v2.usd 없음: {V2}")
        app.close()
        sys.exit(1)

    # ── v3 골격 생성: /Root 가 vehicle_v2.usd 를 상대경로로 reference ──
    # 상대경로라 다른 팀원 머신(다른 절대경로)에서도 그대로 동작한다.
    if os.path.exists(V3):
        os.remove(V3)
    base = Usd.Stage.CreateNew(V3)
    root = base.DefinePrim("/Root", "Xform")
    root.GetReferences().AddReference("./vehicle_v2.usd")
    base.SetDefaultPrim(root)
    base.Save()
    del base
    print(f"[build_v3] v3 골격 생성 — /Root → ./vehicle_v2.usd")

    # ── context 로 열어 (v2 합성) 센서 그래프 author ──
    ctx = omni.usd.get_context()
    ctx.open_stage(V3)
    for _ in range(80):
        app.update()
    stage = ctx.get_stage()
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
                # IMU
                ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PubImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
                # 관절 상태
                ("PubJoint", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                # 카메라 — render product 3 + helper 6
                ("RPRover", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RPWristRgb", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RPWristDepth", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CamRoverRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CamRoverDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CamRoverInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("CamWristRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CamWristDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CamWristInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
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
            ],
            keys.CONNECT: [
                # IMU
                ("OnTick.outputs:tick", "ReadIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut", "PubImu.inputs:execIn"),
                ("ReadIMU.outputs:angVel", "PubImu.inputs:angularVelocity"),
                ("ReadIMU.outputs:linAcc", "PubImu.inputs:linearAcceleration"),
                ("ReadIMU.outputs:orientation", "PubImu.inputs:orientation"),
                ("ReadIMU.outputs:sensorTime", "PubImu.inputs:timeStamp"),
                # 관절
                ("OnTick.outputs:tick", "PubJoint.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime",
                 "PubJoint.inputs:timeStamp"),
                # 카메라 render product
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
            ],
        },
    )
    _set_targets(f"{GRAPH}/ReadIMU", "inputs:imuPrim", IMU)
    _set_targets(f"{GRAPH}/PubJoint", "inputs:targetPrim", ARTIC)
    _set_targets(f"{GRAPH}/RPRover", "inputs:cameraPrim", ROVER_CAM)
    _set_targets(f"{GRAPH}/RPWristRgb", "inputs:cameraPrim", WRIST_RGB)
    _set_targets(f"{GRAPH}/RPWristDepth", "inputs:cameraPrim", WRIST_DEP)
    print("[build_v3] 센서 Action Graph author 완료 (IMU·joint·카메라)")

    # ── 저장 ──
    stage.GetRootLayer().Save()
    print(f"[build_v3] ✓ 저장 완료: {V3}")
    app.close()


if __name__ == "__main__":
    main()
