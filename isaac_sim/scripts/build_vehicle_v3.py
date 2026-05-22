"""vehicle_v3.usd 빌드 — vehicle_v2.usd(asset)에 ROS2 Action Graph 를
구워넣은(bake) "액션그래프 내장 로버".

v3 = 고정된 로봇. terrain 에 reference·play 하면 그 자체로 ROS2 인터페이스를
발행/구독한다 — 런타임 그래프 빌더 코드가 필요 없다. 실물 하드웨어처럼.

구조:  vehicle_v3.usd
         └ /Root  (reference → ./vehicle_v2.usd, defaultPrim)
              ├ Vehicle/...     (v2 에서 합성된 외형·물리·센서 prim)
              └ ActionGraph     (이 스크립트가 굽는 그래프)
                  · 센서: IMU·joint·카메라 발행
                  · 주행: /cmd_vel 구독 → ScriptNode Ackermann → 휠 관절 구동

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
# 그래프·target 을 /Root/ 하위로 author → v3 가 terrain 에 reference 될 때
# 경로가 통째로 remap (/Root → /World/Rover).
IMU       = "/Root/Vehicle/rover/Body/Imu_Sensor"
ARTIC     = "/Root/Vehicle/m0609/base_link"
_D455     = "/Root/Vehicle/onrobot_rg2ft/angle_bracket/realsense_d455/RSD455"
ROVER_CAM = "/Root/Vehicle/rover/Body/Camera"
WRIST_RGB = _D455 + "/Camera_OmniVision_OV9782_Color"
WRIST_DEP = _D455 + "/Camera_Pseudo_Depth"

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
WHEEL_VELOCITY_GAIN = 1.5


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


def main() -> None:
    if not os.path.isfile(V2):
        print(f"[build_v3] ✗ vehicle_v2.usd 없음: {V2}")
        app.close()
        sys.exit(1)

    # ── v3 골격: /Root 가 vehicle_v2.usd 를 상대경로로 reference ──
    if os.path.exists(V3):
        os.remove(V3)
    base = Usd.Stage.CreateNew(V3)
    root = base.DefinePrim("/Root", "Xform")
    root.GetReferences().AddReference("./vehicle_v2.usd")
    base.SetDefaultPrim(root)
    base.Save()
    del base
    print("[build_v3] v3 골격 생성 — /Root → ./vehicle_v2.usd")

    # ── context 로 열어 (v2 합성) 그래프 author ──
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
                # ── 센서 ──
                ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PubImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
                ("PubJoint", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("RPRover", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RPWristRgb", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RPWristDepth", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CamRoverRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CamRoverDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CamRoverInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("CamWristRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CamWristDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CamWristInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                # ── 주행 (지민 RoverAckermannDrive 포팅) ──
                ("SubTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("Ackermann", "omni.graph.scriptnode.ScriptNode"),
                ("SteerCtrl", "isaacsim.core.nodes.IsaacArticulationController"),
                ("DriveCtrl", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.CREATE_ATTRIBUTES: [
                # ScriptNode 커스텀 포트 — Ackermann 입출력
                ("Ackermann.inputs:linearVelocity", "vectord[3]"),
                ("Ackermann.inputs:angularVelocity", "vectord[3]"),
                ("Ackermann.outputs:steerJointNames", "token[]"),
                ("Ackermann.outputs:driveJointNames", "token[]"),
                ("Ackermann.outputs:steeringAngles", "double[]"),
                ("Ackermann.outputs:wheelVelocities", "double[]"),
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
                # 주행
                ("SubTwist.inputs:topicName", "/cmd_vel"),
                ("Ackermann.inputs:script", ACK_SCRIPT),
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
            ],
        },
    )
    # relationship(target) 입력 — 센서 prim · articulation 연결
    _set_targets(f"{GRAPH}/ReadIMU", "inputs:imuPrim", IMU)
    _set_targets(f"{GRAPH}/PubJoint", "inputs:targetPrim", ARTIC)
    _set_targets(f"{GRAPH}/RPRover", "inputs:cameraPrim", ROVER_CAM)
    _set_targets(f"{GRAPH}/RPWristRgb", "inputs:cameraPrim", WRIST_RGB)
    _set_targets(f"{GRAPH}/RPWristDepth", "inputs:cameraPrim", WRIST_DEP)
    _set_targets(f"{GRAPH}/SteerCtrl", "inputs:targetPrim", ARTIC)
    _set_targets(f"{GRAPH}/DriveCtrl", "inputs:targetPrim", ARTIC)
    print("[build_v3] Action Graph author 완료 — 센서(IMU·joint·카메라) + "
          "주행(/cmd_vel→Ackermann→휠)")

    stage.GetRootLayer().Save()
    print(f"[build_v3] ✓ 저장 완료: {V3}")
    app.close()


if __name__ == "__main__":
    main()
