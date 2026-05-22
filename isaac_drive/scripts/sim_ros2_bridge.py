"""Isaac Sim ↔ ROS2 브리지 (정공법) — isaacsim.ros2.bridge 확장 사용.

UDP 우회(sim_ros_bridge.py + coverage_udp_relay.py) 대신, Isaac Sim 의
ROS2 Bridge 확장으로 ROS2 토픽을 C++/DDS 레벨에서 직접 발행·구독한다.
rclpy 를 안 거치므로 Isaac Sim Python(3.11) vs ROS2 Humble(3.10) 버전
불일치와 무관하다 — UDP 릴레이가 필요 없다.

  · 구독: /cmd_vel (geometry_msgs/Twist)  — ROS2SubscribeTwist OmniGraph 노드
          → 매 프레임 Python 이 읽어 RoverController.drive() (Ackermann 변환)
  · 발행: /odom    (nav_msgs/Odometry)    — ROS2PublishOdometry 에 로버 절대 월드 pose 기록
  · 발행: /imu/data, /joint_states_raw, /camera/rover/*, /camera/wrist/*
          — 지민(T5) vehicle_v2_scene.usd 의 LocalizationSensors·RoverStatePublishers
            Action Graph 를 코드로 포팅한 것 (USD 임베디드 아님, 런타임 코드 저작).

닫힌 루프:
  coverage_node ─/cmd_vel─▶ [이 브리지] ─▶ Isaac Sim 로버
  coverage_node ◀─/rover/estimated_pose─ odom_to_estimated_pose ◀─/odom─ [이 브리지]

실행 (터미널 2개 — UDP 릴레이 불필요):
    # A — Isaac Sim 브리지
    <isaac-python> sim_ros2_bridge.py --terrain terrain_00004
    # B — coverage + odom 어댑터 (시스템 ROS2)
    ros2 run isaac_drive odom_to_estimated_pose
    ros2 run isaac_drive coverage_node

⚠️ coverage_node 와 ROS_DOMAIN_ID 를 맞출 것 (이 프로세스 실행 셸의 env).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

# SimulationApp 환경에서 print 가 블록 버퍼링돼 로그가 지연/유실되지 않도록 line-buffered.
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

# argparse 는 SimulationApp 보다 먼저.
_parser = argparse.ArgumentParser(description="Isaac Sim ↔ ROS2 bridge (정공법)")
_parser.add_argument("--terrain", default="terrain_00004",
                     help="terrain id (coverage_node 의 terrain_dir 와 일치시킬 것). "
                          "terrain_00001 은 v1 잔재라 씬 어긋남 — v2 terrain 사용")
_parser.add_argument("--headless", action="store_true", help="GUI 없이 실행")
_args, _ = _parser.parse_known_args()

# SimulationApp 은 다른 omniverse import 보다 먼저.
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": _args.headless})

# ── ROS2 Bridge 확장 활성화 (SimulationApp 직후) ──
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import omni.graph.core as og
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Sdf

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)            # .../isaac_drive
WS = os.path.dirname(PKG_ROOT)              # .../a2_isaac
sys.path.insert(0, HERE)

from rover import RoverController, ROVER_PRIM_PATH, ARTIC_PRIM_PATH

TERRAIN_ID = _args.terrain
MARS_WORLD = f"{WS}/isaac_sim/worlds/{TERRAIN_ID}.usd"
TERRAIN_DIR = f"{WS}/isaac_sim/assets/generated_terrains/{TERRAIN_ID}"

GRAPH_PATH = "/ActionGraph"
CMD_VEL_TOPIC = "cmd_vel"   # → /cmd_vel
ODOM_TOPIC = "odom"         # → /odom

# 지민(T5) vehicle_v2_scene.usd 의 Action Graph 가 가리키던 센서 prim.
# 그 씬은 rover 를 /World/Vehicle/rover 에 참조하지만 이 브리지는 /World/Rover
# 에 참조하므로 경로를 remap 한다.
_RG2_D455 = "/Vehicle/onrobot_rg2ft/angle_bracket/realsense_d455/RSD455"
IMU_PRIM_PATH        = ROVER_PRIM_PATH + "/Vehicle/rover/Body/Imu_Sensor"
ROVER_CAM_PATH       = ROVER_PRIM_PATH + "/Vehicle/rover/Body/Camera"
WRIST_RGB_CAM_PATH   = ROVER_PRIM_PATH + _RG2_D455 + "/Camera_OmniVision_OV9782_Color"
WRIST_DEPTH_CAM_PATH = ROVER_PRIM_PATH + _RG2_D455 + "/Camera_Pseudo_Depth"


def _set_targets(node_path: str, input_name: str, target_path: str) -> None:
    """OmniGraph 노드의 target(relationship) 입력 설정.

    og.Controller.edit 의 SET_VALUES 는 relationship 입력을 못 잡으므로 따로
    author 한다. isaacsim 유틸을 우선 쓰고, 없으면 USD 관계로 직접 설정한다.
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


def _build_ros2_graph() -> None:
    """ROS2 Bridge OmniGraph 구축 — 코드 저작 (USD 임베디드 아님).

    핵심(주행):
      구독 ROS2SubscribeTwist(/cmd_vel) — Python 이 매 프레임 읽어 Ackermann 구동.
      발행 ROS2PublishOdometry(/odom)   — Python 이 매 프레임 로버 절대 월드
           pose 를 써 넣는다 (coverage_node 가 obstacle_grid 를 절대좌표로
           인덱싱하므로 상대 odometry 는 쓰지 않는다).

    센서(지민 T5 vehicle_v2_scene.usd 의 Action Graph 를 코드로 포팅):
      LocalizationSensors  → /imu/data, /camera/rover/*, /camera/wrist/*
      RoverStatePublishers → /joint_states_raw
    RoverAckermannDrive 는 포팅하지 않는다 — 라이브 경로의 Ackermann 은
    rover.py(Python)가 담당하므로 USD 스크립트노드 버전과 중복된다.
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                # 주행
                ("SubTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                # IMU
                ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PubImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
                # 관절 상태
                ("PubJoint", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                # 카메라 — render product 3종 + helper 6종
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
                # 주행
                ("SubTwist.inputs:topicName", CMD_VEL_TOPIC),
                ("PubOdom.inputs:topicName", ODOM_TOPIC),
                ("PubOdom.inputs:odomFrameId", "odom"),
                ("PubOdom.inputs:chassisFrameId", "base_link"),
                # IMU → /imu/data
                ("ReadIMU.inputs:readGravity", True),
                ("PubImu.inputs:topicName", "/imu/data"),
                ("PubImu.inputs:frameId", "sim_imu"),
                ("PubImu.inputs:publishAngularVelocity", True),
                ("PubImu.inputs:publishLinearAcceleration", True),
                ("PubImu.inputs:publishOrientation", True),
                # 관절 → /joint_states_raw
                ("PubJoint.inputs:topicName", "/joint_states_raw"),
                # 카메라 render product 해상도
                ("RPRover.inputs:width", 640),
                ("RPRover.inputs:height", 480),
                ("RPWristRgb.inputs:width", 640),
                ("RPWristRgb.inputs:height", 480),
                ("RPWristDepth.inputs:width", 640),
                ("RPWristDepth.inputs:height", 480),
                # 로버 바디 카메라 → /camera/rover/*
                ("CamRoverRgb.inputs:topicName", "/camera/rover/image_raw"),
                ("CamRoverRgb.inputs:type", "rgb"),
                ("CamRoverRgb.inputs:frameId", "rover_camera"),
                ("CamRoverDepth.inputs:topicName", "/camera/rover/depth"),
                ("CamRoverDepth.inputs:type", "depth"),
                ("CamRoverDepth.inputs:frameId", "rover_camera"),
                ("CamRoverInfo.inputs:topicName", "/camera/rover/camera_info"),
                ("CamRoverInfo.inputs:frameId", "rover_camera"),
                # 손목 D455 카메라 → /camera/wrist/*
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
                # 주행
                ("OnTick.outputs:tick", "SubTwist.inputs:execIn"),
                ("OnTick.outputs:tick", "PubOdom.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime",
                 "PubOdom.inputs:timeStamp"),
                # IMU: OnTick → ReadIMU → PubImu
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
                # 카메라 render product 생성
                ("OnTick.outputs:tick", "RPRover.inputs:execIn"),
                ("OnTick.outputs:tick", "RPWristRgb.inputs:execIn"),
                ("OnTick.outputs:tick", "RPWristDepth.inputs:execIn"),
                # 로버 바디 카메라: RPRover → rgb/depth/info
                ("RPRover.outputs:execOut", "CamRoverRgb.inputs:execIn"),
                ("RPRover.outputs:renderProductPath",
                 "CamRoverRgb.inputs:renderProductPath"),
                ("RPRover.outputs:execOut", "CamRoverDepth.inputs:execIn"),
                ("RPRover.outputs:renderProductPath",
                 "CamRoverDepth.inputs:renderProductPath"),
                ("RPRover.outputs:execOut", "CamRoverInfo.inputs:execIn"),
                ("RPRover.outputs:renderProductPath",
                 "CamRoverInfo.inputs:renderProductPath"),
                # 손목 카메라: rgb 는 RPWristRgb, depth/info 는 RPWristDepth
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
    # relationship(target) 입력 — 센서 prim 연결 (og.Controller.edit 미지원)
    _set_targets(f"{GRAPH_PATH}/ReadIMU", "inputs:imuPrim", IMU_PRIM_PATH)
    _set_targets(f"{GRAPH_PATH}/PubJoint", "inputs:targetPrim", ARTIC_PRIM_PATH)
    _set_targets(f"{GRAPH_PATH}/RPRover", "inputs:cameraPrim", ROVER_CAM_PATH)
    _set_targets(f"{GRAPH_PATH}/RPWristRgb", "inputs:cameraPrim",
                 WRIST_RGB_CAM_PATH)
    _set_targets(f"{GRAPH_PATH}/RPWristDepth", "inputs:cameraPrim",
                 WRIST_DEPTH_CAM_PATH)


def main() -> None:
    if not os.path.isfile(MARS_WORLD):
        print(f"[sim_ros2_bridge] ✗ 월드 USD 없음: {MARS_WORLD}")
        simulation_app.close()
        sys.exit(1)

    # ── Isaac Sim World + per-terrain 씬 ──
    my_world = World(stage_units_in_meters=1.0)
    add_reference_to_stage(usd_path=MARS_WORLD, prim_path="/World/MarsScene")
    print(f"[sim_ros2_bridge] 씬 로드: {MARS_WORLD}")

    # ── 로버 spawn (meta.json 의 검증된 spawn 위치) ──
    spawn_xyz = (0.0, 0.0, 1.0)
    meta_path = os.path.join(TERRAIN_DIR, "meta.json")
    if os.path.isfile(meta_path):
        with open(meta_path) as f:
            spots = json.load(f).get("spawn_locations") or []
        if spots:
            s = spots[0]
            spawn_xyz = (float(s["x"]), float(s["y"]), float(s["z"]) + 0.3)
    print(f"[sim_ros2_bridge] 로버 spawn: {spawn_xyz}")

    rover = RoverController(my_world)
    rover.spawn(initial_position=spawn_xyz)
    for _ in range(10):
        simulation_app.update()
    # 로버 카메라 뷰포트 — GUI 모드에서만 (headless 면 viewport 무의미).
    # run_coverage_test.py 와 동일하게 reset 전에 부착한다.
    if not _args.headless:
        rover.attach_camera()
        for _ in range(5):
            simulation_app.update()
    my_world.reset()
    rover.initialize()

    # ── ROS2 Bridge OmniGraph 구축 (로버 prim 존재 후) ──
    _build_ros2_graph()
    print("[sim_ros2_bridge] ROS2 그래프 구축 완료 — 구독 /cmd_vel · 발행 "
          "/odom /imu/data /joint_states_raw /camera/rover/* /camera/wrist/*")

    my_world.play()
    print("[sim_ros2_bridge] ready — coverage_node + odom_to_estimated_pose 를 띄우세요")

    lin_attr = og.Controller.attribute(
        f"{GRAPH_PATH}/SubTwist.outputs:linearVelocity")
    ang_attr = og.Controller.attribute(
        f"{GRAPH_PATH}/SubTwist.outputs:angularVelocity")
    pos_attr = og.Controller.attribute(f"{GRAPH_PATH}/PubOdom.inputs:position")
    ori_attr = og.Controller.attribute(f"{GRAPH_PATH}/PubOdom.inputs:orientation")

    # /odom 첫 프레임 시드 — PubOdom 의 inputs:position 기본값은 (0,0,0)이다.
    # 루프가 첫 step 에서 실제 pose 를 써넣기 전에 OnTick 이 한 번 발행하면
    # /odom 첫 메시지가 (0,0,0)으로 나가고, coverage_node 가 그걸 첫 pose 로
    # 받아 맵 원점에 reveal_radius 짜리 가짜 reveal 을 새긴다 — 미니맵에서
    # 베이스캠프 박스 안 원형으로 보이던 그것. 루프 전에 실제 spawn pose 로 채운다.
    x0, y0, yaw0 = rover.get_pose_2d()
    og.Controller.set(pos_attr, [float(x0), float(y0), 0.0])
    og.Controller.set(ori_attr,
                      [0.0, 0.0, math.sin(yaw0 / 2.0), math.cos(yaw0 / 2.0)])
    print(f"[sim_ros2_bridge] /odom 초기 pose 시드 "
          f"({x0:+.2f},{y0:+.2f}, {math.degrees(yaw0):+.0f}°)")

    lin_x = ang_z = 0.0
    step = 0
    try:
        while simulation_app.is_running():
            my_world.step(render=True)
            if not my_world.is_playing():
                continue

            # 로버 절대 월드 pose → /odom 발행 노드에 기록 (상대 odometry 아님)
            x, y, yaw = rover.get_pose_2d()
            og.Controller.set(pos_attr, [float(x), float(y), 0.0])
            og.Controller.set(
                ori_attr,
                [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])

            # /cmd_vel (그래프가 구독) → Ackermann 구동
            lin = og.Controller.get(lin_attr)
            ang = og.Controller.get(ang_attr)
            if lin is not None:
                lin_x = float(lin[0])
            if ang is not None:
                ang_z = float(ang[2])
            rover.drive(lin_x, ang_z)

            if step % 120 == 0:
                print(f"[sim_ros2_bridge] step {step:6d}  "
                      f"pose=({x:+6.2f},{y:+6.2f})  cmd=({lin_x:+.2f},{ang_z:+.2f})")
            step += 1
    except KeyboardInterrupt:
        pass
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
