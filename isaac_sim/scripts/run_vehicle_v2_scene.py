"""vehicle_v2_scene.usd 로드·play + ground-truth /odom 발행.

T5 Action Graph 3종(LocalizationSensors / RoverStatePublishers /
RoverAckermannDrive)이 센서 발행·Ackermann 주행을 담당하고, 이 런처는
거기에 **ground-truth odometry** 발행을 더한다 — 개발단계 스캐폴드.

  · localization 담당: /ground_truth/odom 을 *참조하지 않고* pose 추정 개발
  · 그 외 기능(coverage 등): 이 토픽을 perfect-pose 대용으로 참조해 병렬 개발

T5 씬은 /odom 을 의도적으로 뺐으므로(실세계 인터페이스 지향), GT 는 별도
토픽명 /ground_truth/odom 으로 발행 — 실 인터페이스와 혼동 방지.

실행:
    <isaac-python> isaac_sim/scripts/run_vehicle_v2_scene.py [--headless] [--no-odom]

coverage 연결 (별도 터미널, 시스템 ROS2):
    ros2 run isaac_drive odom_to_estimated_pose \\
        --ros-args -p odom_topic:=/ground_truth/odom
    ros2 run isaac_drive coverage_node

⚠️ ROS_DOMAIN_ID 를 구독측과 맞출 것. 첫 실행은 로그를 보며 확인.
"""
import argparse
import math
import os
import sys

# SimulationApp 환경에서 print 가 버퍼링돼 유실되지 않도록 line-buffered.
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

_parser = argparse.ArgumentParser(description="vehicle_v2 씬 로드·play + GT odom")
_parser.add_argument("--headless", action="store_true", help="GUI 없이 실행")
_parser.add_argument("--no-odom", action="store_true",
                     help="ground-truth /odom 발행 끄기 (씬만 구동)")
_args, _ = _parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": _args.headless})

# ROS2 Bridge 확장 — Action Graph 의 ROS2 노드가 동작하려면 필수.
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import omni.graph.core as og
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from pxr import UsdGeom

HERE = os.path.dirname(os.path.abspath(__file__))     # .../isaac_sim/scripts
ISAAC_SIM = os.path.dirname(HERE)                     # .../isaac_sim
SCENE = f"{ISAAC_SIM}/assets/vehicle/vehicle_v2_scene.usd"

# 씬 내부 prim 경로 (vehicle_v2.usd 가 /World/Vehicle/rover 에 참조됨)
ARTIC = "/World/Vehicle/rover/Vehicle/m0609/base_link"   # articulation root
BODY  = "/World/Vehicle/rover/Vehicle/rover/Body"        # 주행 섀시
ODOM_GRAPH = "/GroundTruthOdom"
GT_ODOM_TOPIC = "ground_truth/odom"                      # → /ground_truth/odom


def _yaw_from_quat(q):
    """quat (w,x,y,z) → yaw."""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _build_odom_graph():
    """ground-truth odom 발행 OmniGraph — Python 이 매 프레임 절대 pose 를 써넣는다.

    IsaacComputeOdometry 는 spawn 을 원점으로 하는 상대 odometry 라 쓰지
    않는다 — coverage_node 가 obstacle_grid 를 절대좌표로 인덱싱하므로
    절대 pose 가 필수. sim_ros2_bridge.py 와 동일한 방식.
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": ODOM_GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ],
            keys.SET_VALUES: [
                ("PubOdom.inputs:topicName", GT_ODOM_TOPIC),
                ("PubOdom.inputs:odomFrameId", "odom"),
                ("PubOdom.inputs:chassisFrameId", "base_link"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "PubOdom.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PubOdom.inputs:timeStamp"),
            ],
        },
    )


def _measure_body_offset(stage):
    """articulation root(m0609 base) → 섀시(Body) 강체 오프셋 (로버 로컬프레임).

    get_world_pose() 는 articulation root pose 라 섀시 중심과 ~0.17m
    어긋난다. 고정 오프셋을 측정해 보정한다 (yaw 동일).
    """
    root = stage.GetPrimAtPath(ARTIC)
    body = stage.GetPrimAtPath(BODY)
    if not root.IsValid() or not body.IsValid():
        print("[run_v2_scene] ⚠ Body 오프셋 측정 실패 — root pose 그대로 사용")
        return (0.0, 0.0)
    cache = UsdGeom.XformCache()
    rm = cache.GetLocalToWorldTransform(root)
    rt = rm.ExtractTranslation()
    bt = cache.GetLocalToWorldTransform(body).ExtractTranslation()
    q = rm.ExtractRotationQuat()
    iq = q.GetImaginary()
    yaw = _yaw_from_quat((q.GetReal(), iq[0], iq[1], iq[2]))
    dxw, dyw = rt[0] - bt[0], rt[1] - bt[1]
    ox = math.cos(yaw) * dxw + math.sin(yaw) * dyw
    oy = -math.sin(yaw) * dxw + math.cos(yaw) * dyw
    return (float(ox), float(oy))


def _publish_pose(rover, body_off, pos_attr, ori_attr):
    """로버 섀시 절대 pose 를 PubOdom 노드 입력에 써넣는다."""
    pos, ori = rover.get_world_pose()          # ori = (w,x,y,z)
    yaw = _yaw_from_quat(ori)
    ox, oy = body_off
    cx = float(pos[0]) - (math.cos(yaw) * ox - math.sin(yaw) * oy)
    cy = float(pos[1]) - (math.sin(yaw) * ox + math.cos(yaw) * oy)
    og.Controller.set(pos_attr, [cx, cy, 0.0])
    og.Controller.set(ori_attr,
                      [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])
    return cx, cy


def main() -> None:
    if not os.path.isfile(SCENE):
        print(f"[run_v2_scene] ✗ 씬 USD 없음: {SCENE}")
        simulation_app.close()
        sys.exit(1)

    omni.usd.get_context().open_stage(SCENE)
    print(f"[run_v2_scene] 씬 로드: {SCENE}")
    # 씬이 terrain·차량·D455 텍스처를 참조 — 로드 완료까지 충분히 update.
    for _ in range(200):
        simulation_app.update()

    world = World()

    rover = None
    pos_attr = ori_attr = None
    body_off = (0.0, 0.0)
    if not _args.no_odom:
        _build_odom_graph()
        rover = SingleArticulation(prim_path=ARTIC, name="v2_rover_gt")
        print(f"[run_v2_scene] ground-truth odom 그래프 구축 → /{GT_ODOM_TOPIC}")

    world.reset()

    if rover is not None:
        rover.initialize()
        body_off = _measure_body_offset(omni.usd.get_context().get_stage())
        pos_attr = og.Controller.attribute(f"{ODOM_GRAPH}/PubOdom.inputs:position")
        ori_attr = og.Controller.attribute(f"{ODOM_GRAPH}/PubOdom.inputs:orientation")
        print(f"[run_v2_scene] Body 오프셋 {body_off} — 섀시 기준으로 보정")

    world.play()
    print("[run_v2_scene] play — T5 Action Graph + ground-truth odom 발행 중")

    if rover is not None:
        # /ground_truth/odom 첫 프레임을 실제 pose 로 시드 (기본값 0,0,0 방지)
        cx, cy = _publish_pose(rover, body_off, pos_attr, ori_attr)
        print(f"[run_v2_scene] /{GT_ODOM_TOPIC} 시드 ({cx:+.2f},{cy:+.2f})")
        print("[run_v2_scene] ⚠ /ground_truth/odom 은 개발용 GT 스캐폴드 "
              "— localization 은 미참조")

    step = 0
    try:
        while simulation_app.is_running():
            world.step(render=not _args.headless)
            if not world.is_playing():
                continue
            if rover is not None:
                _publish_pose(rover, body_off, pos_attr, ori_attr)
            if step % 600 == 0:
                print(f"[run_v2_scene] running... step {step}")
            step += 1
    except KeyboardInterrupt:
        pass
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
