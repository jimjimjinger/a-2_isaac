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

    for _ in range(20):
        app.update()
    world.reset()
    world.play()
    print("[run_v3] ready — v3 내장 Action Graph 가 센서 토픽 발행 중 "
          "(/imu/data /joint_states_raw /camera/*)")

    step = 0
    try:
        while app.is_running():
            world.step(render=True)
            if step % 600 == 0:
                print(f"[run_v3] running... step {step}")
            step += 1
    except KeyboardInterrupt:
        pass
    finally:
        app.close()


if __name__ == "__main__":
    main()
