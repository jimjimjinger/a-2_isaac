"""m0609 팔이 펴지는 원인 점검 (일회성).

[A] USD 에 박힌 팔 조인트 상태 (pxr 직접 읽기)
[B] Isaac Lab: 생성 직후(USD 자세) vs reset 후(내 init_state 자세) joint_pos

    /home/rokey/dev_ws/venv/isaaclab/bin/python _inspect_armpose.py --headless
    cat /tmp/armpose.txt
"""

import argparse
import math
import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Usd  # noqa: E402

from isaaclab.envs import ManagerBasedEnv  # noqa: E402

from drive_env_cfg import DriveEnvCfg  # noqa: E402

REPORT = "/tmp/armpose.txt"
out = open(REPORT, "w", buffering=1)
try:
    # --- [A] USD 가 가진 팔 조인트 상태 ---
    _REPO = os.path.abspath(os.path.join(THIS_DIR, "..", ".."))
    USD = os.path.join(_REPO, "isaac_sim", "assets", "vehicle", "vehicle_v1.usd")
    out.write(f"USD: {USD}\n\n[A] USD 에 박힌 팔 조인트 상태 (deg)\n")
    stage = Usd.Stage.Open(USD)
    arm = {f"joint_{i}" for i in range(1, 7)}
    found = 0
    for prim in stage.Traverse():
        if prim.GetName() in arm and "Revolute" in str(prim.GetTypeName()):
            found += 1
            st = prim.GetAttribute("state:angular:physics:position")
            dr = prim.GetAttribute("drive:angular:physics:targetPosition")
            sv = st.Get() if st and st.IsValid() else None
            dv = dr.Get() if dr and dr.IsValid() else None
            out.write(f"  {prim.GetName():10s} state={sv}  driveTarget={dv}  "
                      f"[{prim.GetPath()}]\n")
    if found == 0:
        out.write("  (팔 revolute 조인트를 못 찾음)\n")

    # --- [B] Isaac Lab joint_pos ---
    out.write("\n[B] Isaac Lab joint_pos\n")
    env = ManagerBasedEnv(DriveEnvCfg())
    robot = env.scene["robot"]
    names = list(robot.data.joint_names)
    before = robot.data.joint_pos[0].detach().cpu().tolist()
    env.reset()
    after = robot.data.joint_pos[0].detach().cpu().tolist()
    out.write(f"  {'joint':14s} {'생성직후(deg)':>16} {'reset후(deg)':>16}\n")
    for i, n in enumerate(names):
        if n.startswith("joint_"):
            out.write(f"  {n:14s} {math.degrees(before[i]):16.1f} "
                      f"{math.degrees(after[i]):16.1f}\n")
    out.write("\n[해석] '생성직후' = USD 가 가진 자세, 'reset후' = 내 init_state 가 강제한 자세.\n")
    out.write("끝\n")
    env.close()
except Exception as exc:  # noqa: BLE001
    import traceback
    out.write(f"\n[ERROR] {type(exc).__name__}: {exc}\n{traceback.format_exc()}")
finally:
    out.close()
    print(f"[inspect_armpose] 리포트: {REPORT}")

simulation_app.close()
