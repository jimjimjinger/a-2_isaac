"""m0609 팔 — USD 정적 link 자세 vs Isaac Lab 물리 자세 비교 (일회성).

[C1] USD baked link 위치 (physics OFF, pxr) — base_link 기준 상대
[C2] Isaac Lab link 위치 (physics ON) — reset후 / 200step후, base_link 기준 상대

C1 ≈ C2(reset후)  → USD 정적 자세 = 물리 자세 (일치)
C1 ≠ C2(reset후)  → USD link xform 과 joint state 불일치 (USD 자체 문제)
C2 reset후 ≠ 200step후 → 중력으로 팔 처짐
"""

import argparse
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

from pxr import Usd, UsdGeom  # noqa: E402
import torch  # noqa: E402

from isaaclab.envs import ManagerBasedEnv  # noqa: E402

from drive_env_cfg import DriveEnvCfg  # noqa: E402
from rover_vehicle import keep_arm_folded  # noqa: E402

REPORT = "/tmp/armlinks.txt"
LINKS = ["base_link", "link_2", "link_4", "link_6", "tool0"]

out = open(REPORT, "w", buffering=1)
try:
    # --- [C1] USD baked (physics OFF) ---
    _REPO = os.path.abspath(os.path.join(THIS_DIR, "..", ".."))
    USD = os.path.join(_REPO, "isaac_sim", "assets", "vehicle", "vehicle_v1.usd")
    stage = Usd.Stage.Open(USD)
    usd_pos = {}
    for prim in stage.Traverse():
        n = prim.GetName()
        if n in LINKS and "/m0609/" in str(prim.GetPath()) and n not in usd_pos:
            m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            t = m.ExtractTranslation()
            usd_pos[n] = (t[0], t[1], t[2])
    out.write("[C1] USD baked (physics OFF) — base_link 기준 상대 위치 (m)\n")
    base = usd_pos.get("base_link", (0.0, 0.0, 0.0))
    for n in LINKS:
        if n in usd_pos:
            p = usd_pos[n]
            r = (p[0] - base[0], p[1] - base[1], p[2] - base[2])
            out.write(f"  {n:10s} rel=({r[0]:+.3f}, {r[1]:+.3f}, {r[2]:+.3f})\n")

    # --- [C2] Isaac Lab (physics ON) ---
    env = ManagerBasedEnv(DriveEnvCfg())
    robot = env.scene["robot"]
    bn = list(robot.data.body_names)
    idx = {n: bn.index(n) for n in LINKS if n in bn}
    env.reset()
    keep_arm_folded(robot)
    robot.update(0.0)

    def rel_pos():
        bp = robot.data.body_pos_w[0]
        b = bp[idx["base_link"]]
        return {n: (bp[i] - b).cpu().tolist() for n, i in idx.items()}

    con = rel_pos()
    zero = torch.zeros((1, env.action_manager.total_action_dim), device=env.device)
    for _ in range(200):
        env.step(zero)
        keep_arm_folded(robot)
    robot.update(0.0)
    aft = rel_pos()

    out.write("\n[C2] Isaac Lab (physics ON) — base_link 기준 상대 위치 (m)\n")
    for n in LINKS:
        if n in idx:
            c, a = con[n], aft[n]
            out.write(f"  {n:10s} reset후=({c[0]:+.3f}, {c[1]:+.3f}, {c[2]:+.3f})  "
                      f"200step후=({a[0]:+.3f}, {a[1]:+.3f}, {a[2]:+.3f})\n")

    out.write("\n[해석]\n")
    out.write("  C1 ≈ C2(reset후)  → USD 정적 자세 = 물리 자세 (일치)\n")
    out.write("  C1 ≠ C2(reset후)  → USD link xform 과 joint state 불일치\n")
    out.write("  C2 reset후 ≠ 200step후 → 중력으로 팔 처짐\n")
    env.close()
except Exception as exc:  # noqa: BLE001
    import traceback
    out.write(f"\n[ERROR] {type(exc).__name__}: {exc}\n{traceback.format_exc()}")
finally:
    out.close()
    print(f"[inspect_armlinks] 리포트: {REPORT}")

simulation_app.close()
