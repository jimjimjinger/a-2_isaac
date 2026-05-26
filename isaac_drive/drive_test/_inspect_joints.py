"""vehicle_v1.usd — m0609 팔 조인트의 USD 에 박힌 상태/드라이브 값 점검 (일회성).

Isaac Lab 은 USD 조인트 자세를 안 읽고 init_state.joint_pos(없으면 0)로
덮어쓴다.  USD 가 HOME 을 어디에(joint state? drive target? link xform?)
박아놨는지 확인해, init_state 를 맞춘다.

    /home/rokey/dev_ws/venv/isaaclab/bin/python _inspect_joints.py
    cat /tmp/joint_state.txt
"""

import os

from isaaclab.app import AppLauncher

app_launcher = AppLauncher({"headless": True})
simulation_app = app_launcher.app

from pxr import Usd  # noqa: E402

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
USD_PATH = os.path.join(_REPO_ROOT, "isaac_sim", "assets", "vehicle", "vehicle_v1.usd")
REPORT = "/tmp/joint_state.txt"

out = open(REPORT, "w", buffering=1)
out.write("===== m0609 팔 조인트 USD 상태 점검 =====\n")

stage = Usd.Stage.Open(USD_PATH)
arm_joints = {f"joint_{i}" for i in range(1, 7)}

for prim in stage.Traverse():
    name = prim.GetName()
    path = str(prim.GetPath())
    if name in arm_joints and "m0609" in path:
        out.write(f"\n=== {path}  ({prim.GetTypeName()}) ===\n")
        for attr in prim.GetAttributes():
            an = attr.GetName()
            # 자세/드라이브 관련 속성만.
            if any(k in an.lower() for k in ("position", "target", "state", "rotat", "xform")):
                authored = attr.HasAuthoredValue()
                out.write(f"  {'[A]' if authored else '[ ]'} {an} = {attr.Get()}\n")
        # link(=child body)의 xform 도 — HOME 이 xform 에 구워졌는지 확인.

# 팔 링크 xform 확인.
out.write("\n===== m0609 링크 xformOp =====\n")
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "/m0609/link_" in path or path.endswith("/m0609/base"):
        ops = prim.GetAttribute("xformOpOrder")
        out.write(f"\n{path}\n")
        for attr in prim.GetAttributes():
            an = attr.GetName()
            if an.startswith("xformOp:") and attr.HasAuthoredValue():
                out.write(f"  {an} = {attr.Get()}\n")

out.write("\n===== 끝 =====\n")
out.close()
print(f"[inspect_joints] 리포트: {REPORT}")
simulation_app.close()
