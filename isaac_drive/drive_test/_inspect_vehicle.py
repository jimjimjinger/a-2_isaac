"""vehicle_v1.usd 구조 점검 — 조인트/링크/articulation root 확인용 (일회성).

avoid_test 의 레이캐스트/Ackermann 설정을 vehicle_v1.usd 에 맞추려면
실제 조인트·링크 이름이 필요하다.  headless 로 USD 를 열어 덤프한다.

    /home/rokey/dev_ws/venv/isaaclab/bin/python _inspect_vehicle.py
"""

import os

from isaaclab.app import AppLauncher

app_launcher = AppLauncher({"headless": True})
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom  # noqa: E402

_REPO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..")
)
USD_PATH = os.path.join(_REPO_ROOT, "isaac_sim", "assets", "vehicle", "vehicle_v1.usd")
REPORT = "/tmp/vehicle_struct.txt"

out = open(REPORT, "w", buffering=1)
out.write("===== vehicle_v1.usd 구조 점검 =====\n")
out.write(f"USD: {USD_PATH}\n")

stage = Usd.Stage.Open(USD_PATH)
out.write(f"defaultPrim: {stage.GetDefaultPrim().GetPath()}\n\n")

joints, links, arts = [], [], []
for prim in stage.Traverse():
    path = str(prim.GetPath())
    tname = str(prim.GetTypeName())
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        arts.append(path)
    if "Joint" in tname:
        joints.append((path, tname))
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        links.append(path)

out.write(f"[ArticulationRoot] ({len(arts)})\n")
for a in arts:
    out.write(f"  {a}\n")

out.write(f"\n[RigidBody 링크] ({len(links)})\n")
for l in links:
    out.write(f"  {l}\n")

out.write(f"\n[Joint] ({len(joints)})\n")
for p, t in joints:
    out.write(f"  {t:26s} {p}\n")

out.write("\n===== 점검 끝 =====\n")
out.close()
print(f"[inspect] 리포트 저장: {REPORT}")
simulation_app.close()
