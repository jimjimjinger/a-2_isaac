"""Rebuild the vehicle PhysX USD with explicit contact-report APIs."""
from __future__ import annotations

import argparse

from isaaclab.app import AppLauncher


def main() -> None:
    parser = argparse.ArgumentParser(description="Rebuild vehicle_v3_physx.usd")
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()

    app_launcher = AppLauncher(args)
    _ = app_launcher.app

    from usd_physx_setup import main as rebuild_main

    rebuild_main()

    from pxr import PhysxSchema, Usd

    usd_path = "/home/kimi/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/vehicle/vehicle_v3_physx.usd"
    stage = Usd.Stage.Open(usd_path)
    for path in ["/Root/Vehicle/rover/Body", "/Root/Vehicle/rover/FL_Drive", "/Root/Vehicle/m0609/tool0"]:
        prim = stage.GetPrimAtPath(path)
        print(f"[verify] {path}: has_contact_report={prim.HasAPI(PhysxSchema.PhysxContactReportAPI)}")


if __name__ == "__main__":
    main()
