"""Manual YOLO data capture — GUI viewport 에서 직접 카메라 조작 + Q 키로 저장.

사용:
  isaac-python manual_capture.py
  isaac-python manual_capture.py --output /tmp/my_captures

조작:
  WASD / 마우스 우클릭 드래그 / 휠 — viewport 카메라 이동/회전 (Isaac Sim 기본)
  Q   — 현재 viewport view 를 출력 폴더에 PNG 로 저장
  ESC — 종료

scene:
  - terrain_00022.usd (배경)
  - mineral_blue / red / yellow 각 1개씩 고정 배치
  - viewport 활성 카메라 = Vehicle.usd 의 intrinsics 와 동일한 DataCam
"""
from __future__ import annotations

import argparse
import os
import sys
import tempfile
from pathlib import Path

try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

# ── argparse pre (headless 여부) ─────────────────────────────────────
_ap = argparse.ArgumentParser(add_help=False)
_ap.add_argument("--output", type=str,
                 default="/home/rokey/dev_ws/rover_ws/src/a2_isaac/isaac_perception/dataset/manual")
_ap.add_argument("--resolution", type=str, default="1280x720")
args, _ = _ap.parse_known_args()

# ── SimulationApp ────────────────────────────────────────────────────
os.chdir(tempfile.mkdtemp(prefix="manual_cap_"))
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import carb.input
import cv2
import numpy as np
import omni.appwindow
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom

from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera


# ── 자산 경로 ──────────────────────────────────────────────────────
PKG_ROOT = Path("/home/rokey/dev_ws/rover_ws/src/a2_isaac")
TERRAIN_USD = PKG_ROOT / "isaac_sim/worlds/terrain_00022.usd"
VEHICLE_USD = Path("/home/rokey/dev_ws/rover_ws/src/Vehicle.usd")
MINERAL_ASSETS_DIR = PKG_ROOT / "isaac_sim/assets/markers/tier2_mineral"

MINERAL_PLACEMENTS = [
    (0, "blue_mineral",   MINERAL_ASSETS_DIR / "blue_mineral.usd",   (4.5, -1.0, 1.0)),
    (1, "green_gas",      MINERAL_ASSETS_DIR / "green_gas.usd",      (4.5,  0.0, 1.0)),
    (2, "yellow_mineral", MINERAL_ASSETS_DIR / "yellow_mineral.usd", (4.5,  1.0, 1.0)),
]


def build_scene(stage):
    print("[1/3] terrain 로드 …")
    terrain_prim = stage.DefinePrim("/World/Terrain", "Xform")
    terrain_prim.GetReferences().AddReference(str(TERRAIN_USD))
    for _ in range(20):
        simulation_app.update()

    print("[2/3] mineral 3 개 고정 spawn …")
    for cls_id, name, asset_path, pos in MINERAL_PLACEMENTS:
        prim_path = f"/World/Minerals/m_{cls_id}_{name}"
        container = stage.DefinePrim(prim_path, "Xform")
        xf = UsdGeom.Xformable(container)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
        inner = stage.DefinePrim(f"{prim_path}/Asset", "Xform")
        inner.GetReferences().AddReference(str(asset_path))
        for _ in range(3):
            simulation_app.update()
        print(f"  {name:14s} @ ({pos[0]:+.2f},{pos[1]:+.2f},{pos[2]:+.2f})")

    print("[3/3] Vehicle.usd intrinsics 적용 카메라 생성 …")
    veh_ref = "/World/_VehicleTemp"
    stage.DefinePrim(veh_ref, "Xform").GetReferences().AddReference(str(VEHICLE_USD))
    for _ in range(8):
        simulation_app.update()
    veh_cam_prim = None
    for prim in Usd.PrimRange(stage.GetPrimAtPath(veh_ref)):
        if prim.GetTypeName() == "Camera":
            veh_cam_prim = prim
            break

    cam_path = "/World/DataCam"
    UsdGeom.Camera.Define(stage, cam_path)
    cam_prim = stage.GetPrimAtPath(cam_path)
    if veh_cam_prim:
        for attr in ("focalLength", "horizontalAperture", "verticalAperture", "clippingRange"):
            src = veh_cam_prim.GetAttribute(attr)
            if src and src.IsValid():
                t = (Sdf.ValueTypeNames.Float2 if attr == "clippingRange"
                     else Sdf.ValueTypeNames.Float)
                cam_prim.CreateAttribute(attr, t).Set(src.Get())
        print(f"  Vehicle camera intrinsics 복사 완료")
    xf = UsdGeom.Xformable(cam_prim)
    xf.ClearXformOpOrder()
    # 초기 위치: minerals 정면 약간 위/뒤
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(2.0, 0.0, 1.8))
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    for _ in range(10):
        simulation_app.update()
    return cam_path


def main():
    out_dir = Path(args.output).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    res_w, res_h = (int(v) for v in args.resolution.split("x"))

    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    cam_path = build_scene(stage)

    # viewport 활성 카메라 = DataCam
    try:
        import omni.kit.viewport.utility as vp_util
        vp = vp_util.get_active_viewport()
        vp.camera_path = cam_path
        print(f"\n[viewport] active camera → {cam_path}")
    except Exception as e:
        print(f"\n[WARN] viewport active camera 설정 실패: {e}")

    # Camera wrapper — 캡쳐용
    camera = Camera(prim_path=cam_path, resolution=(res_w, res_h), frequency=30)
    world.reset()
    camera.initialize()
    camera.add_distance_to_image_plane_to_frame()

    # 키보드 listener
    start_idx = _count_existing(out_dir)
    captured = [start_idx]
    quit_flag = [False]

    def on_kb(event, *args_, **kwargs_):
        if event.type != carb.input.KeyboardEventType.KEY_PRESS:
            return True
        if event.input == carb.input.KeyboardInput.Q:
            rgba = camera.get_rgba()
            if rgba is None or rgba.size == 0:
                print(f"  [Q] frame empty, skip")
                return True
            bgr = cv2.cvtColor(rgba[..., :3], cv2.COLOR_RGB2BGR)
            idx = captured[0]
            out_path = out_dir / f"manual_{idx:04d}.png"
            cv2.imwrite(str(out_path), bgr)
            print(f"  [Q] saved → {out_path}  ({bgr.shape[1]}x{bgr.shape[0]})")
            captured[0] += 1
        elif event.input == carb.input.KeyboardInput.ESCAPE:
            print("[ESC] 종료 요청")
            quit_flag[0] = True
        return True

    app_window = omni.appwindow.get_default_app_window()
    input_iface = carb.input.acquire_input_interface()
    sub_id = input_iface.subscribe_to_keyboard_events(app_window.get_keyboard(), on_kb)

    world.play()
    print("\n" + "=" * 60)
    print("  수동 캡쳐 도구 시작")
    print("=" * 60)
    print(f"  Q   : 현재 viewport view 저장")
    print(f"  ESC : 종료")
    print(f"  WASD / 마우스 우클릭드래그 / 휠 : viewport 카메라 조작")
    print(f"")
    print(f"  output : {out_dir}")
    print(f"  start#  : {captured[0]:04d}")
    print("=" * 60 + "\n")

    try:
        while simulation_app.is_running() and not quit_flag[0]:
            world.step(render=True)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            input_iface.unsubscribe_to_keyboard_events(
                app_window.get_keyboard(), sub_id
            )
        except Exception:
            pass
        print(f"\n[done] captured this session = {captured[0] - start_idx}  "
              f"(folder now has {captured[0]} files)")
        simulation_app.close()


def _count_existing(folder: Path):
    if not folder.exists():
        return 0
    pngs = list(folder.glob("manual_*.png"))
    if not pngs:
        return 0
    # 가장 큰 인덱스 + 1
    mx = -1
    for p in pngs:
        try:
            n = int(p.stem.split("_")[1])
            mx = max(mx, n)
        except ValueError:
            continue
    return mx + 1


if __name__ == "__main__":
    main()
