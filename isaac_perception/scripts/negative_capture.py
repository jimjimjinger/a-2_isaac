"""Negative sample 캡쳐 — mineral 없이 terrain 만 + Vehicle 카메라.

목적:
  학습 시 false positive 줄이기 위한 빈 라벨 이미지 (terrain, rocks, rover 만 있고
  mineral 없음). 각 PNG 옆에 빈 .txt 가 같이 생성됨.

사용:
  isaac-python negative_capture.py

조작 — manual_capture.py 와 동일:
  WASD / 마우스 우클릭드래그 / 휠 — viewport 카메라 이동
  Q   — 현재 view PNG + 빈 .txt 저장
  ESC — 종료

scene:
  - terrain_00022.usd (배경만)
  - viewport 카메라 = Vehicle.usd intrinsics 와 동일한 DataCam
  - mineral 0 개 (이게 핵심)
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

# ── argparse pre ──────────────────────────────────────────────────────
from pathlib import Path as _Path
_A2_ROOT_DEFAULT = os.environ.get("A2_ISAAC_ROOT") or str(
    _Path(__file__).resolve().parents[2]
)
_ap = argparse.ArgumentParser(add_help=False)
_ap.add_argument("--output", type=str,
                 default=f"{_A2_ROOT_DEFAULT}/isaac_perception/dataset/manual/negative")
_ap.add_argument("--resolution", type=str, default="1280x720")
args, _ = _ap.parse_known_args()

# ── SimulationApp ────────────────────────────────────────────────────
os.chdir(tempfile.mkdtemp(prefix="neg_cap_"))
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
PKG_ROOT = Path(_A2_ROOT_DEFAULT)
TERRAIN_USD = PKG_ROOT / "isaac_sim/worlds/terrain_00022.usd"
VEHICLE_USD = PKG_ROOT.parent / "Vehicle.usd"


def build_scene(stage):
    print("[1/2] terrain 로드 …")
    terrain_prim = stage.DefinePrim("/World/Terrain", "Xform")
    terrain_prim.GetReferences().AddReference(str(TERRAIN_USD))
    for _ in range(20):
        simulation_app.update()

    print("[2/2] Vehicle.usd intrinsics 적용 카메라 생성 …")
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
        print("  Vehicle camera intrinsics 복사 완료")
    xf = UsdGeom.Xformable(cam_prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(2.0, 0.0, 1.8))
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    for _ in range(10):
        simulation_app.update()
    return cam_path


def _count_existing(folder: Path):
    if not folder.exists():
        return 0
    pngs = list(folder.glob("negative_*.png"))
    if not pngs:
        return 0
    mx = -1
    for p in pngs:
        try:
            n = int(p.stem.split("_")[1])
            mx = max(mx, n)
        except ValueError:
            continue
    return mx + 1


def main():
    out_dir = Path(args.output).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    res_w, res_h = (int(v) for v in args.resolution.split("x"))

    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    cam_path = build_scene(stage)

    try:
        import omni.kit.viewport.utility as vp_util
        vp = vp_util.get_active_viewport()
        vp.camera_path = cam_path
        print(f"\n[viewport] active camera → {cam_path}")
    except Exception as e:
        print(f"\n[WARN] viewport active camera 설정 실패: {e}")

    camera = Camera(prim_path=cam_path, resolution=(res_w, res_h), frequency=30)
    world.reset()
    camera.initialize()
    camera.add_distance_to_image_plane_to_frame()

    start_idx = _count_existing(out_dir)
    captured = [start_idx]
    quit_flag = [False]

    def on_kb(event, *_a, **_k):
        if event.type != carb.input.KeyboardEventType.KEY_PRESS:
            return True
        if event.input == carb.input.KeyboardInput.Q:
            rgba = camera.get_rgba()
            if rgba is None or rgba.size == 0:
                print("  [Q] frame empty, skip")
                return True
            bgr = cv2.cvtColor(rgba[..., :3], cv2.COLOR_RGB2BGR)
            idx = captured[0]
            png_path = out_dir / f"negative_{idx:04d}.png"
            txt_path = out_dir / f"negative_{idx:04d}.txt"
            cv2.imwrite(str(png_path), bgr)
            txt_path.write_text("")    # 빈 라벨 = negative sample
            print(f"  [Q] saved → {png_path.name}  + empty .txt")
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
    print("  Negative sample 캡쳐 (mineral 없음)")
    print("=" * 60)
    print("  Q   : PNG + 빈 .txt 저장")
    print("  ESC : 종료")
    print("  카메라 조작 — manual_capture 와 동일")
    print(f"  output : {out_dir}")
    print(f"  start#  : {captured[0]:04d}")
    print(f"  ⚠ 다양한 시점 권장: 평지, 바위 클로즈업, 그림자, 멀리, 위에서 등")
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


if __name__ == "__main__":
    main()
