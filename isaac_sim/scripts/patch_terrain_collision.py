#!/usr/bin/env python3
"""기존 generated_terrains USD 에 물리 collider 를 baking 하는 1회성 마이그레이션.

procedural_terrain_generator.py 가 collision baking 을 지원하기 전에 생성된
terrain 자산(terrain_only.usd / rocks_merged.usd)은 collider 가 없어
Isaac Sim 에서 로버가 지형을 통과해 떨어진다. 이 스크립트는 그런 기존 USD 를
열어 collision 만 추가한다 — geometry 는 그대로 둔다.

새로 생성하는 terrain 은 generator 가 이미 collision 을 포함하므로 불필요.

실행 (Isaac Sim 파이썬으로):
    isaac-python isaac_sim/scripts/patch_terrain_collision.py \
        [--terrain-dir isaac_sim/assets/generated_terrains/terrain_00001]
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import argparse
import sys
from pathlib import Path

from pxr import Usd, UsdGeom, UsdPhysics

HERE = Path(__file__).resolve().parent                       # isaac_sim/scripts
DEFAULT_TERRAIN = HERE.parent / "assets/generated_terrains/terrain_00001"


def patch_terrain_usd(path: Path) -> int:
    """terrain_only.usd 의 mesh 에 정적 triangle-mesh collider 추가."""
    stage = Usd.Stage.Open(str(path))
    count = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        UsdPhysics.CollisionAPI.Apply(prim)
        mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mesh_collision.CreateApproximationAttr().Set("none")
        count += 1
    stage.GetRootLayer().Save()
    return count


def patch_rocks_usd(path: Path) -> int:
    """rocks_merged.usd 의 Sphere 에 analytic collider 추가."""
    stage = Usd.Stage.Open(str(path))
    count = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Sphere):
            continue
        UsdPhysics.CollisionAPI.Apply(prim)
        count += 1
    stage.GetRootLayer().Save()
    return count


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--terrain-dir", default=str(DEFAULT_TERRAIN),
                    help="patch 대상 terrain 디렉터리")
    args = ap.parse_args()

    tdir = Path(args.terrain_dir).resolve()
    terrain_usd = tdir / "terrain_only.usd"
    rocks_usd = tdir / "rocks_merged.usd"
    print(f"[patch] 대상: {tdir}")

    if terrain_usd.exists():
        n = patch_terrain_usd(terrain_usd)
        print(f"[patch] {terrain_usd.name}: mesh {n}개에 collider 추가"
              + ("" if n else "  ← mesh 미발견, 확인 필요"))
    else:
        print(f"[patch] 건너뜀 (없음): {terrain_usd}")

    if rocks_usd.exists():
        n = patch_rocks_usd(rocks_usd)
        print(f"[patch] {rocks_usd.name}: 암석 {n}개에 collider 추가"
              + ("" if n else "  ← Sphere 미발견, 확인 필요"))
    else:
        print(f"[patch] 건너뜀 (없음): {rocks_usd}")

    print("[patch] 완료")


if __name__ == "__main__":
    main()
    sys.stdout.flush()  # Isaac Sim 종료(os._exit) 전에 print 버퍼 비우기
    simulation_app.close()
