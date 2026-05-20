#!/usr/bin/env python3
"""
T1 (김현중) — 절차생성 Mars terrain generator.

지금은 1개 샘플 단순 버전 (성선규/T4가 김현중 합류 전 임시 작성).
김현중 합류 시 batch 모드 + 난이도 sweep + USD asset_pool 확장 예정.

출력: I1 계약 (INTERFACE_CONTRACTS.md 참조)
  generated_terrains/terrain_NNNNN/{terrain_only.usd, rocks_merged.usd,
                                    obstacle_grid.npy, heightmap.npy, meta.json}
  generated_terrains/index.json
  + isaac_sim/assets/markers/{mineral_blue.usd, mineral_red.usd,
                              mineral_yellow.usd, basecamp_dome.usd}

사용:
  python3 isaac_sim/scripts/procedural_terrain_generator.py \
      --seed 12345 --terrain-id terrain_00001
"""

import argparse
import json
import os
import time
from pathlib import Path

import numpy as np
from noise import pnoise2
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux
from scipy import ndimage

# ─── 기본 파라미터 (medium 난이도 preset) ──────────────────────────
DEFAULTS = {
    "size_m": [50.0, 50.0],
    "resolution_m": 0.05,
    # Perlin
    "amplitude_m": 3.0,
    "octaves": 4,
    "frequency": 0.08,
    "lacunarity": 2.0,
    "persistence": 0.5,
    # Rocks
    "rocks_count": 80,
    "rocks_size_range": [0.3, 1.5],
    "rocks_min_spacing": 1.0,
    "rocks_slope_thr": 25.0,
    # Minerals
    "minerals_count": 12,
    "minerals_min_spacing": 3.0,
    "minerals_exclude_basecamp_radius": 5.0,
    # Basecamp (Tier 1)
    "basecamp_center": [0.0, 0.0],
    "basecamp_radius": 3.0,
    # Spawn
    "spawn_count": 50,
    # Mesh 다운샘플 (시각용. npy는 풀해상도 유지)
    "mesh_stride": 5,  # 1000/5 = 200 → 200×200 vis mesh
}


# ─── 1. Heightmap (Perlin noise) ────────────────────────────────
def generate_heightmap(W, H, params, res, seed):
    """Perlin 2D noise → heightmap (H, W) float32 (m).

    frequency는 world 미터 단위 (1/freq = wavelength in m).
    예: freq=0.08 → wavelength 12.5m feature.
    """
    base = seed % 1024
    freq = params["frequency"]
    amp = params["amplitude_m"]
    arr = np.zeros((H, W), dtype=np.float32)
    for i in range(H):
        y_world = i * res
        for j in range(W):
            x_world = j * res
            arr[i, j] = pnoise2(
                x_world * freq, y_world * freq,
                octaves=params["octaves"],
                persistence=params["persistence"],
                lacunarity=params["lacunarity"],
                base=base,
            )
    return arr * amp


# ─── 2. Slope (Sobel) ───────────────────────────────────────────
def compute_slope_deg(hm, res):
    gx = ndimage.sobel(hm, axis=1) / (8.0 * res)
    gy = ndimage.sobel(hm, axis=0) / (8.0 * res)
    return np.degrees(np.arctan(np.sqrt(gx * gx + gy * gy)))


# ─── 좌표 변환 헬퍼 ───────────────────────────────────────────────
def sample_at(grid, x, y, origin, res):
    j = int((x - origin[0]) / res)
    i = int((y - origin[1]) / res)
    i = int(np.clip(i, 0, grid.shape[0] - 1))
    j = int(np.clip(j, 0, grid.shape[1] - 1))
    return float(grid[i, j])


# ─── 4. Rocks (rejection sampling) ───────────────────────────────
def place_rocks(slope_deg, origin, res, rng, p):
    rocks = []  # [(x, y, size), ...]
    cx, cy = p["basecamp_center"]
    radius_excl = p["basecamp_radius"] + 1.0
    max_tries = p["rocks_count"] * 20
    for _ in range(max_tries):
        if len(rocks) >= p["rocks_count"]:
            break
        x = rng.uniform(origin[0] + 1, origin[0] + p["size_m"][0] - 1)
        y = rng.uniform(origin[1] + 1, origin[1] + p["size_m"][1] - 1)
        if np.hypot(x - cx, y - cy) < radius_excl:
            continue
        if sample_at(slope_deg, x, y, origin, res) > p["rocks_slope_thr"]:
            continue
        if any(np.hypot(x - rx, y - ry) < p["rocks_min_spacing"]
               for rx, ry, _ in rocks):
            continue
        size = float(rng.uniform(*p["rocks_size_range"]))
        rocks.append((float(x), float(y), size))
    return rocks


# ─── 5. Minerals ────────────────────────────────────────────────
def place_minerals(rocks, origin, res, hm, rng, p):
    minerals = []
    cx, cy = p["basecamp_center"]
    next_id = 1
    max_tries = p["minerals_count"] * 50
    for _ in range(max_tries):
        if len(minerals) >= p["minerals_count"]:
            break
        x = rng.uniform(origin[0] + 2, origin[0] + p["size_m"][0] - 2)
        y = rng.uniform(origin[1] + 2, origin[1] + p["size_m"][1] - 2)
        if np.hypot(x - cx, y - cy) < p["minerals_exclude_basecamp_radius"]:
            continue
        if any(np.hypot(x - rx, y - ry) < rsize + 0.5
               for rx, ry, rsize in rocks):
            continue
        if any(np.hypot(x - m["position"]["x"], y - m["position"]["y"])
               < p["minerals_min_spacing"] for m in minerals):
            continue
        r = rng.random()
        if r < 0.5:
            mtype, value = "blue", 10
        elif r < 0.8:
            mtype, value = "red", 25
        else:
            mtype, value = "yellow", 50
        z = sample_at(hm, x, y, origin, res) + 0.10
        minerals.append({
            "id": next_id,
            "type": mtype,
            "position": {"x": round(float(x), 2),
                         "y": round(float(y), 2),
                         "z": round(float(z), 2)},
            "value": value,
        })
        next_id += 1
    return minerals


# ─── 7. Spawn locations ─────────────────────────────────────────
def place_spawns(hm, slope_deg, obstacle, origin, res, rng, p):
    spawns = []
    cx, cy = p["basecamp_center"]
    max_tries = p["spawn_count"] * 30
    for _ in range(max_tries):
        if len(spawns) >= p["spawn_count"]:
            break
        x = rng.uniform(origin[0] + 2, origin[0] + p["size_m"][0] - 2)
        y = rng.uniform(origin[1] + 2, origin[1] + p["size_m"][1] - 2)
        if np.hypot(x - cx, y - cy) < p["basecamp_radius"] + 0.5:
            continue
        if sample_at(slope_deg, x, y, origin, res) > 15:
            continue
        if sample_at(obstacle, x, y, origin, res) > 0.5:
            continue
        z = sample_at(hm, x, y, origin, res) + 0.18
        yaw = float(rng.uniform(0, 2 * np.pi))
        spawns.append({
            "x": round(float(x), 2),
            "y": round(float(y), 2),
            "z": round(float(z), 2),
            "yaw": round(yaw, 3),
            "group": "default",
        })
    return spawns


# ─── 9. obstacle_grid ────────────────────────────────────────────
def build_obstacle_grid(slope_deg, rocks, origin, res, slope_thr):
    grid = (slope_deg > slope_thr).astype(np.int8)
    for rx, ry, rsize in rocks:
        ci = int((ry - origin[1]) / res)
        cj = int((rx - origin[0]) / res)
        rc = int(rsize / res) + 1
        i0, i1 = max(0, ci - rc), min(grid.shape[0], ci + rc + 1)
        j0, j1 = max(0, cj - rc), min(grid.shape[1], cj + rc + 1)
        ii, jj = np.ogrid[i0:i1, j0:j1]
        mask = (ii - ci) ** 2 + (jj - cj) ** 2 <= rc * rc
        grid[i0:i1, j0:j1] = np.maximum(grid[i0:i1, j0:j1], mask.astype(np.int8))
    return grid


# ─── 10. Difficulty 메트릭 ────────────────────────────────────────
def compute_difficulty(slope_deg, obstacle, rocks, size_m, res):
    area = size_m[0] * size_m[1]
    rock_density = len(rocks) / area
    max_slope = float(slope_deg.max())
    mean_slope = float(slope_deg.mean())
    passable = float((obstacle == 0).mean())
    labeled, n = ndimage.label(obstacle == 0)
    if n > 0:
        sizes = ndimage.sum(obstacle == 0, labeled, range(1, n + 1))
        longest_corr_m = float(np.sqrt(sizes.max()) * res)
    else:
        longest_corr_m = 0.0
    score = float(np.clip(
        0.3 * (rock_density / 0.03)
        + 0.4 * (mean_slope / 30.0)
        + 0.3 * (1 - passable),
        0.0, 1.0,
    ))
    return {
        "score": round(score, 3),
        "rock_density": round(rock_density, 4),
        "max_slope_deg": round(max_slope, 2),
        "mean_slope_deg": round(mean_slope, 2),
        "passable_ratio": round(passable, 3),
        "longest_corridor_m": round(longest_corr_m, 2),
    }


# ─── 8a. USD: terrain_only.usd ────────────────────────────────────
def export_terrain_usd(hm, origin, res, out_path, stride):
    H, W = hm.shape
    is_idx = list(range(0, H, stride))
    js_idx = list(range(0, W, stride))
    rows, cols = len(is_idx), len(js_idx)

    pts_np = np.empty((rows * cols, 3), dtype=np.float32)
    for ri, i in enumerate(is_idx):
        for rj, j in enumerate(js_idx):
            pts_np[ri * cols + rj] = (
                origin[0] + j * res,
                origin[1] + i * res,
                float(hm[i, j]),
            )

    fvi, fvc, face_normals = [], [], []
    for ri in range(rows - 1):
        for rj in range(cols - 1):
            v0 = ri * cols + rj
            v1 = ri * cols + (rj + 1)
            v2 = (ri + 1) * cols + (rj + 1)
            v3 = (ri + 1) * cols + rj
            # Two triangles: (v0, v1, v2) and (v0, v2, v3)
            fvi.extend([v0, v1, v2, v0, v2, v3])
            fvc.extend([3, 3])
            # Per-face normals (CCW → outward = +z-ish)
            for tri in [(v0, v1, v2), (v0, v2, v3)]:
                a, b, c = pts_np[tri[0]], pts_np[tri[1]], pts_np[tri[2]]
                n = np.cross(b - a, c - a)
                ln = np.linalg.norm(n)
                if ln > 0:
                    n = n / ln
                face_normals.append(Gf.Vec3f(float(n[0]), float(n[1]), float(n[2])))

    bbox_min = pts_np.min(axis=0)
    bbox_max = pts_np.max(axis=0)
    extent = [Gf.Vec3f(*bbox_min.tolist()), Gf.Vec3f(*bbox_max.tolist())]
    pts = [Gf.Vec3f(*p.tolist()) for p in pts_np]

    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    # Root는 Xform (reference 시 type 호환). 실제 mesh는 자식 prim으로.
    root = UsdGeom.Xform.Define(stage, "/Terrain")
    mesh = UsdGeom.Mesh.Define(stage, "/Terrain/TerrainMesh")
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexIndicesAttr(fvi)
    mesh.CreateFaceVertexCountsAttr(fvc)
    mesh.CreateExtentAttr(extent)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateNormalsAttr(face_normals)
    mesh.SetNormalsInterpolation("uniform")  # per-face
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.78, 0.45, 0.30)])  # Mars red-orange
    stage.SetDefaultPrim(root.GetPrim())
    stage.GetRootLayer().Save()


# ─── 8b. USD: rocks_merged.usd ────────────────────────────────────
def export_rocks_usd(rocks, hm, origin, res, out_path):
    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Rocks")
    for idx, (x, y, size) in enumerate(rocks):
        z = sample_at(hm, x, y, origin, res) + size * 0.5
        sph = UsdGeom.Sphere.Define(stage, f"/Rocks/rock_{idx:03d}")
        sph.GetRadiusAttr().Set(float(size * 0.5))
        UsdGeom.XformCommonAPI(sph).SetTranslate((float(x), float(y), float(z)))
        sph.GetDisplayColorAttr().Set([Gf.Vec3f(0.45, 0.30, 0.25)])
    stage.SetDefaultPrim(root.GetPrim())
    stage.GetRootLayer().Save()


# ─── Marker assets (1회만 생성) ──────────────────────────────────
def export_mineral_markers(out_dir):
    colors = {
        "blue":   Gf.Vec3f(0.20, 0.40, 0.94),
        "red":    Gf.Vec3f(0.90, 0.24, 0.24),
        "yellow": Gf.Vec3f(0.94, 0.86, 0.20),
    }
    for cname, color in colors.items():
        out = out_dir / f"mineral_{cname}.usd"
        stage = Usd.Stage.CreateNew(str(out))
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        # Root Xform + 자식 Sphere (reference 호환)
        root = UsdGeom.Xform.Define(stage, f"/Mineral_{cname}")
        sph = UsdGeom.Sphere.Define(stage, f"/Mineral_{cname}/Geom")
        sph.GetRadiusAttr().Set(0.20)  # 0.10 → 0.20 (시각 보강)
        sph.GetDisplayColorAttr().Set([color])
        sph.CreateExtentAttr([Gf.Vec3f(-0.2, -0.2, -0.2), Gf.Vec3f(0.2, 0.2, 0.2)])
        stage.SetDefaultPrim(root.GetPrim())
        stage.GetRootLayer().Save()


def export_basecamp_dome(out_path):
    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Basecamp")
    # 패드
    pad = UsdGeom.Cylinder.Define(stage, "/Basecamp/Pad")
    pad.GetRadiusAttr().Set(3.0)
    pad.GetHeightAttr().Set(0.1)
    pad.GetAxisAttr().Set("Z")
    UsdGeom.XformCommonAPI(pad).SetTranslate((0.0, 0.0, 0.05))
    pad.GetDisplayColorAttr().Set([Gf.Vec3f(0.4, 0.4, 0.45)])
    # 돔
    dome = UsdGeom.Sphere.Define(stage, "/Basecamp/Dome")
    dome.GetRadiusAttr().Set(1.5)
    UsdGeom.XformCommonAPI(dome).SetTranslate((0.0, 0.0, 1.5))
    dome.GetDisplayColorAttr().Set([Gf.Vec3f(0.85, 0.85, 0.90)])
    # 안테나
    ant = UsdGeom.Cylinder.Define(stage, "/Basecamp/Antenna")
    ant.GetRadiusAttr().Set(0.05)
    ant.GetHeightAttr().Set(2.5)
    ant.GetAxisAttr().Set("Z")
    UsdGeom.XformCommonAPI(ant).SetTranslate((1.2, 0.0, 2.75))
    ant.GetDisplayColorAttr().Set([Gf.Vec3f(0.7, 0.7, 0.7)])
    stage.SetDefaultPrim(root.GetPrim())
    stage.GetRootLayer().Save()


# ─── Master scene: terrain + rocks + basecamp + minerals + light ───
def compose_world_usd(world_path, terrain_id, meta, marker_dir, terrain_dir):
    """Isaac Sim에서 바로 열 수 있는 master scene 작성.

    터레인/암석/베이스캠프/광물을 reference로 합치고 조명 추가.
    참조 경로는 world_path 위치 기준 상대 경로.
    """
    world_path = Path(world_path)
    world_path.parent.mkdir(parents=True, exist_ok=True)

    def rel(target):
        # USD reference 경로 = master scene 기준 상대 경로 (../assets/...)
        return os.path.relpath(
            str(Path(target).resolve()),
            start=str(world_path.parent.resolve()),
        )

    stage = Usd.Stage.CreateNew(str(world_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # Terrain
    t = stage.DefinePrim("/World/Terrain", "Xform")
    t.GetReferences().AddReference(rel(terrain_dir / "terrain_only.usd"))

    # Rocks
    r = stage.DefinePrim("/World/Rocks", "Xform")
    r.GetReferences().AddReference(rel(terrain_dir / "rocks_merged.usd"))

    # Basecamp
    bc = stage.DefinePrim("/World/Basecamp", "Xform")
    bc.GetReferences().AddReference(rel(marker_dir / "basecamp_dome.usd"))
    bc_center = meta["basecamp"]["center"]
    UsdGeom.XformCommonAPI(bc).SetTranslate(
        (float(bc_center["x"]), float(bc_center["y"]), 0.0)
    )

    # Minerals (id, type, position from meta)
    UsdGeom.Xform.Define(stage, "/World/Minerals")
    for m in meta["minerals"]:
        prim_path = f"/World/Minerals/mineral_{m['id']:02d}_{m['type']}"
        mp = stage.DefinePrim(prim_path, "Xform")
        mp.GetReferences().AddReference(
            rel(marker_dir / f"mineral_{m['type']}.usd")
        )
        UsdGeom.XformCommonAPI(mp).SetTranslate(
            (float(m["position"]["x"]),
             float(m["position"]["y"]),
             float(m["position"]["z"]))
        )

    # 조명: distant sun + dome
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
    sun.CreateIntensityAttr(2500.0)
    sun.CreateAngleAttr(0.53)  # 태양 각경 (deg)
    UsdGeom.XformCommonAPI(sun.GetPrim()).SetRotate(
        Gf.Vec3f(-45.0, 0.0, 30.0)
    )
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Sky")
    dome.CreateIntensityAttr(300.0)
    dome.CreateColorAttr(Gf.Vec3f(0.95, 0.75, 0.6))  # 화성 하늘 톤

    # 메타데이터 (어느 terrain을 합쳤는지)
    stage.GetRootLayer().customLayerData = {
        "terrain_id": terrain_id,
        "generated_at": meta["generated_at"],
    }
    stage.GetRootLayer().Save()


# ─── Main ────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=12345)
    ap.add_argument("--terrain-id", default="terrain_00001")
    ap.add_argument("--out-dir", default="isaac_sim/assets/generated_terrains")
    ap.add_argument("--marker-dir", default="isaac_sim/assets/markers")
    ap.add_argument("--world-path", default="isaac_sim/worlds/mars_exploration_world.usd",
                    help="master scene USD (Isaac Sim에서 바로 열 수 있는 합성본)")
    ap.add_argument("--split", default="train", choices=["train", "holdout"])
    ap.add_argument("--no-compose", action="store_true",
                    help="master scene 갱신 안 함 (테스트용)")
    args = ap.parse_args()

    p = dict(DEFAULTS)
    out_root = Path(args.out_dir)
    out_root.mkdir(parents=True, exist_ok=True)
    terrain_dir = out_root / args.terrain_id
    terrain_dir.mkdir(parents=True, exist_ok=True)

    marker_dir = Path(args.marker_dir)
    marker_dir.mkdir(parents=True, exist_ok=True)
    if not (marker_dir / "mineral_blue.usd").exists():
        print(f"[markers] generating {marker_dir}/mineral_*.usd")
        export_mineral_markers(marker_dir)
    if not (marker_dir / "basecamp_dome.usd").exists():
        print(f"[markers] generating {marker_dir}/basecamp_dome.usd")
        export_basecamp_dome(marker_dir / "basecamp_dome.usd")

    rng = np.random.default_rng(args.seed)
    np.random.seed(args.seed)

    W = int(p["size_m"][0] / p["resolution_m"])
    H = int(p["size_m"][1] / p["resolution_m"])
    origin = [-p["size_m"][0] / 2, -p["size_m"][1] / 2]
    res = p["resolution_m"]

    print(f"[1/10] Heightmap {H}x{W} (Perlin, may take ~30s)...")
    t0 = time.time()
    hm = generate_heightmap(W, H, {
        "frequency": p["frequency"],
        "amplitude_m": p["amplitude_m"],
        "octaves": p["octaves"],
        "lacunarity": p["lacunarity"],
        "persistence": p["persistence"],
    }, res, args.seed)
    print(f"       done in {time.time()-t0:.1f}s "
          f"(min={hm.min():.2f}m, max={hm.max():.2f}m)")

    print("[2/10] Slope analysis (Sobel)...")
    slope_deg = compute_slope_deg(hm, res)

    print(f"[3/10] Basecamp at {p['basecamp_center']} r={p['basecamp_radius']}m")

    print(f"[4/10] Rocks (target {p['rocks_count']})...")
    rocks = place_rocks(slope_deg, origin, res, rng, p)
    print(f"       placed {len(rocks)} rocks")

    print(f"[5/10] Minerals (target {p['minerals_count']})...")
    minerals = place_minerals(rocks, origin, res, hm, rng, p)
    print(f"       placed {len(minerals)} minerals")

    print("[6/10] Physics zones (sand + rocky, Tier 2 stub)...")
    physics_zones = [
        {"type": "sand",
         "polygon": [[-10, -10], [10, -10], [10, 0], [-10, 0]],
         "static_friction": 0.30, "dynamic_friction": 0.25},
        {"type": "rocky",
         "polygon": [[-25, -25], [-10, -25], [-10, -10], [-25, -10]],
         "static_friction": 0.55, "dynamic_friction": 0.50},
    ]

    print("[7/10] obstacle_grid...")
    obstacle = build_obstacle_grid(slope_deg, rocks, origin, res, p["rocks_slope_thr"])

    print(f"[8/10] Spawn locations (target {p['spawn_count']})...")
    spawns = place_spawns(hm, slope_deg, obstacle, origin, res, rng, p)
    print(f"       placed {len(spawns)} spawns")

    print("[9/10] Saving npy + USD...")
    np.save(terrain_dir / "heightmap.npy", hm)
    np.save(terrain_dir / "obstacle_grid.npy", obstacle)
    export_terrain_usd(hm, origin, res,
                       terrain_dir / "terrain_only.usd",
                       stride=p["mesh_stride"])
    export_rocks_usd(rocks, hm, origin, res,
                     terrain_dir / "rocks_merged.usd")

    diff = compute_difficulty(slope_deg, obstacle, rocks, p["size_m"], res)

    print("[10/10] meta.json + index.json...")
    meta = {
        "terrain_id": args.terrain_id,
        "version": "1.0",
        "seed": args.seed,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "size_m": p["size_m"],
        "resolution_m": res,
        "origin": {"x": origin[0], "y": origin[1]},
        "generation_params": {
            "terrain": {
                "type": "perlin",
                "octaves": p["octaves"],
                "frequency": p["frequency"],
                "amplitude_m": p["amplitude_m"],
                "lacunarity": p["lacunarity"],
                "persistence": p["persistence"],
            },
            "rocks": {
                "count": p["rocks_count"],
                "size_range_m": p["rocks_size_range"],
                "min_spacing_m": p["rocks_min_spacing"],
                "slope_threshold_deg": p["rocks_slope_thr"],
                "asset_pool": ["rock_default"],
            },
            "minerals": {
                "count": p["minerals_count"],
                "min_spacing_m": p["minerals_min_spacing"],
                "exclude_basecamp_radius_m": p["minerals_exclude_basecamp_radius"],
                "value_distribution": {
                    "blue":   {"prob": 0.5, "score": 10},
                    "red":    {"prob": 0.3, "score": 25},
                    "yellow": {"prob": 0.2, "score": 50},
                },
            },
            "physics_zones": {
                "type": "noise_based",
                "noise_frequency": 0.04,
                "sand_threshold": 0.3,
            },
        },
        "spawn_locations": spawns,
        "basecamp": {
            "center": {"x": float(p["basecamp_center"][0]),
                       "y": float(p["basecamp_center"][1])},
            "radius": float(p["basecamp_radius"]),
            "marker_usd": "basecamp_dome.usd",
            "visual_footprint_m": [3.0, 3.0],
            "marker_height_m": 5.5,
            "shape": None,
            "entry_points": [],
            "collision_usd_path": None,
        },
        "minerals": minerals,
        "physics_zones": physics_zones,
        "minimap": {
            "grid_size": [25, 25],
            "cell_size_m": 2.0,
            "origin": {"x": origin[0], "y": origin[1]},
        },
        "difficulty": diff,
    }
    with open(terrain_dir / "meta.json", "w") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    index_path = out_root / "index.json"
    if index_path.exists():
        with open(index_path) as f:
            index = json.load(f)
    else:
        index = {"version": "1.0",
                 "generated_at": meta["generated_at"],
                 "terrains": []}
    index["terrains"] = [t for t in index["terrains"]
                         if t["id"] != args.terrain_id]
    index["terrains"].append({
        "id": args.terrain_id,
        "split": args.split,
        "difficulty": diff["score"],
        "seed": args.seed,
    })
    with open(index_path, "w") as f:
        json.dump(index, f, indent=2)

    # Master scene composition
    if not args.no_compose:
        compose_world_usd(Path(args.world_path), args.terrain_id, meta,
                          marker_dir, terrain_dir)

    print(f"\n✅ Generated {args.terrain_id} → {terrain_dir}/")
    for fname in ["terrain_only.usd", "rocks_merged.usd",
                  "obstacle_grid.npy", "heightmap.npy", "meta.json"]:
        size = (terrain_dir / fname).stat().st_size
        print(f"   {fname:24s} {size:>12,} bytes")
    print(f"   ../index.json  ({len(index['terrains'])} terrains)")
    print(f"   markers:        {marker_dir}/{{mineral_*,basecamp_dome}}.usd")
    if not args.no_compose:
        print(f"   master scene:   {args.world_path}")
    print(f"\n📊 Difficulty: score={diff['score']}, "
          f"rocks={len(rocks)}, mean_slope={diff['mean_slope_deg']}°, "
          f"passable={diff['passable_ratio']*100:.0f}%")
    print(f"\n🚀 Isaac Sim 실행: isaac {args.world_path}")


if __name__ == "__main__":
    main()
