from __future__ import annotations

import math
from typing import Any, Sequence

import numpy as np
from scipy import ndimage

try:
    from pxr import Gf, Sdf, UsdGeom, UsdShade  # type: ignore
except Exception:  # pragma: no cover
    Gf = Sdf = UsdGeom = UsdShade = None  # type: ignore


def _box_blur(arr: np.ndarray, passes: int = 1) -> np.ndarray:
    out = arr.astype(np.float32, copy=True)
    for _ in range(passes):
        out = (
            out
            + np.roll(out, 1, axis=0)
            + np.roll(out, -1, axis=0)
            + np.roll(out, 1, axis=1)
            + np.roll(out, -1, axis=1)
        ) / 5.0
    return out


def _grid(map_size: int) -> tuple[np.ndarray, np.ndarray]:
    axis = np.linspace(-1.0, 1.0, map_size, dtype=np.float32)
    return np.meshgrid(axis, axis)


def generate_heightmap(
    seed: int,
    *,
    map_size: int = 1000,
    height_scale: float = 8.0,
    base_hill_count: int = 22,
    crater_count: int = 14,
    deep_crater_range: tuple[int, int] = (1, 2),
) -> np.ndarray:
    # 씨드 기반으로 언덕, 크레이터, 잔노이즈가 있는 높이맵을 만든다.
    rng = np.random.default_rng(int(seed))
    xx, yy = _grid(map_size)
    height = np.zeros((map_size, map_size), dtype=np.float32)

    for _ in range(base_hill_count):
        cx, cy = rng.uniform(-1.0, 1.0, size=2)
        amp = rng.uniform(0.12, 0.40)
        sigma = rng.uniform(0.14, 0.34)
        dist = (xx - cx) ** 2 + (yy - cy) ** 2
        height += (amp * np.exp(-dist / (2.0 * sigma * sigma))).astype(np.float32)

    height += 0.11 * np.sin(1.7 * math.pi * xx + 0.8) * np.cos(1.3 * math.pi * yy - 0.4)
    height += 0.06 * np.sin(3.1 * math.pi * (xx + yy) + 1.2)

    for _ in range(crater_count):
        cx, cy = rng.uniform(-0.85, 0.85, size=2)
        radius = rng.uniform(0.05, 0.14)
        depth = rng.uniform(0.16, 0.48)
        dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
        bowl = -depth * np.exp(-(dist**2) / (2.0 * (radius * 0.55) ** 2))
        rim = depth * 0.33 * np.exp(-((dist - radius) ** 2) / (2.0 * (radius * 0.20) ** 2))
        height += (bowl + rim).astype(np.float32)

    deep_craters = int(rng.integers(deep_crater_range[0], deep_crater_range[1] + 1))
    for _ in range(deep_craters):
        cx, cy = rng.uniform(-0.75, 0.75, size=2)
        radius = rng.uniform(0.08, 0.18)
        depth = rng.uniform(0.55, 1.05)
        dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
        bowl = -depth * np.exp(-(dist**2) / (2.0 * (radius * 0.48) ** 2))
        rim = depth * 0.42 * np.exp(-((dist - radius) ** 2) / (2.0 * (radius * 0.17) ** 2))
        height += (bowl + rim).astype(np.float32)

    height = _box_blur(height, passes=3)
    height += rng.normal(0.0, 0.028, height.shape).astype(np.float32)
    height = _box_blur(height, passes=2)
    height = 0.72 * height + 0.28 * _box_blur(height, passes=1)
    height -= float(height.min())
    maximum = float(height.max())
    if maximum <= 0.0:
        maximum = 1.0
    height = height / maximum
    return (height * float(height_scale)).astype(np.float32)


def compute_slope_deg(heightmap: np.ndarray, resolution_m: float) -> np.ndarray:
    # 높이 기울기를 도(degree) 단위 경사도로 바꾼다.
    gx = ndimage.sobel(heightmap, axis=1) / (8.0 * resolution_m)
    gy = ndimage.sobel(heightmap, axis=0) / (8.0 * resolution_m)
    return np.degrees(np.arctan(np.sqrt(gx * gx + gy * gy))).astype(np.float32)


def sample_at(grid: np.ndarray, x: float, y: float, origin: Sequence[float], resolution_m: float) -> float:
    j = int((x - float(origin[0])) / resolution_m)
    i = int((y - float(origin[1])) / resolution_m)
    i = int(np.clip(i, 0, grid.shape[0] - 1))
    j = int(np.clip(j, 0, grid.shape[1] - 1))
    return float(grid[i, j])


def build_obstacle_grid(
    slope_deg: np.ndarray,
    rocks: Sequence[dict[str, Any]],
    origin: Sequence[float],
    resolution_m: float,
    slope_thr_deg: float,
) -> np.ndarray:
    # 급경사와 바위 영역을 이동 불가 셀로 표시한다.
    grid = (slope_deg > slope_thr_deg).astype(np.int8)
    for rock in rocks:
        x, y = rock["position"][0], rock["position"][1]
        radius = float(rock.get("radius", rock["scale"][0] * 0.5))
        ci = int((y - float(origin[1])) / resolution_m)
        cj = int((x - float(origin[0])) / resolution_m)
        rc = int(radius / resolution_m) + 1
        i0, i1 = max(0, ci - rc), min(grid.shape[0], ci + rc + 1)
        j0, j1 = max(0, cj - rc), min(grid.shape[1], cj + rc + 1)
        ii, jj = np.ogrid[i0:i1, j0:j1]
        mask = (ii - ci) ** 2 + (jj - cj) ** 2 <= rc * rc
        grid[i0:i1, j0:j1] = np.maximum(grid[i0:i1, j0:j1], mask.astype(np.int8))
    return grid


def compute_difficulty(
    slope_deg: np.ndarray,
    obstacle_grid: np.ndarray,
    rocks: Sequence[dict[str, Any]],
    map_size_m: float,
    resolution_m: float,
) -> dict[str, float]:
    # 생성된 지형의 난이도를 한눈에 볼 수 있게 요약한다.
    area = map_size_m * map_size_m
    rock_density = len(rocks) / area
    max_slope = float(slope_deg.max())
    mean_slope = float(slope_deg.mean())
    passable = float((obstacle_grid == 0).mean())
    labeled, n = ndimage.label(obstacle_grid == 0)
    if n > 0:
        sizes = ndimage.sum(obstacle_grid == 0, labeled, range(1, n + 1))
        longest_corridor_m = float(np.sqrt(float(np.max(sizes))) * resolution_m)
    else:
        longest_corridor_m = 0.0
    score = float(
        np.clip(
            0.3 * (rock_density / 0.03) + 0.4 * (mean_slope / 30.0) + 0.3 * (1.0 - passable),
            0.0,
            1.0,
        )
    )
    return {
        "score": round(score, 3),
        "rock_density": round(rock_density, 4),
        "max_slope_deg": round(max_slope, 2),
        "mean_slope_deg": round(mean_slope, 2),
        "passable_ratio": round(passable, 3),
        "longest_corridor_m": round(longest_corridor_m, 2),
    }


def build_terrain_mesh_data(
    heightmap: np.ndarray,
    origin: Sequence[float],
    resolution_m: float,
    *,
    mesh_stride: int = 5,
) -> dict[str, Any]:
    # 높이맵을 줄여서 메쉬 점과 삼각형 인덱스로 바꾼다.
    if mesh_stride <= 0:
        raise ValueError("mesh_stride must be positive")

    h, w = heightmap.shape
    row_idx = list(range(0, h, mesh_stride))
    col_idx = list(range(0, w, mesh_stride))
    rows, cols = len(row_idx), len(col_idx)

    points = np.empty((rows * cols, 3), dtype=np.float32)
    for ri, i in enumerate(row_idx):
        for rj, j in enumerate(col_idx):
            points[ri * cols + rj] = (
                float(origin[0]) + j * resolution_m,
                float(origin[1]) + i * resolution_m,
                float(heightmap[i, j]),
            )

    face_vertex_indices: list[int] = []
    face_vertex_counts: list[int] = []
    normals: list[tuple[float, float, float]] = []

    for ri in range(rows - 1):
        for rj in range(cols - 1):
            v0 = ri * cols + rj
            v1 = ri * cols + (rj + 1)
            v2 = (ri + 1) * cols + (rj + 1)
            v3 = (ri + 1) * cols + rj
            for tri in ((v0, v1, v2), (v0, v2, v3)):
                face_vertex_indices.extend(tri)
                face_vertex_counts.append(3)
                a, b, c = points[tri[0]], points[tri[1]], points[tri[2]]
                normal = np.cross(b - a, c - a)
                length = float(np.linalg.norm(normal))
                if length > 0.0:
                    normal = normal / length
                normals.append((float(normal[0]), float(normal[1]), float(normal[2])))

    bbox_min = points.min(axis=0)
    bbox_max = points.max(axis=0)
    return {
        "points": points,
        "face_vertex_indices": face_vertex_indices,
        "face_vertex_counts": face_vertex_counts,
        "normals": normals,
        "extent": [bbox_min.astype(np.float32), bbox_max.astype(np.float32)],
        "stride": mesh_stride,
        "rows": rows,
        "cols": cols,
    }


def author_terrain_stage(
    stage: Any,
    terrain_mesh: dict[str, Any],
    *,
    terrain_prim_path: str = "/Terrain",
    mesh_prim_path: str = "/Terrain/TerrainMesh",
    material_prim_path: str = "/Terrain/Materials/MarsSoil",
    material_shader_name: str = "MarsSoilShader",
    texture_paths: dict[str, str] | None = None,
) -> Any:
    # 지형 prim을 작성하고 Mars 재질을 메쉬에 바인딩한다.
    if UsdGeom is None or Gf is None or Sdf is None:
        raise RuntimeError("pxr is not available in this environment")

    root = UsdGeom.Xform.Define(stage, terrain_prim_path)
    mesh = UsdGeom.Mesh.Define(stage, mesh_prim_path)

    points = [Gf.Vec3f(*point.tolist()) for point in terrain_mesh["points"]]
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexIndicesAttr(list(terrain_mesh["face_vertex_indices"]))
    mesh.CreateFaceVertexCountsAttr(list(terrain_mesh["face_vertex_counts"]))
    mesh.CreateExtentAttr([Gf.Vec3f(*terrain_mesh["extent"][0].tolist()), Gf.Vec3f(*terrain_mesh["extent"][1].tolist())])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)

    primvars = UsdGeom.PrimvarsAPI(mesh)
    st_values = []
    rows = int(terrain_mesh["rows"])
    cols = int(terrain_mesh["cols"])
    for row in range(rows):
        for col in range(cols):
            st_values.append(
                Gf.Vec2f(
                    (col / float(max(cols - 1, 1))) * 48.0,
                    (row / float(max(rows - 1, 1))) * 48.0,
                )
            )
    primvars.CreatePrimvar(
        "st",
        Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.vertex,
    ).Set(st_values)

    stage.SetDefaultPrim(root.GetPrim())
    return mesh
