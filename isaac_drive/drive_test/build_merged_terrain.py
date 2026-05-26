# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""terrain_00022 의 지형 USD + 바위 USD 를 '단일 메시' 하나로 병합한다.

레이캐스터는 메시 1개만 스캔한다.  terrain_only.usd(지형)와
rocks_merged.usd(바위 80개)는 서로 다른 Mesh 라 따로 두면 레이캐스터가
하나만 본다.  이 스크립트는 두 USD 의 **모든 삼각형을 월드 좌표로 구워
하나의 Mesh** 로 이어붙여 terrain_00022_new.usdc 를 만든다.

  · 모양은 그대로 (삼각형을 좌표 변환해 이어붙이기만 함)
  · 결과는 Mesh 1개 → 레이캐스터가 지형+바위를 한 번에 스캔
  · 충돌(PhysicsCollisionAPI, trimesh)도 구워 넣어 차량이 올라설 수 있음

실행 (Isaac Sim 안 띄워도 됨 — isaacsim 의 standalone USD 라이브러리 사용):
    /home/rokey/dev_ws/venv/isaaclab/bin/python build_merged_terrain.py

⚠️ 입력 USD(terrain_00022 폴더)는 drive_test 밖 자산 — 읽기만 한다.
   출력 terrain_00022_new.usdc 는 drive_test 안에 생성한다.
"""

from __future__ import annotations

import glob
import os
import sys


# ---------------------------------------------------------------------------
# pxr(USD) 부트스트랩 — isaacsim 의 omni.usd.libs 를 경로에 올리고 재실행.
# (LD_LIBRARY_PATH 는 프로세스 시작 전에 박혀야 해서 os.execv 로 재실행한다)
# ---------------------------------------------------------------------------
def _bootstrap_pxr() -> None:
    try:
        import pxr  # noqa: F401
        return
    except ImportError:
        pass
    if os.environ.get("_PXR_BOOTSTRAPPED") == "1":
        sys.exit("[ERROR] pxr 를 불러올 수 없음 — isaacsim omni.usd.libs 확인 필요")
    libs = sorted(glob.glob(
        f"{sys.prefix}/lib/python*/site-packages/isaacsim/extscache/omni.usd.libs-*"))
    if not libs:
        sys.exit("[ERROR] omni.usd.libs 를 찾지 못함 — isaacsim 패키지 필요")
    usd_root = libs[-1]
    py_lib = os.path.join(sys.base_prefix, "lib")
    os.environ["_PXR_BOOTSTRAPPED"] = "1"
    os.environ["LD_LIBRARY_PATH"] = os.pathsep.join(
        [f"{usd_root}/bin", py_lib, os.environ.get("LD_LIBRARY_PATH", "")])
    os.environ["PYTHONPATH"] = os.pathsep.join(
        [usd_root, os.environ.get("PYTHONPATH", "")])
    os.execv(sys.executable, [sys.executable] + sys.argv)


_bootstrap_pxr()

import numpy as np  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt  # noqa: E402


# ---------------------------------------------------------------------------
# 경로
# ---------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
# drive_test → parents[2] = a2_isaac (repo 루트)
_REPO = os.path.abspath(os.path.join(_HERE, "..", ".."))
_TERRAIN_DIR = os.path.join(_REPO, "isaac_sim", "assets",
                            "generated_terrains", "terrain_00022")
TERRAIN_USD = os.path.join(_TERRAIN_DIR, "terrain_only.usd")
ROCKS_USD = os.path.join(_TERRAIN_DIR, "rocks_merged.usd")
OUT_USD = os.path.join(_HERE, "terrain_00022_new.usdc")


# ---------------------------------------------------------------------------
# 헬퍼
# ---------------------------------------------------------------------------
def _mat_to_np(m: Gf.Matrix4d) -> np.ndarray:
    """Gf.Matrix4d → (4,4) numpy (USD 행벡터 규약 그대로)."""
    return np.array([[m[i][j] for j in range(4)] for i in range(4)], dtype=np.float64)


def _transform_points(pts: np.ndarray, mat: np.ndarray) -> np.ndarray:
    """로컬 점들을 월드 좌표로 (USD 규약: p' = p · M)."""
    ph = np.empty((len(pts), 4), dtype=np.float64)
    ph[:, :3] = pts
    ph[:, 3] = 1.0
    w = ph @ mat
    return w[:, :3] / w[:, 3:4]


def _triangulate(fvi: np.ndarray, fvc: np.ndarray) -> np.ndarray:
    """faceVertexIndices/Counts → (ntri, 3) 삼각형 인덱스 (fan triangulation)."""
    fvi = np.asarray(fvi, dtype=np.int64)
    fvc = np.asarray(fvc, dtype=np.int64)
    if len(fvc) == 0:
        return np.zeros((0, 3), dtype=np.int64)
    if np.all(fvc == 3):                       # 이미 삼각형
        return fvi.reshape(-1, 3)
    if np.all(fvc == 4):                       # 사각형 → 삼각형 2개
        q = fvi.reshape(-1, 4)
        return np.vstack([q[:, [0, 1, 2]], q[:, [0, 2, 3]]])
    tris = []                                  # 일반 n-각형 fan
    off = 0
    for c in fvc:
        c = int(c)
        for k in range(1, c - 1):
            tris.append((fvi[off], fvi[off + k], fvi[off + k + 1]))
        off += c
    return np.array(tris, dtype=np.int64).reshape(-1, 3)


def collect_meshes(usd_path: str, label: str):
    """USD 안의 모든 Mesh 를 (로컬점, fvi, fvc, 월드행렬) 로 모은다."""
    if not os.path.isfile(usd_path):
        sys.exit(f"[ERROR] 입력 USD 없음: {usd_path}")
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        sys.exit(f"[ERROR] USD 열기 실패: {usd_path}")
    up = UsdGeom.GetStageUpAxis(stage)
    meshes = []
    # 인스턴스 프록시까지 순회 (바위가 instanceable 참조여도 geometry 를 읽도록)
    rng = Usd.PrimRange(stage.GetPseudoRoot(),
                        Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate))
    for prim in rng:
        if not prim.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(prim)
        pts = m.GetPointsAttr().Get()
        fvi = m.GetFaceVertexIndicesAttr().Get()
        fvc = m.GetFaceVertexCountsAttr().Get()
        if not pts or fvi is None or fvc is None or len(pts) == 0:
            continue
        mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        meshes.append((np.array(pts, dtype=np.float64),
                       np.array(fvi, dtype=np.int64),
                       np.array(fvc, dtype=np.int64),
                       _mat_to_np(mat)))
    print(f"  {label:24s} : Mesh {len(meshes):3d}개  (upAxis={up})")
    return meshes, up


# ---------------------------------------------------------------------------
# 메인
# ---------------------------------------------------------------------------
def main() -> None:
    print("=" * 64)
    print("  terrain_00022  지형 + 바위  →  단일 메시 병합")
    print("-" * 64)
    print("소스 USD 로드:")
    terr, up_t = collect_meshes(TERRAIN_USD, "terrain_only.usd")
    rock, up_r = collect_meshes(ROCKS_USD, "rocks_merged.usd")
    if not terr and not rock:
        sys.exit("[ERROR] 병합할 메시가 없음")

    # 모든 메시를 월드 좌표로 변환 → 삼각형화 → 인덱스 오프셋해서 이어붙임
    all_pts, all_tris, voff = [], [], 0
    for src, up in ((terr, up_t), (rock, up_r)):
        for pts, fvi, fvc, mat in src:
            wp = _transform_points(pts, mat)
            if str(up) == "Y":                 # Y-up → Z-up 보정 (보통 안 씀)
                wp = np.column_stack([wp[:, 0], -wp[:, 2], wp[:, 1]])
            tris = _triangulate(fvi, fvc)
            if len(tris) == 0:
                continue
            all_tris.append(tris + voff)
            all_pts.append(wp)
            voff += len(wp)

    points = np.vstack(all_pts).astype(np.float32)
    tris = np.vstack(all_tris).astype(np.int64)
    fvi_flat = tris.reshape(-1).astype(np.int32)
    fvc_flat = np.full(len(tris), 3, dtype=np.int32)
    lo, hi = points.min(axis=0), points.max(axis=0)
    print("-" * 64)
    print(f"  병합 결과 : 정점 {len(points):,}개 · 삼각형 {len(tris):,}개")
    print(f"  bbox      : x[{lo[0]:.1f},{hi[0]:.1f}] "
          f"y[{lo[1]:.1f},{hi[1]:.1f}] z[{lo[2]:.2f},{hi[2]:.2f}]")

    # --- 단일 Mesh USD 작성 ---
    if os.path.exists(OUT_USD):
        os.remove(OUT_USD)
    stage = Usd.Stage.CreateNew(OUT_USD)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    mesh = UsdGeom.Mesh.Define(stage, "/World/CombinedTerrain")
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(fvi_flat))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(fvc_flat))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateExtentAttr([Gf.Vec3f(*lo.tolist()), Gf.Vec3f(*hi.tolist())])

    # 충돌 — trimesh 정적 콜라이더 (차량이 지형·바위 위를 달릴 수 있도록)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    mca = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
    mca.CreateApproximationAttr(UsdPhysics.Tokens.none)

    stage.GetRootLayer().Save()
    size_mb = os.path.getsize(OUT_USD) / 1e6
    print("-" * 64)
    print(f"  저장 완료 : {OUT_USD}  ({size_mb:.1f} MB)")
    print("=" * 64)


if __name__ == "__main__":
    main()
