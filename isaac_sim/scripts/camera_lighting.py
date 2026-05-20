from __future__ import annotations

import math
from typing import Any, Sequence

import numpy as np

try:
    from pxr import Gf, UsdGeom, UsdLux  # type: ignore
except Exception:  # pragma: no cover
    Gf = UsdGeom = UsdLux = None  # type: ignore


def _normalize(vector: np.ndarray) -> np.ndarray:
    length = float(np.linalg.norm(vector))
    if length <= 1e-8:
        return vector
    return vector / length


def look_at_matrix(camera_position: Sequence[float], target_position: Sequence[float], up: Sequence[float] = (0.0, 0.0, 1.0)) -> np.ndarray:
    # 카메라가 원점을 보도록 간단한 변환 행렬을 만든다.
    camera_position = np.asarray(camera_position, dtype=np.float64)
    target_position = np.asarray(target_position, dtype=np.float64)
    up = np.asarray(up, dtype=np.float64)

    forward = _normalize(target_position - camera_position)
    right = _normalize(np.cross(forward, up))
    if float(np.linalg.norm(right)) <= 1e-8:
        up = np.asarray((0.0, 1.0, 0.0), dtype=np.float64)
        right = _normalize(np.cross(forward, up))
    true_up = _normalize(np.cross(right, forward))

    matrix = np.eye(4, dtype=np.float64)
    matrix[0, :3] = right
    matrix[1, :3] = true_up
    matrix[2, :3] = -forward
    matrix[:3, 3] = camera_position
    return matrix


def setup_lights(stage: Any, *, world_path: str = "/World") -> None:
    # 지형이 잘 보이도록 강한 태양광과 부드러운 하늘광을 넣는다.
    if UsdLux is None or UsdGeom is None:
        raise RuntimeError("pxr is not available in this environment")

    sun = UsdLux.DistantLight.Define(stage, f"{world_path}/Lights/Sun")
    sun.CreateIntensityAttr(4000.0)
    sun.CreateAngleAttr(0.8)
    dome = UsdLux.DomeLight.Define(stage, f"{world_path}/Lights/Sky")
    dome.CreateIntensityAttr(300.0)
    dome.CreateColorAttr(Gf.Vec3f(0.96, 0.78, 0.60))


def setup_camera(
    stage: Any,
    *,
    map_size_m: float,
    world_path: str = "/World",
    seed: int = 1,
) -> dict[str, Any]:
    # 고정 오버뷰 카메라와 그 주변 보조 조명을 배치한다.
    if UsdGeom is None or Gf is None:
        raise RuntimeError("pxr is not available in this environment")

    camera_distance = map_size_m * 1.15
    camera_height = map_size_m * 0.72
    jitter = float((seed % 7) * 0.15)
    camera_position = np.array([camera_distance, -camera_distance + jitter, camera_height], dtype=np.float64)
    target = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    matrix = look_at_matrix(camera_position, target)

    rig = UsdGeom.Xform.Define(stage, f"{world_path}/CameraRig")
    rig_prim = rig.GetPrim()
    xform = UsdGeom.Xformable(rig_prim)
    op = xform.AddTransformOp()
    mat = Gf.Matrix4d(1.0)
    for row in range(4):
        for col in range(4):
            mat[row][col] = float(matrix[row, col])
    op.Set(mat)

    camera = UsdGeom.Camera.Define(stage, f"{world_path}/CameraRig/Camera")
    camera.GetHorizontalApertureAttr().Set(20.955)
    camera.GetVerticalApertureAttr().Set(15.2908)
    camera.GetFocalLengthAttr().Set(24.0)

    camera_light = UsdLux.SphereLight.Define(stage, f"{world_path}/Lights/CameraLight")
    camera_light.CreateIntensityAttr(1200.0)
    camera_light.CreateRadiusAttr(0.12)
    camera_light.CreateColorAttr(Gf.Vec3f(1.0, 0.92, 0.80))
    camera_light_prim = camera_light.GetPrim()
    camera_light_xform = UsdGeom.Xformable(camera_light_prim)
    light_op = camera_light_xform.AddTranslateOp()
    light_position = camera_position + np.array([0.0, 0.0, -1.5], dtype=np.float64)
    light_op.Set(Gf.Vec3d(float(light_position[0]), float(light_position[1]), float(light_position[2])))

    return {
        "camera_prim_path": f"{world_path}/CameraRig/Camera",
        "camera_position": [float(x) for x in camera_position.tolist()],
        "target": [0.0, 0.0, 0.0],
    }
