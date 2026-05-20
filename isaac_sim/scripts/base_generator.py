from __future__ import annotations

from typing import Any, Sequence

import numpy as np


# 모든 생성 맵에 공통으로 들어가는 베이스캠프 오브젝트를 만든다.
def build_base_layout(
    *,
    seed: int,
    terrain_size_m: float = 100.0,
    center: Sequence[float] = (0.0, 0.0, 0.0),
) -> list[dict[str, Any]]:
    rng = np.random.default_rng(int(seed) + 37)
    offset = 10.0 + float(rng.uniform(-1.5, 1.5))
    bunker_shift = 14.0 + float(rng.uniform(-2.0, 2.0))

    command_center = {
        "type": "base",
        "name": "command_center",
        "prim_path": "/World/Base/command_center",
        "asset_path": "assets/command_center.usd",
        "position": [round(float(center[0]), 3), round(float(center[1]), 3), round(float(center[2]), 3)],
        "rotation": [0.0, 0.0, 0.0],
        "scale": [1.0, 1.0, 1.0],
    }
    bunker = {
        "type": "base",
        "name": "bunker",
        "prim_path": "/World/Base/bunker",
        "asset_path": "assets/bunker.usd",
        "position": [round(float(center[0] + offset), 3), round(float(center[1] - bunker_shift), 3), round(float(center[2]), 3)],
        "rotation": [0.0, 0.0, 90.0],
        "scale": [1.0, 1.0, 1.0],
    }
    return [command_center, bunker]
