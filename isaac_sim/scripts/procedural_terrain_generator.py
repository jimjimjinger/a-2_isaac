#!/usr/bin/env python3
from __future__ import annotations

import argparse

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from scene_builder import build_mars_scene


# 단일 생성과 배치 생성을 모두 지원하는 CLI 래퍼.
def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--terrain-id", default=None)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--batch", action="store_true")
    parser.add_argument("--batch-count", type=int, default=50)
    parser.add_argument("--start-seed", type=int, default=1)
    args = parser.parse_args()

    if args.batch:
        for seed in range(args.start_seed, args.start_seed + args.batch_count):
            build_mars_scene(seed=seed, terrain_id=f"terrain_{seed:05d}", overwrite=args.overwrite)
        simulation_app.close()
        return

    build_mars_scene(seed=args.seed, terrain_id=args.terrain_id, overwrite=True)
    simulation_app.close()


if __name__ == "__main__":
    main()
