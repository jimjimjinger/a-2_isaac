from __future__ import annotations

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from scene_builder import build_mars_scene


# 검증이나 데이터셋 생성을 위한 소량 배치 생성기.
OVERWRITE = True


def main() -> None:
    for seed in range(1, 51):
        build_mars_scene(seed=seed, overwrite=OVERWRITE)
    simulation_app.close()


if __name__ == "__main__":
    main()
