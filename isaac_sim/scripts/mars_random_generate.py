from __future__ import annotations

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from scene_builder import build_mars_scene


# 단일 화성 장면을 생성하는 실행 진입점.
def main() -> None:
    build_mars_scene(seed=1)
    simulation_app.close()


if __name__ == "__main__":
    main()
