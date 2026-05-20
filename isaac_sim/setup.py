from setuptools import setup
from pathlib import Path

package_name = "isaac_sim"


def _files(pattern: str):
    base = Path(__file__).resolve().parent
    return [str(path) for path in (base / pattern.split("/")[0]).glob(pattern.split("/", 1)[1])]

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/worlds", ["worlds/mars_exploration_world.usd"]),
        (f"share/{package_name}/assets/markers", _files("assets/markers/*.usd")),
        (f"share/{package_name}/assets/textures/Mars", _files("assets/textures/Mars/*")),
        (f"share/{package_name}/worlds/generated", ["worlds/generated/.gitkeep"]),
        (f"share/{package_name}/assets", ["assets/command_center.usd", "assets/bunker.usd"]),
        (f"share/{package_name}/assets/minerals", ["assets/minerals/mineral.usd"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jimjimjinger",
    maintainer_email="jimjimjinger@users.noreply.github.com",
    description="Isaac Sim environment package for Mars rover exploration.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "sim_bridge_node = isaac_sim.sim_bridge_node:main",
        ],
    },
)
