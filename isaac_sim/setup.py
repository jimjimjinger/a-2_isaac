from setuptools import setup
from pathlib import Path

package_name = "isaac_sim"


def _files(pattern: str):
    # colcon requires data_files sources to be relative to the package root.
    base = Path(__file__).resolve().parent
    head, tail = pattern.split("/", 1)
    return [str(path.relative_to(base)) for path in sorted((base / head).glob(tail))]

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jimjimjinger",
    maintainer_email="jimjimjinger@users.noreply.github.com",
    description="Isaac Sim environment package for Mars rover exploration.",
    license="MIT",
    # entry_points 비움 — sim_bridge_node (mock service) 삭제.
    # isaac_sim 패키지 자체는 worlds/assets share install 만 담당.
)
