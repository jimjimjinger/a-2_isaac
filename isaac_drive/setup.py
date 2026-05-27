from setuptools import setup

package_name = "isaac_drive"

setup(
    name=package_name,
    version="0.0.1",
    packages=[
        package_name,
        f"{package_name}.navigation",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jimjimjinger",
    maintainer_email="jimjimjinger@users.noreply.github.com",
    description="Drive package: autonomous/manual driving flow + rover drive execution + navigation primitives.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "coverage_node = isaac_drive.coverage_node:main",
            "odom_to_estimated_pose = isaac_drive.odom_to_estimated_pose:main",
            "raycast_relay_node = isaac_drive.raycast_relay_node:main",
            "raycast_map_viewer = isaac_drive.raycast_map_viewer:main",
            "rl_avoid_node = isaac_drive.rl_avoid_node:main",
        ],
    },
)
