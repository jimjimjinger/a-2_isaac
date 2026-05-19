from setuptools import setup

package_name = "isaac_nodes"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name, f"{package_name}.manipulation_primitives"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jimjimjinger",
    maintainer_email="jimjimjinger@users.noreply.github.com",
    description="Mission management, battery monitoring, and robot arm execution nodes for the Mars rover.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission_manager_node = isaac_nodes.mission_manager_node:main",
            "battery_monitor_node = isaac_nodes.battery_monitor_node:main",
            "arm_executor_node = isaac_nodes.arm_executor_node:main",
        ],
    },
)
