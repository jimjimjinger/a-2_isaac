from setuptools import setup

package_name = "isaac_supervisor"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jimjimjinger",
    maintainer_email="jimjimjinger@users.noreply.github.com",
    description="Mission supervisor: orchestration + battery monitoring.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission_manager_node = isaac_supervisor.mission_manager_node:main",
            "battery_monitor_node = isaac_supervisor.battery_monitor_node:main",
        ],
    },
)
