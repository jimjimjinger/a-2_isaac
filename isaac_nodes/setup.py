from setuptools import setup

package_name = "isaac_nodes"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/nodes.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jimjimjinger",
    maintainer_email="jimjimjinger@users.noreply.github.com",
    description="State collection, task management, robot execution, and logging nodes.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "state_collector_node = isaac_nodes.state_collector_node:main",
            "task_manager_node = isaac_nodes.task_manager_node:main",
            "robot_executor_node = isaac_nodes.robot_executor_node:main",
            "logger_node = isaac_nodes.logger_node:main",
        ],
    },
)

