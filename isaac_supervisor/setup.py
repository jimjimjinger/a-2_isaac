import os
from glob import glob
from setuptools import setup

package_name = "isaac_supervisor"


def _web_data_files():
    """Recursively include web/templates and web/static under share/<pkg>/web/.
    setup.py's data_files needs a flat list of (dest_dir, [files]).
    """
    pairs = []
    for root, _dirs, files in os.walk("web"):
        if not files:
            continue
        dest = os.path.join("share", package_name, root)
        pairs.append((dest, [os.path.join(root, f) for f in files]))
    return pairs


setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        *_web_data_files(),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jimjimjinger",
    maintainer_email="jimjimjinger@users.noreply.github.com",
    description="Mission supervisor: orchestration + battery monitoring.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "battery_monitor_node = isaac_supervisor.battery_monitor_node:main",
            "mission_manager_node = isaac_supervisor.mission_manager_node:main",
            "mission_web_node = isaac_supervisor.mission_web_node:main",
        ],
    },
)
