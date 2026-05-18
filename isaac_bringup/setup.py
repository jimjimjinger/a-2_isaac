from setuptools import setup

package_name = "isaac_bringup"

setup(
    name=package_name,
    version="0.0.1",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", [
            "launch/full_system.launch.py",
            "launch/sim.launch.py",
            "launch/ai.launch.py",
            "launch/navigation.launch.py",
            "launch/nodes.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jimjimjinger",
    maintainer_email="jimjimjinger@users.noreply.github.com",
    description="Launch package for the Mars rover exploration system.",
    license="MIT",
)
