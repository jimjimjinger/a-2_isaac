from setuptools import setup

package_name = "isaac_manipulation"

setup(
    name=package_name,
    version="0.0.1",
    packages=[
        package_name,
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jimjimjinger",
    maintainer_email="jimjimjinger@users.noreply.github.com",
    description="Manipulation package: M0609 DLS-IK arm executor + /grasp/command snap.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "arm_executor_node = isaac_manipulation.arm_executor_node:main",
        ],
    },
)
