from setuptools import setup

package_name = "isaac_drive"

setup(
    name=package_name,
    version="0.0.1",
    packages=[
        package_name,
        f"{package_name}.primitives",
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
            "drive_manager_node = isaac_drive.drive_manager_node:main",
            "mobile_base_executor_node = isaac_drive.mobile_base_executor_node:main",
            "coverage_node = isaac_drive.coverage_node:main",
        ],
    },
)
