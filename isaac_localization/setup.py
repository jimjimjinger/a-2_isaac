from setuptools import setup

package_name = "isaac_localization"

setup(
    name=package_name,
    version="0.0.1",
    packages=[
        package_name,
        f"{package_name}.sensors",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jimjimjinger",
    maintainer_email="jimjimjinger@users.noreply.github.com",
    description="Localization: TRN + EKF multi-sensor fusion (Wheel/IMU/Sun).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "localization_node = isaac_localization.localization_node:main",
        ],
    },
)
