from setuptools import find_packages, setup

package_name = "isaac_localization"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    package_data={
        package_name: [
            "maps/*.npy",
            "maps/*.yaml",
            "assets/*.md",
            "assets/*.obj",
            "assets/*.png",
            "assets/*.usda",
            "tools/*.py",
        ],
    },
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "matplotlib"],
    zip_safe=True,
    maintainer="jimjimjinger",
    maintainer_email="jimjimjinger@users.noreply.github.com",
    description="GPS-less rover localization with wheel odometry, IMU integration, TRN, and EKF fusion.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "wheel_odom_node = isaac_localization.sensors.wheel_odom:main",
            "imu_integrator_node = isaac_localization.sensors.imu_integrator:main",
            "trn_node = isaac_localization.trn:main",
            "ekf_fusion_node = isaac_localization.ekf_fusion:main",
            "localization_node = isaac_localization.localization_node:main",
        ],
    },
)
