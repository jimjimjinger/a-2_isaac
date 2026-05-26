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
            "launch/localization.launch.py",
            "launch/integrated_localization.launch.py",
            "launch/mvp.launch.py",
            "launch/mvp_multi.launch.py",
            "launch/rqt_views.launch.py",
            "launch/rqt_views_multi.launch.py",
        ]),
        (f"share/{package_name}/rviz", [
            "rviz/localization_map.rviz",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jimjimjinger",
    maintainer_email="jimjimjinger@users.noreply.github.com",
    description="Launch package for the Mars rover exploration system.",
    license="MIT",
)
