from setuptools import setup

package_name = "isaac_nodes"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name, f"{package_name}.motion_primitives"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jimjimjinger",
    maintainer_email="jimjimjinger@users.noreply.github.com",
    description="Mission, control, battery, and executor nodes for the Mars rover.",
    license="MIT",
)
