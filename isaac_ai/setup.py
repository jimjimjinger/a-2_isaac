from setuptools import setup

package_name = "isaac_ai"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name, f"{package_name}.vision", f"{package_name}.rl"],
    data_files=[("share/ament_index/resource_index/packages", [f"resource/{package_name}"]), (f"share/{package_name}", ["package.xml"])],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jimjimjinger",
    maintainer_email="jimjimjinger@users.noreply.github.com",
    description="AI package for mineral perception and reinforcement-learning-based driving decisions.",
    license="MIT",
)
