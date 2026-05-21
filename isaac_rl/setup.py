from setuptools import setup

package_name = "isaac_rl"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jimjimjinger",
    maintainer_email="jimjimjinger@users.noreply.github.com",
    description="RL package: driving policy, training, and policy loader.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "rl_trainer = isaac_rl.rl_trainer:main",
        ],
    },
)
