from setuptools import setup

package_name = "isaac_ai"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/ai.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jimjimjinger",
    maintainer_email="jimjimjinger@users.noreply.github.com",
    description="Vision AI, object pose estimation, and RL policy nodes.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vision_ai_node = isaac_ai.vision_ai_node:main",
            "object_pose_node = isaac_ai.object_pose_node:main",
            "rl_policy_node = isaac_ai.rl_policy_node:main",
        ],
    },
)

