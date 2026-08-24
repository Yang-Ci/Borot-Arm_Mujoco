from glob import glob
from setuptools import setup

package_name = "rebotarm_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/description/urdf", glob("description/urdf/*.urdf")),
        (f"share/{package_name}/description/meshes", glob("description/meshes/*")),
        (
            f"share/{package_name}/description/meshes_b601_gripper",
            glob("description/meshes_b601_gripper/*"),
        ),
        (
            f"share/{package_name}/description/meshes_rs",
            glob("description/meshes_rs/*"),
        ),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Launch, configuration, and description files for reBotArm.",
    license="Apache-2.0",
)
