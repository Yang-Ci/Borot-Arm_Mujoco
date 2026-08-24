from glob import glob
from setuptools import find_packages, setup

package_name = "rebotarm_mujoco"
mesh_files = glob("../rebotarm_bringup/description/meshes/*.STL") + glob(
    "../rebotarm_bringup/description/meshes/*.stl"
)

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/models", glob("models/*.xml")),
        (f"share/{package_name}/meshes", mesh_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="MuJoCo real-to-sim synchronization tools for reBotArm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "real2sim_sync = rebotarm_mujoco.real2sim_sync:main",
            "joint_slider_gui = rebotarm_mujoco.joint_slider_gui:main",
            "mujoco_torque_control = rebotarm_mujoco.mujoco_torque_control:main",
            "mujoco_physics_grasp = rebotarm_mujoco.mujoco_physics_grasp:main",
            "sim_task_server = rebotarm_mujoco.sim_task_server:main",
            "sim_rgb_camera = rebotarm_mujoco.sim_rgb_camera:main",
            "sim_color_detector = rebotarm_mujoco.sim_color_detector:main",
        ],
    },
)
