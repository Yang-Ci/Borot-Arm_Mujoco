from setuptools import find_packages, setup

package_name = "rebotarmcontroller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="ROS 2 controller node for reBotArm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "reBotArmController = rebotarmcontroller.rebotarm_controller:main",
            "FakeReBotArmDriver = rebotarmcontroller.fake_driver:main",
            "GravityCompensation = rebotarmcontroller.examples.gravity_compensation:main",
            "GripperControl = rebotarmcontroller.examples.gripper_control:main",
            "MoveTo = rebotarmcontroller.examples.move_to:main",
            "MoveToPose = rebotarmcontroller.examples.move_to_pose:main",
        ],
    },
)
