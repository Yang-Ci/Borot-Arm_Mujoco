from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joint_state_topic = LaunchConfiguration("joint_state_topic")
    publish_hz = LaunchConfiguration("publish_hz")
    arm_namespace = LaunchConfiguration("arm_namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument("joint_state_topic", default_value="/rebotarm/joint_states"),
            DeclareLaunchArgument("publish_hz", default_value="30.0"),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            Node(
                package="rebotarm_mujoco",
                executable="joint_slider_gui",
                name="rebotarm_joint_slider_gui",
                output="screen",
                parameters=[
                    {
                        "joint_state_topic": joint_state_topic,
                        "publish_hz": publish_hz,
                        "arm_namespace": arm_namespace,
                    }
                ],
            ),
        ]
    )
