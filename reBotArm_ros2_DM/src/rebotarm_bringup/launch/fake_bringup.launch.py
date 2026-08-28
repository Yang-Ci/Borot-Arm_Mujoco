from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    arm_namespace = LaunchConfiguration("arm_namespace")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    max_joint_speed = LaunchConfiguration("max_joint_speed")
    use_rviz = LaunchConfiguration("use_rviz")

    urdf_file = PathJoinSubstitution(
        [bringup_share, "description", "urdf", "ReBot_Arm_DM.urdf"]
    )
    rviz_urdf_compat = PathJoinSubstitution(
        [bringup_share, "launch", "rviz_urdf_compat.py"]
    )
    rviz_config = PathJoinSubstitution([bringup_share, "rviz", "rebotarm.rviz"])
    robot_description = ParameterValue(
        Command(["python3 ", rviz_urdf_compat, " ", urdf_file]), value_type=str
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            DeclareLaunchArgument("max_joint_speed", default_value="1.8"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            Node(
                package="rebotarmcontroller",
                executable="FakeReBotArmDriver",
                name="fake_rebotarm_driver",
                output="screen",
                parameters=[
                    {
                        "arm_namespace": arm_namespace,
                        "joint_state_rate": joint_state_rate,
                        "max_joint_speed": max_joint_speed,
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", ["/", arm_namespace, "/joint_states"])],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
