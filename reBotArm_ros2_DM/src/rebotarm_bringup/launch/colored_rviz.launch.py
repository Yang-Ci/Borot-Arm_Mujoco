from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    urdf_file = PathJoinSubstitution(
        [bringup_share, "description", "urdf", "ReBot_Arm_DM.urdf"]
    )
    rviz_config = PathJoinSubstitution([bringup_share, "rviz", "rebotarm.rviz"])
    robot_description = ParameterValue(Command(["cat ", urdf_file]), value_type=str)
    publish_demo_joints = LaunchConfiguration("publish_demo_joints")

    demo_joint_state = (
        "{header: auto, "
        "name: [joint1, joint2, joint3, joint4, joint5, joint6, finger_left, finger_right], "
        "position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "publish_demo_joints",
                default_value="true",
                description="Publish a timestamped offline pose for RViz viewing.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="colored_robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", "/rebotarm/joint_states")],
            ),
            TimerAction(
                period=1.0,
                actions=[
                    ExecuteProcess(
                        condition=IfCondition(publish_demo_joints),
                        cmd=[
                            "ros2",
                            "topic",
                            "pub",
                            "--once",
                            "/rebotarm/joint_states",
                            "sensor_msgs/msg/JointState",
                            demo_joint_state,
                        ],
                        output="screen",
                    )
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="colored_rviz",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
