from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = LaunchConfiguration("model")
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("rebotarm_moveit_demos"),
            "config",
            PythonExpression(
                [
                    "'pick_place_rs.yaml' if '",
                    model,
                    "'.lower() == 'rs' else 'pick_place.yaml'",
                ]
            ),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value="dm",
                description="Robot model used by the active MoveIt demo: dm or rs",
            ),
            Node(
                package="rebotarm_moveit_demos",
                executable="pick_place",
                name="pick_place",
                output="screen",
                parameters=[config_file],
            )
        ]
    )
