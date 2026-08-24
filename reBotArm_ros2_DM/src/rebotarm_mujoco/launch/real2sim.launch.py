from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mujoco_share = FindPackageShare("rebotarm_mujoco")

    model_path = LaunchConfiguration("model_path")
    joint_map_file = LaunchConfiguration("joint_map_file")
    joint_state_topic = LaunchConfiguration("joint_state_topic")
    target_pose_topic = LaunchConfiguration("target_pose_topic")
    target_visible_timeout = LaunchConfiguration("target_visible_timeout")
    open_viewer = LaunchConfiguration("open_viewer")
    sync_hz = LaunchConfiguration("sync_hz")
    smoothing_alpha = LaunchConfiguration("smoothing_alpha")
    stale_timeout = LaunchConfiguration("stale_timeout")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [mujoco_share, "models", "rebotarm_b601_colored.xml"]
                ),
            ),
            DeclareLaunchArgument(
                "joint_map_file",
                default_value=PathJoinSubstitution(
                    [mujoco_share, "config", "joint_map_kinematic.yaml"]
                ),
            ),
            DeclareLaunchArgument("joint_state_topic", default_value="/rebotarm/joint_states"),
            DeclareLaunchArgument("target_pose_topic", default_value="/rebotarm/mujoco/target_pose"),
            DeclareLaunchArgument("target_visible_timeout", default_value="0.7"),
            DeclareLaunchArgument("open_viewer", default_value="true"),
            DeclareLaunchArgument("sync_hz", default_value="60.0"),
            DeclareLaunchArgument("smoothing_alpha", default_value="1.0"),
            DeclareLaunchArgument("stale_timeout", default_value="1.0"),
            Node(
                package="rebotarm_mujoco",
                executable="real2sim_sync",
                name="rebotarm_mujoco_real2sim",
                output="screen",
                parameters=[
                    {
                        "model_path": model_path,
                        "joint_map_file": joint_map_file,
                        "joint_state_topic": joint_state_topic,
                        "target_pose_topic": target_pose_topic,
                        "target_visible_timeout": target_visible_timeout,
                        "open_viewer": open_viewer,
                        "sync_hz": sync_hz,
                        "smoothing_alpha": smoothing_alpha,
                        "stale_timeout": stale_timeout,
                    }
                ],
            ),
        ]
    )
