from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mujoco_share = FindPackageShare("rebotarm_mujoco")

    model_path = LaunchConfiguration("model_path")
    joint_map_file = LaunchConfiguration("joint_map_file")
    target_joint_state_topic = LaunchConfiguration("target_joint_state_topic")
    compare_joint_state_topic = LaunchConfiguration("compare_joint_state_topic")
    sim_joint_state_topic = LaunchConfiguration("sim_joint_state_topic")
    mujoco_tau_g_topic = LaunchConfiguration("mujoco_tau_g_topic")
    sdk_tau_g_topic = LaunchConfiguration("sdk_tau_g_topic")
    tau_g_diff_topic = LaunchConfiguration("tau_g_diff_topic")
    open_viewer = LaunchConfiguration("open_viewer")
    control_hz = LaunchConfiguration("control_hz")
    publish_hz = LaunchConfiguration("publish_hz")
    compare_log_hz = LaunchConfiguration("compare_log_hz")
    sdk_compare_enabled = LaunchConfiguration("sdk_compare_enabled")
    torque_limit = LaunchConfiguration("torque_limit")

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
            DeclareLaunchArgument(
                "target_joint_state_topic",
                default_value="/rebotarm/joint_states",
            ),
            DeclareLaunchArgument(
                "compare_joint_state_topic",
                default_value="/rebotarm/joint_states",
            ),
            DeclareLaunchArgument(
                "sim_joint_state_topic",
                default_value="/rebotarm/mujoco/joint_states",
            ),
            DeclareLaunchArgument(
                "mujoco_tau_g_topic",
                default_value="/rebotarm/mujoco/tau_g",
            ),
            DeclareLaunchArgument(
                "sdk_tau_g_topic",
                default_value="/rebotarm/mujoco/sdk_tau_g",
            ),
            DeclareLaunchArgument(
                "tau_g_diff_topic",
                default_value="/rebotarm/mujoco/tau_g_diff",
            ),
            DeclareLaunchArgument("open_viewer", default_value="true"),
            DeclareLaunchArgument("control_hz", default_value="500.0"),
            DeclareLaunchArgument("publish_hz", default_value="60.0"),
            DeclareLaunchArgument("compare_log_hz", default_value="2.0"),
            DeclareLaunchArgument("sdk_compare_enabled", default_value="true"),
            DeclareLaunchArgument("torque_limit", default_value="18.0"),
            Node(
                package="rebotarm_mujoco",
                executable="mujoco_torque_control",
                name="rebotarm_mujoco_torque_control",
                output="screen",
                parameters=[
                    {
                        "model_path": model_path,
                        "joint_map_file": joint_map_file,
                        "target_joint_state_topic": target_joint_state_topic,
                        "compare_joint_state_topic": compare_joint_state_topic,
                        "sim_joint_state_topic": sim_joint_state_topic,
                        "mujoco_tau_g_topic": mujoco_tau_g_topic,
                        "sdk_tau_g_topic": sdk_tau_g_topic,
                        "tau_g_diff_topic": tau_g_diff_topic,
                        "open_viewer": open_viewer,
                        "control_hz": control_hz,
                        "publish_hz": publish_hz,
                        "compare_log_hz": compare_log_hz,
                        "sdk_compare_enabled": sdk_compare_enabled,
                        "torque_limit": torque_limit,
                    }
                ],
            ),
        ]
    )
