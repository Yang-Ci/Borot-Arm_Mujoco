from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mujoco_share = FindPackageShare("rebotarm_mujoco")

    arm_namespace = LaunchConfiguration("arm_namespace")
    model_path = LaunchConfiguration("model_path")
    joint_map_file = LaunchConfiguration("joint_map_file")
    joint_state_topic = LaunchConfiguration("joint_state_topic")
    target_pose_topic = LaunchConfiguration("target_pose_topic")
    command_hz = LaunchConfiguration("command_hz")
    record_hz = LaunchConfiguration("record_hz")
    ik_iterations = LaunchConfiguration("ik_iterations")
    ik_tolerance = LaunchConfiguration("ik_tolerance")
    ik_damping = LaunchConfiguration("ik_damping")
    ik_orientation_weight = LaunchConfiguration("ik_orientation_weight")
    ik_orientation_tolerance = LaunchConfiguration("ik_orientation_tolerance")

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
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
            DeclareLaunchArgument("command_hz", default_value="60.0"),
            DeclareLaunchArgument("record_hz", default_value="30.0"),
            DeclareLaunchArgument("ik_iterations", default_value="360"),
            DeclareLaunchArgument("ik_tolerance", default_value="0.004"),
            DeclareLaunchArgument("ik_damping", default_value="0.035"),
            DeclareLaunchArgument("ik_orientation_weight", default_value="0.75"),
            DeclareLaunchArgument("ik_orientation_tolerance", default_value="0.07"),
            Node(
                package="rebotarm_mujoco",
                executable="sim_task_server",
                name="rebotarm_mujoco_sim_task_server",
                output="screen",
                parameters=[
                    {
                        "arm_namespace": arm_namespace,
                        "model_path": model_path,
                        "joint_map_file": joint_map_file,
                        "joint_state_topic": joint_state_topic,
                        "target_pose_topic": target_pose_topic,
                        "command_hz": command_hz,
                        "record_hz": record_hz,
                        "ik_iterations": ik_iterations,
                        "ik_tolerance": ik_tolerance,
                        "ik_damping": ik_damping,
                        "ik_orientation_weight": ik_orientation_weight,
                        "ik_orientation_tolerance": ik_orientation_tolerance,
                    }
                ],
            ),
        ]
    )
