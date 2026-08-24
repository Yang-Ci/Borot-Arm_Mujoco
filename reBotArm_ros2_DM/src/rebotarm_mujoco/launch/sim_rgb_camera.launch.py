from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mujoco_share = FindPackageShare("rebotarm_mujoco")

    arm_namespace = LaunchConfiguration("arm_namespace")
    model_path = LaunchConfiguration("model_path")
    joint_map_file = LaunchConfiguration("joint_map_file")
    joint_state_topic = LaunchConfiguration("joint_state_topic")
    object_states_topic = LaunchConfiguration("object_states_topic")
    target_pose_topic = LaunchConfiguration("target_pose_topic")
    target_visible_timeout = LaunchConfiguration("target_visible_timeout")
    render_target_marker = LaunchConfiguration("render_target_marker")
    camera_name = LaunchConfiguration("camera_name")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    frame_id = LaunchConfiguration("frame_id")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    publish_hz = LaunchConfiguration("publish_hz")
    virtual_grasp_enabled = LaunchConfiguration("virtual_grasp_enabled")

    return LaunchDescription(
        [
            SetEnvironmentVariable("MUJOCO_GL", "egl"),
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
            DeclareLaunchArgument("object_states_topic", default_value=""),
            DeclareLaunchArgument("target_pose_topic", default_value="/rebotarm/mujoco/target_pose"),
            DeclareLaunchArgument("target_visible_timeout", default_value="0.7"),
            DeclareLaunchArgument("render_target_marker", default_value="true"),
            DeclareLaunchArgument("camera_name", default_value="overhead_rgb"),
            DeclareLaunchArgument("image_topic", default_value=""),
            DeclareLaunchArgument("camera_info_topic", default_value=""),
            DeclareLaunchArgument("frame_id", default_value="overhead_rgb_frame"),
            DeclareLaunchArgument("width", default_value="320"),
            DeclareLaunchArgument("height", default_value="240"),
            DeclareLaunchArgument("publish_hz", default_value="8.0"),
            DeclareLaunchArgument("virtual_grasp_enabled", default_value="true"),
            Node(
                package="rebotarm_mujoco",
                executable="sim_rgb_camera",
                name="rebotarm_mujoco_sim_rgb_camera",
                output="screen",
                parameters=[
                    {
                        "arm_namespace": arm_namespace,
                        "model_path": model_path,
                        "joint_map_file": joint_map_file,
                        "joint_state_topic": joint_state_topic,
                        "object_states_topic": object_states_topic,
                        "target_pose_topic": target_pose_topic,
                        "target_visible_timeout": target_visible_timeout,
                        "render_target_marker": render_target_marker,
                        "camera_name": camera_name,
                        "image_topic": image_topic,
                        "camera_info_topic": camera_info_topic,
                        "frame_id": frame_id,
                        "width": width,
                        "height": height,
                        "publish_hz": publish_hz,
                        "virtual_grasp_enabled": virtual_grasp_enabled,
                    }
                ],
            ),
        ]
    )
