from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arm_namespace = LaunchConfiguration("arm_namespace")
    image_topic = LaunchConfiguration("image_topic")
    annotated_topic = LaunchConfiguration("annotated_topic")
    detections_topic = LaunchConfiguration("detections_topic")
    target_pose_topic = LaunchConfiguration("target_pose_topic")
    poses_topic = LaunchConfiguration("poses_topic")
    object_states_topic = LaunchConfiguration("object_states_topic")
    object_association_max_distance_m = LaunchConfiguration("object_association_max_distance_m")
    target_color = LaunchConfiguration("target_color")
    process_hz = LaunchConfiguration("process_hz")
    roi_u_min = LaunchConfiguration("roi_u_min")
    roi_u_max = LaunchConfiguration("roi_u_max")
    roi_v_min = LaunchConfiguration("roi_v_min")
    roi_v_max = LaunchConfiguration("roi_v_max")
    min_bbox_width_px = LaunchConfiguration("min_bbox_width_px")
    min_bbox_height_px = LaunchConfiguration("min_bbox_height_px")
    max_bbox_width_px = LaunchConfiguration("max_bbox_width_px")
    max_bbox_height_px = LaunchConfiguration("max_bbox_height_px")
    object_max_size_m = LaunchConfiguration("object_max_size_m")
    table_edge_margin_m = LaunchConfiguration("table_edge_margin_m")
    table_x_min = LaunchConfiguration("table_x_min")
    table_x_max = LaunchConfiguration("table_x_max")
    table_y_min = LaunchConfiguration("table_y_min")
    table_y_max = LaunchConfiguration("table_y_max")

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("image_topic", default_value="/rebotarm/mujoco/overhead_rgb/image_raw"),
            DeclareLaunchArgument(
                "annotated_topic",
                default_value="/rebotarm/vision/color_blocks/annotated",
            ),
            DeclareLaunchArgument(
                "detections_topic",
                default_value="/rebotarm/vision/color_blocks/detections",
            ),
            DeclareLaunchArgument(
                "target_pose_topic",
                default_value="/rebotarm/vision/color_blocks/target_pose",
            ),
            DeclareLaunchArgument("poses_topic", default_value="/rebotarm/vision/color_blocks/poses"),
            DeclareLaunchArgument(
                "object_states_topic",
                default_value="/rebotarm/mujoco/object_states",
            ),
            DeclareLaunchArgument("object_association_max_distance_m", default_value="0.055"),
            DeclareLaunchArgument("target_color", default_value="red"),
            DeclareLaunchArgument("process_hz", default_value="8.0"),
            DeclareLaunchArgument("roi_u_min", default_value="0.24"),
            DeclareLaunchArgument("roi_u_max", default_value="0.82"),
            DeclareLaunchArgument("roi_v_min", default_value="0.29"),
            DeclareLaunchArgument("roi_v_max", default_value="0.86"),
            DeclareLaunchArgument("min_bbox_width_px", default_value="4"),
            DeclareLaunchArgument("min_bbox_height_px", default_value="4"),
            DeclareLaunchArgument("max_bbox_width_px", default_value="52"),
            DeclareLaunchArgument("max_bbox_height_px", default_value="34"),
            DeclareLaunchArgument("object_max_size_m", default_value="0.13"),
            DeclareLaunchArgument("table_edge_margin_m", default_value="0.025"),
            DeclareLaunchArgument("table_x_min", default_value="0.24"),
            DeclareLaunchArgument("table_x_max", default_value="0.72"),
            DeclareLaunchArgument("table_y_min", default_value="-0.26"),
            DeclareLaunchArgument("table_y_max", default_value="0.26"),
            Node(
                package="rebotarm_mujoco",
                executable="sim_color_detector",
                name="rebotarm_mujoco_sim_color_detector",
                output="screen",
                parameters=[
                    {
                        "arm_namespace": arm_namespace,
                        "image_topic": image_topic,
                        "annotated_topic": annotated_topic,
                        "detections_topic": detections_topic,
                        "target_pose_topic": target_pose_topic,
                        "poses_topic": poses_topic,
                        "object_states_topic": object_states_topic,
                        "object_association_max_distance_m": object_association_max_distance_m,
                        "target_color": target_color,
                        "process_hz": process_hz,
                        "roi_u_min": roi_u_min,
                        "roi_u_max": roi_u_max,
                        "roi_v_min": roi_v_min,
                        "roi_v_max": roi_v_max,
                        "min_bbox_width_px": min_bbox_width_px,
                        "min_bbox_height_px": min_bbox_height_px,
                        "max_bbox_width_px": max_bbox_width_px,
                        "max_bbox_height_px": max_bbox_height_px,
                        "object_max_size_m": object_max_size_m,
                        "table_edge_margin_m": table_edge_margin_m,
                        "table_x_min": table_x_min,
                        "table_x_max": table_x_max,
                        "table_y_min": table_y_min,
                        "table_y_max": table_y_max,
                    }
                ],
            ),
        ]
    )
