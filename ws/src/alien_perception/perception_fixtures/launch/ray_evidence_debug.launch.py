import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


DEBUG_SCAN = {
    "beam_count": 360,
    "angle_min_rad": -math.pi,
    "angle_increment_rad": 2.0 * math.pi / 360.0,
    "range_min_m": 0.1,
    "range_max_m": 10.0,
}
DEBUG_SCAN["angle_max_rad"] = (
    DEBUG_SCAN["angle_min_rad"]
    + (DEBUG_SCAN["beam_count"] - 1) * DEBUG_SCAN["angle_increment_rad"]
)


def debug_scan_transform():
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_debug_scan",
        output="screen",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            str(math.pi / 2.0),
            "--yaw",
            "0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "debug_scan_link",
        ],
    )


def generate_launch_description():
    observation_topic = "/perception/debug/observations"
    marker_topic = "/perception/debug/ray_evidence"
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("perception_fixtures"),
            "config",
            "ray_evidence_debug.rviz",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("show_rviz", default_value="true"),
            DeclareLaunchArgument("beam_stride", default_value="2"),
            Node(
                package="perception_fixtures",
                executable="perception_fixture_publisher",
                name="ray_evidence_debug_fixture",
                output="screen",
                parameters=[
                    {
                        "mode": "2d",
                        "scan_topic_front": "/fixture/debug/scan",
                        "scan_frame": "debug_scan_link",
                        "scan_point_count": DEBUG_SCAN["beam_count"],
                        "scan_angle_min_rad": DEBUG_SCAN["angle_min_rad"],
                        "scan_angle_max_rad": DEBUG_SCAN["angle_max_rad"],
                        "scan_range_min_m": DEBUG_SCAN["range_min_m"],
                        "scan_range_max_m": DEBUG_SCAN["range_max_m"],
                        "inject_debug_returns": True,
                    }
                ],
            ),
            Node(
                package="perception_input_node",
                executable="perception_input_node",
                name="ray_evidence_debug_input",
                output="screen",
                parameters=[
                    {
                        "sensor_ids": ["debug_scan"],
                        "requires_pose": False,
                        "minimum_lidar_type": "2d",
                        "minimum_lidar_count": 1,
                        "minimum_lidar_ray_evidence": "full_ray",
                        "degraded_lidar_type": "2d",
                        "degraded_lidar_count": 1,
                        "degraded_lidar_ray_evidence": "full_ray",
                        "observation_topic": observation_topic,
                        "sensor.debug_scan.type": "2d",
                        "sensor.debug_scan.topic": "/fixture/debug/scan",
                        "sensor.debug_scan.frame_id": "debug_scan_link",
                        "sensor.debug_scan.fov_horizontal_min_rad": DEBUG_SCAN["angle_min_rad"],
                        "sensor.debug_scan.fov_horizontal_max_rad": DEBUG_SCAN["angle_max_rad"],
                        "sensor.debug_scan.angular_resolution_rad": DEBUG_SCAN["angle_increment_rad"],
                        "sensor.debug_scan.range_min_m": DEBUG_SCAN["range_min_m"],
                        "sensor.debug_scan.range_max_m": DEBUG_SCAN["range_max_m"],
                        "sensor.debug_scan.ray_evidence": "full_ray",
                    }
                ],
            ),
            Node(
                package="perception_fixtures",
                executable="ray_evidence_debug_node",
                name="ray_evidence_debug_node",
                output="screen",
                parameters=[
                    {
                        "observation_topic": observation_topic,
                        "marker_topic": marker_topic,
                        "beam_stride": LaunchConfiguration("beam_stride"),
                        "marker_lifetime_s": 0.5,
                        "point_size_m": 0.12,
                        "line_width_m": 0.012,
                        "invalid_indicator_length_m": 2.5,
                    }
                ],
            ),
            debug_scan_transform(),
            Node(
                package="rviz2",
                executable="rviz2",
                name="ray_evidence_debug_rviz",
                output="screen",
                condition=IfCondition(LaunchConfiguration("show_rviz")),
                arguments=["-d", rviz_config],
            ),
        ]
    )
