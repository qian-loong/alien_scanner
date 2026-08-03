from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def static_transform(name, child_frame, x, y, z, pitch="0"):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        output="screen",
        arguments=[
            "--x",
            x,
            "--y",
            y,
            "--z",
            z,
            "--roll",
            "0",
            "--pitch",
            pitch,
            "--yaw",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            child_frame,
        ],
    )


def generate_launch_description():
    tilted_scan_pitch = LaunchConfiguration("tilted_scan_pitch_rad")
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("perception_fixtures"),
            "config",
            "perception_fixtures.rviz",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("show_rviz", default_value="true"),
            DeclareLaunchArgument(
                "tilted_scan_pitch_rad",
                default_value="-0.5235987756",
                description=(
                    "Pitch of the tilted 2D lidar relative to base_link; "
                    "the negative default raises sensor +X toward base_link +Z"
                ),
            ),
            Node(
                package="perception_fixtures",
                executable="perception_fixture_publisher",
                name="fixture_scan_flat_publisher",
                output="screen",
                parameters=[
                    {
                        "mode": "2d",
                        "scan_frame": "fixture_scan_flat_link",
                        "scan_topic_front": "/fixture/scan/flat",
                    }
                ],
            ),
            Node(
                package="perception_fixtures",
                executable="perception_fixture_publisher",
                name="fixture_scan_tilted_publisher",
                output="screen",
                parameters=[
                    {
                        "mode": "2d",
                        "scan_frame": "fixture_scan_tilted_link",
                        "scan_topic_front": "/fixture/scan/tilted",
                    }
                ],
            ),
            Node(
                package="perception_fixtures",
                executable="perception_fixture_publisher",
                name="fixture_lidar_3d_publisher",
                output="screen",
                parameters=[
                    {
                        "mode": "3d",
                        "cloud_frame": "fixture_lidar_link",
                        "cloud_topic": "/fixture/points",
                    }
                ],
            ),
            static_transform(
                "base_to_fixture_scan_flat",
                "fixture_scan_flat_link",
                "0",
                "0.35",
                "0.15",
            ),
            static_transform(
                "base_to_fixture_scan_tilted",
                "fixture_scan_tilted_link",
                "0",
                "-0.35",
                "0.15",
                tilted_scan_pitch,
            ),
            static_transform(
                "base_to_fixture_lidar",
                "fixture_lidar_link",
                "0",
                "0",
                "0.25",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="perception_fixtures_rviz",
                output="screen",
                condition=IfCondition(LaunchConfiguration("show_rviz")),
                arguments=["-d", rviz_config],
            ),
        ]
    )
