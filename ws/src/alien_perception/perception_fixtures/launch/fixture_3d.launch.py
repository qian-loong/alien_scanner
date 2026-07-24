from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [
            FindPackageShare("perception_input_node"),
            "config",
            "sensor_descriptors_3d.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="perception_fixtures",
                executable="perception_fixture_publisher",
                name="perception_fixture_publisher",
                output="screen",
                parameters=[
                    {
                        "mode": "3d",
                        "cloud_frame": "fixture_lidar_link",
                        "cloud_topic": "fixture/points",
                    }
                ],
            ),
            Node(
                package="perception_input_node",
                executable="perception_input_node",
                name="perception_input_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
