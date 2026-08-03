from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("perception_input_node"),
            "config",
            "sensor_descriptors_2d.yaml",
        ]
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Perception sensor descriptor parameter file",
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
