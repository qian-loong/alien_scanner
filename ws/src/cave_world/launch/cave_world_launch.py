from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cave_publisher_launch = PathJoinSubstitution([
        FindPackageShare('cave_world'),
        'launch',
        'cave_publisher_launch.py',
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('cave_world'),
        'config',
        'cave_world.rviz',
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cave_publisher_launch),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
