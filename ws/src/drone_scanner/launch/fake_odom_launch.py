from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _f(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    line_start_x = DeclareLaunchArgument('line.start_x', default_value='0.0')
    line_start_y = DeclareLaunchArgument('line.start_y', default_value='0.0')
    line_start_z = DeclareLaunchArgument('line.start_z', default_value='1.5')
    line_end_x = DeclareLaunchArgument('line.end_x', default_value='11.0')
    line_end_y = DeclareLaunchArgument('line.end_y', default_value='0.0')
    line_end_z = DeclareLaunchArgument('line.end_z', default_value='1.5')
    line_duration = DeclareLaunchArgument('line.duration_seconds', default_value='60.0')
    publish_rate = DeclareLaunchArgument('publish_rate', default_value='20.0')
    show_cave = DeclareLaunchArgument(
        'show_cave', default_value='true', description='include cave_world cave_publisher_launch.py')

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
        line_start_x,
        line_start_y,
        line_start_z,
        line_end_x,
        line_end_y,
        line_end_z,
        line_duration,
        publish_rate,
        show_cave,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cave_publisher_launch),
            condition=IfCondition(LaunchConfiguration('show_cave')),
        ),
        Node(
            package='drone_scanner',
            executable='fake_odom',
            name='fake_odom',
            namespace='drone_0',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'line.start_x': _f('line.start_x'),
                'line.start_y': _f('line.start_y'),
                'line.start_z': _f('line.start_z'),
                'line.end_x': _f('line.end_x'),
                'line.end_y': _f('line.end_y'),
                'line.end_z': _f('line.end_z'),
                'line.duration_seconds': _f('line.duration_seconds'),
                'publish_rate': _f('publish_rate'),
                'odom_frame': 'odom',
                'base_frame': 'base_link',
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
