from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _f(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    map_frame = DeclareLaunchArgument('map_frame', default_value='map')
    resolution = DeclareLaunchArgument('resolution', default_value='0.1')
    publish_rate = DeclareLaunchArgument('publish_rate', default_value='2.0')
    max_range = DeclareLaunchArgument('max_range', default_value='30.0')
    include_sim = DeclareLaunchArgument(
        'include_sim', default_value='true',
        description='启动 drone_scanner 单机仿真以产生 /drone_0/scan_returns')
    show_rviz_sim = DeclareLaunchArgument('show_rviz_sim', default_value='true')
    show_rviz_map = DeclareLaunchArgument('show_rviz_map', default_value='true')

    fake_lidar_launch = PathJoinSubstitution([
        FindPackageShare('drone_scanner'),
        'launch',
        'fake_lidar_launch.py',
    ])
    octomap_rviz_config = PathJoinSubstitution([
        FindPackageShare('swarm_controller'),
        'config',
        'octomap_map.rviz',
    ])

    return LaunchDescription([
        map_frame,
        resolution,
        publish_rate,
        max_range,
        include_sim,
        show_rviz_sim,
        show_rviz_map,
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(fake_lidar_launch),
                    condition=IfCondition(LaunchConfiguration('include_sim')),
                    launch_arguments={
                        'max_range': LaunchConfiguration('max_range'),
                        'show_rviz_sim': LaunchConfiguration('show_rviz_sim'),
                        'show_rviz_map': 'false',
                    }.items(),
                ),
            ],
        ),
        Node(
            package='swarm_controller',
            executable='octomap_builder',
            name='octomap_builder',
            namespace='drone_0',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'map_frame': LaunchConfiguration('map_frame'),
                'input_topic': 'scan_returns',
                'output_topic': 'octomap',
                'resolution': _f('resolution'),
                'publish_rate': _f('publish_rate'),
                'max_range': _f('max_range'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='octomap_rviz',
            output='screen',
            condition=IfCondition(LaunchConfiguration('show_rviz_map')),
            arguments=['-d', octomap_rviz_config],
            additional_env={'LD_PRELOAD': 'liboctomap.so'},
        ),
    ])
