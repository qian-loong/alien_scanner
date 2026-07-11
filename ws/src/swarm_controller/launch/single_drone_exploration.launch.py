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
    show_rviz = DeclareLaunchArgument('show_rviz', default_value='true')
    show_cave_truth = DeclareLaunchArgument('show_cave_truth', default_value='false')
    resolution = DeclareLaunchArgument('resolution', default_value='0.1')
    max_range = DeclareLaunchArgument('max_range', default_value='30.0')
    ring_pitch_rad = DeclareLaunchArgument('ring_pitch_rad', default_value='0.35')

    fake_lidar_launch = PathJoinSubstitution([
        FindPackageShare('drone_scanner'),
        'launch',
        'fake_lidar_launch.py',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('swarm_controller'),
        'config',
        'exploration.rviz',
    ])

    return LaunchDescription([
        show_rviz,
        show_cave_truth,
        resolution,
        max_range,
        ring_pitch_rad,
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(fake_lidar_launch),
                    launch_arguments={
                        'motion.mode': 'goal',
                        'motion.linear_speed': '0.4',
                        'motion.yaw_rate': '0.5',
                        'max_range': LaunchConfiguration('max_range'),
                        'ring_pitch_rad': LaunchConfiguration('ring_pitch_rad'),
                        'stop_scan_when_trajectory_done': 'false',
                        'show_cave': LaunchConfiguration('show_cave_truth'),
                        'show_rviz_sim': 'false',
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
                'map_frame': 'map',
                'input_topic': 'scan_returns',
                'output_topic': 'octomap',
                'resolution': _f('resolution'),
                'publish_rate': 2.0,
                'max_range': _f('max_range'),
            }],
        ),
        Node(
            package='swarm_controller',
            executable='single_drone_explorer',
            name='single_drone_explorer',
            namespace='drone_0',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'map_frame': 'map',
                'control_rate': 5.0,
                'frontier.planning_radius': 3.5,
                'frontier.max_goal_distance': 2.0,
                'entry.enforce_forward_half_space': True,
                'entry.backward_margin': 0.1,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='exploration_rviz',
            output='screen',
            condition=IfCondition(LaunchConfiguration('show_rviz')),
            arguments=['-d', rviz_config],
            additional_env={'LD_PRELOAD': 'liboctomap.so'},
        ),
    ])
