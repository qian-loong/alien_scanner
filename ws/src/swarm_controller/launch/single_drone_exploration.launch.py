from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _vertical_clearance(context):
    resolution = float(LaunchConfiguration('resolution').perform(context))
    half_height = float(
        LaunchConfiguration('body.robot_half_height').perform(context))
    vertical_margin = float(
        LaunchConfiguration('body.vertical_margin').perform(context))
    epsilon = float(
        LaunchConfiguration('body.clearance_epsilon').perform(context))
    required = half_height + vertical_margin + 0.5 * resolution + epsilon
    requested = LaunchConfiguration(
        'altitude_adapt.min_clearance').perform(context).strip().lower()
    if requested == 'auto':
        return required
    configured = float(requested)
    if configured + 1.0e-9 < required:
        raise RuntimeError(
            'altitude_adapt.min_clearance must be >= '
            f'{required:.3f} m for the configured swept-body envelope')
    return configured


def _launch_setup(context, *args, **kwargs):
    altitude_min_clearance = _vertical_clearance(context)
    resolution = float(LaunchConfiguration('resolution').perform(context))
    max_range = float(LaunchConfiguration('max_range').perform(context))
    half_height = float(
        LaunchConfiguration('body.robot_half_height').perform(context))
    vertical_margin = float(
        LaunchConfiguration('body.vertical_margin').perform(context))

    sensing_stack_launch = PathJoinSubstitution([
        FindPackageShare('drone_scanner'),
        'launch',
        'drone_sensing_stack.launch.py',
    ])
    cave_publisher_launch = PathJoinSubstitution([
        FindPackageShare('cave_world'),
        'launch',
        'cave_publisher_launch.py',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('swarm_controller'),
        'config',
        'exploration.rviz',
    ])

    sensing_args = {
        'drone_ns': 'drone_0',
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_frame': 'base_link',
        'lidar_frame': 'lidar_link',
        'publish_map_to_odom': 'true',
        'enable_scan_accumulator': 'true',
        'motion.mode': 'goal',
        'motion.linear_speed': '0.4',
        'motion.yaw_rate': '0.5',
        'max_range': str(max_range),
        'ring_pitch_rad': LaunchConfiguration(
            'ring_pitch_rad').perform(context),
        'altitude_adapt.min_clearance': str(altitude_min_clearance),
        'stop_scan_when_trajectory_done': 'false',
    }

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cave_publisher_launch),
            condition=IfCondition(LaunchConfiguration('show_cave_truth')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensing_stack_launch),
            launch_arguments=sensing_args.items(),
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
                'resolution': resolution,
                'publish_rate': 2.0,
                'max_range': max_range,
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
                'frontier.forward_lookahead_min': 0.8,
                'frontier.forward_lookahead_max': 2.0,
                'frontier.forward_lateral_limit': 0.5,
                'entry.enforce_forward_half_space': True,
                'entry.backward_margin': 0.1,
                'body.robot_half_height': half_height,
                'body.vertical_margin': vertical_margin,
                'safety.altitude_min_clearance': altitude_min_clearance,
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
    ]


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument('show_rviz', default_value='true'),
        DeclareLaunchArgument('show_cave_truth', default_value='false'),
        DeclareLaunchArgument('resolution', default_value='0.1'),
        DeclareLaunchArgument('max_range', default_value='30.0'),
        DeclareLaunchArgument('ring_pitch_rad', default_value='0.35'),
        DeclareLaunchArgument('body.robot_half_height', default_value='0.15'),
        DeclareLaunchArgument('body.vertical_margin', default_value='0.20'),
        DeclareLaunchArgument('body.clearance_epsilon', default_value='0.01'),
        DeclareLaunchArgument('altitude_adapt.min_clearance', default_value='auto'),
    ]
    return LaunchDescription([
        *declarations,
        OpaqueFunction(function=_launch_setup),
    ])
