"""Multi-drone sensing + per-drone OctoMap (Phase 3 Step 3-6).

No explorer, scheduler, or global map merge.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _f(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _cave_arg_names():
    return [
        'cave_mode', 'seed', 'base_radius', 'n_segments', 'density', 'noise_scale',
        'length', 'branch_length', 'branch', 'branch_angle', 'chamber_at', 'chamber_scale',
        'tree.approach_length', 'tree.loop_yaw', 'tree.loop_direct_length', 'tree.loop_bulge',
        'tree.exit1_length', 'tree.right_yaw', 'tree.right_corridor_length',
        'tree.exit_yaw_spread', 'tree.exit_arm_length', 'tree.vertical_step',
        'tree.asymmetry', 'tree.chamber_on_approach', 'tree.chamber_at', 'tree.chamber_scale',
    ]


def _default_start_xy(index: int):
    # Lateral stagger at cave entrance; yaw remains +X.
    offsets = {
        0: (0.0, 0.0),
        1: (0.0, 1.0),
        2: (0.0, -1.0),
    }
    if index in offsets:
        return offsets[index]
    # Beyond 3: continue alternating Y with small X nudge to avoid exact overlap.
    side = 1.0 if (index % 2) else -1.0
    return (0.2 * (index // 2), side * (1.0 + 0.5 * (index // 2)))


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
    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    if num_drones < 1:
        raise RuntimeError('num_drones must be >= 1')

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

    actions = [
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(cave_publisher_launch),
                    condition=IfCondition(LaunchConfiguration('show_cave_truth')),
                    launch_arguments={
                        key: LaunchConfiguration(key) for key in _cave_arg_names()
                    }.items(),
                ),
            ],
        ),
    ]

    motion_mode = LaunchConfiguration('motion.mode').perform(context)
    start_z = LaunchConfiguration('line.start_z').perform(context)
    line_length = float(LaunchConfiguration('line.length').perform(context))
    altitude_min_clearance = _vertical_clearance(context)
    shared_sensing = {
        'map_frame': 'map',
        'publish_map_to_odom': 'false',
        'enable_scan_accumulator': LaunchConfiguration(
            'enable_scan_accumulator'
        ).perform(context),
        'max_cloud_map_points': LaunchConfiguration(
            'max_cloud_map_points'
        ).perform(context),
        'motion.mode': motion_mode,
        'motion.linear_speed': LaunchConfiguration('motion.linear_speed').perform(context),
        'motion.yaw_rate': LaunchConfiguration('motion.yaw_rate').perform(context),
        # line 飞完后仍持续扫，便于看 OctoMap 随位移增长；goal 悬停同样持续扫。
        'stop_scan_when_trajectory_done': 'false',
        'max_range': LaunchConfiguration('max_range').perform(context),
        'ring_pitch_rad': LaunchConfiguration('ring_pitch_rad').perform(context),
        'scan_rate': LaunchConfiguration('scan_rate').perform(context),
        'num_beams': LaunchConfiguration('num_beams').perform(context),
        'line.start_z': start_z,
        'line.duration_seconds': LaunchConfiguration('line.duration_seconds').perform(context),
        'altitude_adapt.min_clearance': str(altitude_min_clearance),
    }
    for key in _cave_arg_names():
        shared_sensing[key] = LaunchConfiguration(key).perform(context)

    for i in range(num_drones):
        drone_ns = f'drone_{i}'
        start_x, start_y = _default_start_xy(i)
        # goal：原地悬停（end=start）。line：沿 +X 飞 line.length，便于目检地图增长。
        if motion_mode == 'line':
            end_x, end_y = start_x + line_length, start_y
        else:
            end_x, end_y = start_x, start_y
        odom_frame = f'{drone_ns}/odom'
        base_frame = f'{drone_ns}/base_link'
        lidar_frame = f'{drone_ns}/lidar_link'

        per_drone = {
            **shared_sensing,
            'drone_ns': drone_ns,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'lidar_frame': lidar_frame,
            'line.start_x': str(start_x),
            'line.start_y': str(start_y),
            'line.end_x': str(end_x),
            'line.end_y': str(end_y),
            'line.end_z': start_z,
        }

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensing_stack_launch),
                launch_arguments=per_drone.items(),
            )
        )
        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom',
                namespace=drone_ns,
                arguments=['0', '0', '0', '0', '0', '0', 'map', odom_frame],
            )
        )
        actions.append(
            Node(
                package='swarm_controller',
                executable='octomap_builder',
                name='octomap_builder',
                namespace=drone_ns,
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
            )
        )

    actions.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='swarm_sensing_rviz',
            output='screen',
            condition=IfCondition(LaunchConfiguration('show_rviz')),
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('swarm_controller'),
                'config',
                'swarm_sensing.rviz',
            ])],
            additional_env={'LD_PRELOAD': 'liboctomap.so'},
        )
    )
    return actions


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument('num_drones', default_value='3'),
        DeclareLaunchArgument('show_rviz', default_value='true'),
        DeclareLaunchArgument('show_cave_truth', default_value='false'),
        DeclareLaunchArgument('resolution', default_value='0.1'),
        DeclareLaunchArgument('max_range', default_value='30.0'),
        DeclareLaunchArgument('ring_pitch_rad', default_value='0.35'),
        DeclareLaunchArgument('scan_rate', default_value='10.0'),
        DeclareLaunchArgument('num_beams', default_value='360'),
        DeclareLaunchArgument(
            'motion.mode', default_value='goal',
            description='goal=悬停建图（默认）；line=沿 +X 短飞便于目检 OctoMap 增长',
        ),
        DeclareLaunchArgument('motion.linear_speed', default_value='0.4'),
        DeclareLaunchArgument('motion.yaw_rate', default_value='0.5'),
        DeclareLaunchArgument('line.start_z', default_value='1.5'),
        DeclareLaunchArgument('body.robot_half_height', default_value='0.15'),
        DeclareLaunchArgument('body.vertical_margin', default_value='0.20'),
        DeclareLaunchArgument('body.clearance_epsilon', default_value='0.01'),
        DeclareLaunchArgument('altitude_adapt.min_clearance', default_value='auto'),
        DeclareLaunchArgument(
            'line.length', default_value='8.0',
            description='motion.mode=line 时沿 +X 的飞行距离 (m)',
        ),
        DeclareLaunchArgument('line.duration_seconds', default_value='40.0'),
        DeclareLaunchArgument(
            'enable_scan_accumulator', default_value='false',
            description='Multi-drone default off to avoid cloud_map×N OOM',
        ),
        DeclareLaunchArgument('max_cloud_map_points', default_value='20000'),
        DeclareLaunchArgument('cave_mode', default_value='tree'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('base_radius', default_value='2.5'),
        DeclareLaunchArgument('n_segments', default_value='200'),
        DeclareLaunchArgument('density', default_value='400'),
        DeclareLaunchArgument('noise_scale', default_value='0.4'),
        DeclareLaunchArgument('length', default_value='40.0'),
        DeclareLaunchArgument('branch_length', default_value='20.0'),
        DeclareLaunchArgument('branch', default_value='true'),
        DeclareLaunchArgument('branch_angle', default_value='0.55'),
        DeclareLaunchArgument('chamber_at', default_value='0.5'),
        DeclareLaunchArgument('chamber_scale', default_value='3.0'),
        DeclareLaunchArgument('tree.approach_length', default_value='12.0'),
        DeclareLaunchArgument('tree.loop_yaw', default_value='0.50'),
        DeclareLaunchArgument('tree.loop_direct_length', default_value='16.0'),
        DeclareLaunchArgument('tree.loop_bulge', default_value='12.0'),
        DeclareLaunchArgument('tree.exit1_length', default_value='14.0'),
        DeclareLaunchArgument('tree.right_yaw', default_value='-0.12'),
        DeclareLaunchArgument('tree.right_corridor_length', default_value='10.0'),
        DeclareLaunchArgument('tree.exit_yaw_spread', default_value='0.35'),
        DeclareLaunchArgument('tree.exit_arm_length', default_value='14.0'),
        DeclareLaunchArgument('tree.vertical_step', default_value='-0.10'),
        DeclareLaunchArgument('tree.asymmetry', default_value='0.22'),
        DeclareLaunchArgument('tree.chamber_on_approach', default_value='false'),
        DeclareLaunchArgument('tree.chamber_at', default_value='0.55'),
        DeclareLaunchArgument('tree.chamber_scale', default_value='2.2'),
    ]
    return LaunchDescription([
        *declarations,
        OpaqueFunction(function=_launch_setup),
    ])
