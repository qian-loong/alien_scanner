"""Multi-drone exploration with local safety and global frontier task ownership.

Reuses 3-6 sensing + per-drone OctoMap, then mounts N SingleDroneExplorer nodes
with peer_namespaces for soft/hard dispersion. The global map remains an output;
each explorer continues planning against its own OctoMap.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
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
    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    if num_drones < 1:
        raise RuntimeError('num_drones must be >= 1')

    # Evaluate before IncludeLaunchDescription: child show_rviz:=false must not
    # shadow this launch's RViz condition (OpaqueFunction + LaunchConfiguration gotcha).
    show_rviz = LaunchConfiguration('show_rviz').perform(context).lower() in (
        'true', '1', 'yes',
    )
    enable_global_map = LaunchConfiguration('enable_global_map').perform(
        context).lower() in ('true', '1', 'yes')
    enable_task_allocation = LaunchConfiguration('task_allocation.enabled').perform(
        context).lower() in ('true', '1', 'yes')
    if enable_task_allocation and not enable_global_map:
        raise RuntimeError('task allocation requires enable_global_map:=true')
    altitude_min_clearance = _vertical_clearance(context)

    sensing_launch = PathJoinSubstitution([
        FindPackageShare('swarm_controller'),
        'launch',
        'multi_drone_sensing.launch.py',
    ])

    # Forward sensing args; force show_rviz false on the include — this launch owns RViz.
    sensing_args = {
        'num_drones': str(num_drones),
        'show_rviz': 'false',
        'show_cave_truth': LaunchConfiguration('show_cave_truth').perform(context),
        'resolution': LaunchConfiguration('resolution').perform(context),
        'max_range': LaunchConfiguration('max_range').perform(context),
        'ring_pitch_rad': LaunchConfiguration('ring_pitch_rad').perform(context),
        'scan_rate': LaunchConfiguration('scan_rate').perform(context),
        'num_beams': LaunchConfiguration('num_beams').perform(context),
        'motion.mode': 'goal',
        'motion.linear_speed': LaunchConfiguration('motion.linear_speed').perform(context),
        'motion.yaw_rate': LaunchConfiguration('motion.yaw_rate').perform(context),
        'line.start_z': LaunchConfiguration('line.start_z').perform(context),
        'body.robot_half_height': LaunchConfiguration(
            'body.robot_half_height').perform(context),
        'body.vertical_margin': LaunchConfiguration(
            'body.vertical_margin').perform(context),
        'body.clearance_epsilon': LaunchConfiguration(
            'body.clearance_epsilon').perform(context),
        'altitude_adapt.min_clearance': str(altitude_min_clearance),
        'enable_scan_accumulator': 'false',
        'cave_mode': LaunchConfiguration('cave_mode').perform(context),
        'seed': LaunchConfiguration('seed').perform(context),
        'base_radius': LaunchConfiguration('base_radius').perform(context),
        'n_segments': LaunchConfiguration('n_segments').perform(context),
        'density': LaunchConfiguration('density').perform(context),
        'noise_scale': LaunchConfiguration('noise_scale').perform(context),
        'tree.approach_length': LaunchConfiguration(
            'tree.approach_length').perform(context),
        'tree.loop_yaw': LaunchConfiguration('tree.loop_yaw').perform(context),
        'tree.loop_direct_length': LaunchConfiguration(
            'tree.loop_direct_length').perform(context),
        'tree.loop_bulge': LaunchConfiguration('tree.loop_bulge').perform(context),
        'tree.exit1_length': LaunchConfiguration('tree.exit1_length').perform(context),
        'tree.right_yaw': LaunchConfiguration('tree.right_yaw').perform(context),
        'tree.right_corridor_length': LaunchConfiguration(
            'tree.right_corridor_length').perform(context),
        'tree.exit_yaw_spread': LaunchConfiguration(
            'tree.exit_yaw_spread').perform(context),
        'tree.exit_arm_length': LaunchConfiguration(
            'tree.exit_arm_length').perform(context),
        'tree.vertical_step': LaunchConfiguration('tree.vertical_step').perform(context),
        'tree.asymmetry': LaunchConfiguration('tree.asymmetry').perform(context),
        'tree.chamber_on_approach': LaunchConfiguration(
            'tree.chamber_on_approach').perform(context),
        'tree.chamber_at': LaunchConfiguration('tree.chamber_at').perform(context),
        'tree.chamber_scale': LaunchConfiguration(
            'tree.chamber_scale').perform(context),
    }

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensing_launch),
            launch_arguments=sensing_args.items(),
        ),
    ]

    control_rate = LaunchConfiguration('control_rate').perform(context)
    forward_lookahead_min = LaunchConfiguration(
        'frontier.forward_lookahead_min').perform(context)
    forward_lookahead_max = LaunchConfiguration(
        'frontier.forward_lookahead_max').perform(context)
    forward_lateral_limit = LaunchConfiguration(
        'frontier.forward_lateral_limit').perform(context)
    forward_distance_samples = LaunchConfiguration(
        'frontier.forward_distance_samples').perform(context)
    forward_lateral_samples = LaunchConfiguration(
        'frontier.forward_lateral_samples').perform(context)
    dispersion_weight = LaunchConfiguration('frontier.dispersion_weight').perform(context)
    min_peer_sep = LaunchConfiguration('frontier.min_peer_goal_separation').perform(context)
    peer_position_timeout = LaunchConfiguration('peer.position_timeout').perform(context)
    peer_goal_timeout = LaunchConfiguration('peer.goal_timeout').perform(context)
    peer_retry_interval = LaunchConfiguration('peer.retry_interval').perform(context)
    travel_heading_update_distance = LaunchConfiguration(
        'motion.travel_heading_update_distance').perform(context)

    for i in range(num_drones):
        drone_ns = f'drone_{i}'
        peers = [f'drone_{j}' for j in range(num_drones) if j != i]
        actions.append(
            Node(
                package='swarm_controller',
                executable='single_drone_explorer',
                name='single_drone_explorer',
                namespace=drone_ns,
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'map_frame': 'map',
                    'control_rate': float(control_rate),
                    'peer_namespaces': peers,
                    'frontier.forward_lookahead_min': float(forward_lookahead_min),
                    'frontier.forward_lookahead_max': float(forward_lookahead_max),
                    'frontier.forward_lateral_limit': float(forward_lateral_limit),
                    'frontier.forward_distance_samples': int(forward_distance_samples),
                    'frontier.forward_lateral_samples': int(forward_lateral_samples),
                    'frontier.dispersion_weight': float(dispersion_weight),
                    'frontier.min_peer_goal_separation': float(min_peer_sep),
                    'peer.position_timeout': float(peer_position_timeout),
                    'peer.goal_timeout': float(peer_goal_timeout),
                    'peer.retry_interval': float(peer_retry_interval),
                    'task.receive_watchdog': float(LaunchConfiguration(
                        'task.receive_watchdog').perform(context)),
                    'task.retry_interval': float(LaunchConfiguration(
                        'task.retry_interval').perform(context)),
                    'task.rescan_max_steps': int(LaunchConfiguration(
                        'task.rescan_max_steps').perform(context)),
                    'task.rescan_step_timeout': float(LaunchConfiguration(
                        'task.rescan_step_timeout').perform(context)),
                    'task.min_progress': float(LaunchConfiguration(
                        'task.min_progress').perform(context)),
                    'task.max_heading_error': float(LaunchConfiguration(
                        'task.max_heading_error').perform(context)),
                    'frontier.task_progress_weight': float(LaunchConfiguration(
                        'frontier.task_progress_weight').perform(context)),
                    'motion.travel_heading_update_distance': float(
                        travel_heading_update_distance),
                    'body.robot_half_height': float(
                        LaunchConfiguration(
                            'body.robot_half_height').perform(context)),
                    'body.robot_radius': float(
                        LaunchConfiguration('body.robot_radius').perform(context)),
                    'body.vertical_margin': float(
                        LaunchConfiguration(
                            'body.vertical_margin').perform(context)),
                    'safety.altitude_min_clearance': altitude_min_clearance,
                    'recovery.timeout': float(
                        LaunchConfiguration('recovery.timeout').perform(context)),
                    'planning.max_snapshot_age': float(
                        LaunchConfiguration(
                            'planning.max_snapshot_age').perform(context)),
                    'entry.enforce_forward_half_space': True,
                    'entry.backward_margin': 0.1,
                }],
            )
        )

    if enable_global_map:
        actions.append(
            Node(
                package='swarm_controller',
                executable='global_map_merger',
                name='global_map_merger',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'map_frame': 'map',
                    'source_topics': [
                        f'/drone_{i}/octomap' for i in range(num_drones)
                    ],
                    'output_topic': 'global_map',
                    'diagnostics_topic': 'global_map_diagnostics',
                    'resolution': float(
                        LaunchConfiguration('resolution').perform(context)),
                    'merge_rate': float(LaunchConfiguration(
                        'global_map.merge_rate').perform(context)),
                    'source_stale_timeout': float(LaunchConfiguration(
                        'global_map.source_stale_timeout').perform(context)),
                    'max_serialized_bytes_per_source': int(LaunchConfiguration(
                        'global_map.max_serialized_bytes_per_source').perform(context)),
                    'max_voxels_per_source': int(LaunchConfiguration(
                        'global_map.max_voxels_per_source').perform(context)),
                    'max_global_voxels': int(LaunchConfiguration(
                        'global_map.max_global_voxels').perform(context)),
                }],
            )
        )

    if enable_task_allocation:
        actions.append(
            Node(
                package='swarm_controller',
                executable='global_task_allocator',
                name='global_task_allocator',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'map_frame': 'map',
                    'drone_namespaces': [
                        f'drone_{i}' for i in range(num_drones)
                    ],
                    'task_allocation.rate': float(LaunchConfiguration(
                        'task_allocation.rate').perform(context)),
                    'task.lease': float(LaunchConfiguration(
                        'task.lease').perform(context)),
                    'global_map.max_serialized_bytes_per_source': int(
                        LaunchConfiguration(
                            'global_map.max_serialized_bytes_per_source').perform(
                                context)),
                    'global_map.max_voxels_per_source': int(
                        LaunchConfiguration(
                            'global_map.max_voxels_per_source').perform(context)),
                    'global_map.max_global_voxels': int(LaunchConfiguration(
                        'global_map.max_global_voxels').perform(context)),
                    'resolution': float(LaunchConfiguration(
                        'resolution').perform(context)),
                    'frontier.column_stride_voxels': int(LaunchConfiguration(
                        'frontier.column_stride_voxels').perform(context)),
                    'frontier.max_scanned_free_voxels': int(LaunchConfiguration(
                        'frontier.max_scanned_free_voxels').perform(context)),
                    'frontier.max_support_samples_per_column': int(LaunchConfiguration(
                        'frontier.max_support_samples_per_column').perform(context)),
                    'frontier.min_z_layers': int(LaunchConfiguration(
                        'frontier.min_z_layers').perform(context)),
                    'frontier.min_z_span': float(LaunchConfiguration(
                        'frontier.min_z_span').perform(context)),
                    'frontier.support_depth': float(LaunchConfiguration(
                        'frontier.support_depth').perform(context)),
                    'frontier.min_columns': int(LaunchConfiguration(
                        'frontier.min_columns').perform(context)),
                    'frontier.min_area': float(LaunchConfiguration(
                        'frontier.min_area').perform(context)),
                    'frontier.min_span': float(LaunchConfiguration(
                        'frontier.min_span').perform(context)),
                    'frontier.min_direction_consistency': float(
                        LaunchConfiguration(
                            'frontier.min_direction_consistency').perform(context)),
                    'frontier.collect_stage_timings': LaunchConfiguration(
                        'frontier.collect_stage_timings').perform(
                            context).lower() in ('true', '1', 'yes'),
                    'frontier.min_persistence_updates': int(
                        LaunchConfiguration(
                            'frontier.min_persistence_updates').perform(context)),
                    'frontier.min_persistence_time': float(LaunchConfiguration(
                        'frontier.min_persistence_time').perform(context)),
                    'frontier.missed_update_grace': int(LaunchConfiguration(
                        'frontier.missed_update_grace').perform(context)),
                    'allocation.max_assignment_distance': float(
                        LaunchConfiguration(
                            'allocation.max_assignment_distance').perform(context)),
                    'allocation.first_hop_distance': float(LaunchConfiguration(
                        'allocation.first_hop_distance').perform(context)),
                    'allocation.no_progress_timeout': float(LaunchConfiguration(
                        'allocation.no_progress_timeout').perform(context)),
                    'allocation.min_owner_progress': float(LaunchConfiguration(
                        'allocation.min_owner_progress').perform(context)),
                    'motion.timeout': 20.0,
                    'hold.timeout': 2.0,
                    'task.rescan_max_steps': int(LaunchConfiguration(
                        'task.rescan_max_steps').perform(context)),
                    'task.rescan_step_timeout': float(LaunchConfiguration(
                        'task.rescan_step_timeout').perform(context)),
                    'body.robot_half_height': float(LaunchConfiguration(
                        'body.robot_half_height').perform(context)),
                    'body.robot_radius': float(LaunchConfiguration(
                        'body.robot_radius').perform(context)),
                    'body.vertical_margin': float(LaunchConfiguration(
                        'body.vertical_margin').perform(context)),
                }],
            )
        )

    if LaunchConfiguration('enable_truth_audit').perform(context).lower() == 'true':
        audit_parameters = {
            'odom_topics': [f'/drone_{i}/odom' for i in range(num_drones)],
            'cave_mode': LaunchConfiguration('cave_mode').perform(context),
            'seed': int(LaunchConfiguration('seed').perform(context)),
            'base_radius': float(LaunchConfiguration('base_radius').perform(context)),
            'n_segments': int(LaunchConfiguration('n_segments').perform(context)),
            'density': int(LaunchConfiguration('density').perform(context)),
            'noise_scale': float(LaunchConfiguration('noise_scale').perform(context)),
            'tree.approach_length': float(
                LaunchConfiguration('tree.approach_length').perform(context)),
            'tree.loop_yaw': float(LaunchConfiguration('tree.loop_yaw').perform(context)),
            'tree.loop_direct_length': float(
                LaunchConfiguration('tree.loop_direct_length').perform(context)),
            'tree.loop_bulge': float(
                LaunchConfiguration('tree.loop_bulge').perform(context)),
            'tree.exit1_length': float(
                LaunchConfiguration('tree.exit1_length').perform(context)),
            'tree.right_yaw': float(LaunchConfiguration('tree.right_yaw').perform(context)),
            'tree.right_corridor_length': float(
                LaunchConfiguration('tree.right_corridor_length').perform(context)),
            'tree.exit_yaw_spread': float(
                LaunchConfiguration('tree.exit_yaw_spread').perform(context)),
            'tree.exit_arm_length': float(
                LaunchConfiguration('tree.exit_arm_length').perform(context)),
            'tree.vertical_step': float(
                LaunchConfiguration('tree.vertical_step').perform(context)),
            'tree.asymmetry': float(
                LaunchConfiguration('tree.asymmetry').perform(context)),
            'tree.chamber_on_approach': LaunchConfiguration(
                'tree.chamber_on_approach').perform(context).lower() == 'true',
            'tree.chamber_at': float(
                LaunchConfiguration('tree.chamber_at').perform(context)),
            'tree.chamber_scale': float(
                LaunchConfiguration('tree.chamber_scale').perform(context)),
            'body.robot_radius': float(
                LaunchConfiguration('body.robot_radius').perform(context)),
            'body.robot_half_height': float(
                LaunchConfiguration('body.robot_half_height').perform(context)),
        }
        actions.append(Node(
            package='drone_scanner',
            executable='truth_collision_audit',
            name='truth_collision_audit',
            output='screen',
            parameters=[audit_parameters],
        ))

    if show_rviz:
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='swarm_exploration_rviz',
                output='screen',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('swarm_controller'),
                    'config',
                    'swarm_exploration.rviz',
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
        DeclareLaunchArgument('enable_truth_audit', default_value='false'),
        DeclareLaunchArgument('enable_global_map', default_value='true'),
        DeclareLaunchArgument('task_allocation.enabled', default_value='true'),
        DeclareLaunchArgument('task_allocation.rate', default_value='1.0'),
        DeclareLaunchArgument('task.lease', default_value='3.0'),
        DeclareLaunchArgument('task.receive_watchdog', default_value='3.5'),
        DeclareLaunchArgument('task.retry_interval', default_value='1.0'),
        DeclareLaunchArgument('task.rescan_max_steps', default_value='4'),
        DeclareLaunchArgument('task.rescan_step_timeout', default_value='2.5'),
        DeclareLaunchArgument('task.min_progress', default_value='0.15'),
        DeclareLaunchArgument('task.max_heading_error', default_value='1.05'),
        DeclareLaunchArgument('global_map.merge_rate', default_value='1.0'),
        DeclareLaunchArgument(
            'global_map.source_stale_timeout', default_value='5.0'),
        DeclareLaunchArgument(
            'global_map.max_serialized_bytes_per_source',
            default_value='67108864'),
        DeclareLaunchArgument(
            'global_map.max_voxels_per_source', default_value='5000000'),
        DeclareLaunchArgument(
            'global_map.max_global_voxels', default_value='10000000'),
        DeclareLaunchArgument('resolution', default_value='0.1'),
        DeclareLaunchArgument('max_range', default_value='30.0'),
        DeclareLaunchArgument('ring_pitch_rad', default_value='0.35'),
        DeclareLaunchArgument('scan_rate', default_value='10.0'),
        DeclareLaunchArgument('num_beams', default_value='360'),
        DeclareLaunchArgument('motion.linear_speed', default_value='0.4'),
        DeclareLaunchArgument('motion.yaw_rate', default_value='0.5'),
        DeclareLaunchArgument('line.start_z', default_value='1.5'),
        DeclareLaunchArgument('body.robot_half_height', default_value='0.15'),
        DeclareLaunchArgument('body.robot_radius', default_value='0.25'),
        DeclareLaunchArgument('body.vertical_margin', default_value='0.20'),
        DeclareLaunchArgument('body.clearance_epsilon', default_value='0.01'),
        DeclareLaunchArgument('altitude_adapt.min_clearance', default_value='auto'),
        DeclareLaunchArgument('control_rate', default_value='3.0'),
        DeclareLaunchArgument('frontier.forward_lookahead_min', default_value='0.8'),
        DeclareLaunchArgument('frontier.forward_lookahead_max', default_value='2.0'),
        DeclareLaunchArgument('frontier.forward_lateral_limit', default_value='0.5'),
        DeclareLaunchArgument('frontier.forward_distance_samples', default_value='4'),
        DeclareLaunchArgument('frontier.forward_lateral_samples', default_value='5'),
        DeclareLaunchArgument('frontier.dispersion_weight', default_value='0.35'),
        DeclareLaunchArgument('frontier.task_progress_weight', default_value='1.0'),
        DeclareLaunchArgument('frontier.min_peer_goal_separation', default_value='0.8'),
        DeclareLaunchArgument('frontier.column_stride_voxels', default_value='2'),
        DeclareLaunchArgument(
            'frontier.max_scanned_free_voxels', default_value='2000000'),
        DeclareLaunchArgument(
            'frontier.max_support_samples_per_column', default_value='10000'),
        DeclareLaunchArgument('frontier.min_z_layers', default_value='5'),
        DeclareLaunchArgument('frontier.min_z_span', default_value='0.4'),
        DeclareLaunchArgument('frontier.support_depth', default_value='0.8'),
        DeclareLaunchArgument('frontier.min_columns', default_value='12'),
        DeclareLaunchArgument('frontier.min_area', default_value='0.48'),
        DeclareLaunchArgument('frontier.min_span', default_value='0.6'),
        DeclareLaunchArgument(
            'frontier.min_direction_consistency', default_value='0.65'),
        DeclareLaunchArgument(
            'frontier.collect_stage_timings', default_value='false'),
        DeclareLaunchArgument(
            'frontier.min_persistence_updates', default_value='3'),
        DeclareLaunchArgument(
            'frontier.min_persistence_time', default_value='2.0'),
        DeclareLaunchArgument('frontier.missed_update_grace', default_value='2'),
        DeclareLaunchArgument(
            'allocation.max_assignment_distance', default_value='8.0'),
        DeclareLaunchArgument(
            'allocation.first_hop_distance', default_value='1.0'),
        DeclareLaunchArgument(
            'allocation.no_progress_timeout', default_value='35.0'),
        DeclareLaunchArgument(
            'allocation.min_owner_progress', default_value='0.30'),
        DeclareLaunchArgument('peer.position_timeout', default_value='2.0'),
        DeclareLaunchArgument('peer.goal_timeout', default_value='25.0'),
        DeclareLaunchArgument('peer.retry_interval', default_value='1.0'),
        DeclareLaunchArgument(
            'motion.travel_heading_update_distance', default_value='0.35'),
        DeclareLaunchArgument('recovery.timeout', default_value='8.0'),
        DeclareLaunchArgument('planning.max_snapshot_age', default_value='1.0'),
        DeclareLaunchArgument('cave_mode', default_value='tree'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('base_radius', default_value='2.5'),
        DeclareLaunchArgument('n_segments', default_value='200'),
        DeclareLaunchArgument('density', default_value='400'),
        DeclareLaunchArgument('noise_scale', default_value='0.4'),
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
