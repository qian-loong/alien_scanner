"""Per-drone sensing stack: fake_odom + fake_lidar + optional scan_accumulator + TF.

Does not start cave_publisher, RViz, explorer, or octomap_builder.
Namespace is set only via the drone_ns parameter (do not wrap with PushRosNamespace).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _f(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _i(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def _b(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _cave_params():
    return {
        'cave_mode': LaunchConfiguration('cave_mode'),
        'seed': _i('seed'),
        'base_radius': _f('base_radius'),
        'n_segments': _i('n_segments'),
        'density': _i('density'),
        'noise_scale': _f('noise_scale'),
        'length': _f('length'),
        'branch_length': _f('branch_length'),
        'branch': _b('branch'),
        'branch_angle': _f('branch_angle'),
        'chamber_at': _f('chamber_at'),
        'chamber_scale': _f('chamber_scale'),
        'tree.approach_length': _f('tree.approach_length'),
        'tree.loop_yaw': _f('tree.loop_yaw'),
        'tree.loop_direct_length': _f('tree.loop_direct_length'),
        'tree.loop_bulge': _f('tree.loop_bulge'),
        'tree.exit1_length': _f('tree.exit1_length'),
        'tree.right_yaw': _f('tree.right_yaw'),
        'tree.right_corridor_length': _f('tree.right_corridor_length'),
        'tree.exit_yaw_spread': _f('tree.exit_yaw_spread'),
        'tree.exit_arm_length': _f('tree.exit_arm_length'),
        'tree.vertical_step': _f('tree.vertical_step'),
        'tree.asymmetry': _f('tree.asymmetry'),
        'tree.chamber_on_approach': _b('tree.chamber_on_approach'),
        'tree.chamber_at': _f('tree.chamber_at'),
        'tree.chamber_scale': _f('tree.chamber_scale'),
    }


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument('drone_ns', default_value='drone_0'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('lidar_frame', default_value='lidar_link'),
        DeclareLaunchArgument(
            'publish_map_to_odom', default_value='true',
            description='Publish identity map→odom_frame (disable when parent owns it)',
        ),
        DeclareLaunchArgument(
            'enable_scan_accumulator', default_value='true',
            description='Publish accumulated hit-only cloud_map',
        ),
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
        DeclareLaunchArgument('line.start_x', default_value='0.0'),
        DeclareLaunchArgument('line.start_y', default_value='0.0'),
        DeclareLaunchArgument('line.start_z', default_value='1.5'),
        DeclareLaunchArgument('line.end_x', default_value='11.0'),
        DeclareLaunchArgument('line.end_y', default_value='0.0'),
        DeclareLaunchArgument('line.end_z', default_value='1.5'),
        DeclareLaunchArgument('line.duration_seconds', default_value='60.0'),
        DeclareLaunchArgument('publish_rate', default_value='20.0'),
        DeclareLaunchArgument('motion.mode', default_value='line'),
        DeclareLaunchArgument('motion.linear_speed', default_value='0.4'),
        DeclareLaunchArgument('motion.yaw_rate', default_value='0.5'),
        DeclareLaunchArgument('scan_rate', default_value='10.0'),
        DeclareLaunchArgument('num_beams', default_value='360'),
        DeclareLaunchArgument('max_range', default_value='30.0'),
        DeclareLaunchArgument('range_noise_std', default_value='0.0'),
        DeclareLaunchArgument('noise_seed', default_value='0'),
        DeclareLaunchArgument('ring_pitch_rad', default_value='0.35'),
        DeclareLaunchArgument('altitude_adapt.enable', default_value='true'),
        DeclareLaunchArgument('altitude_adapt.target_fraction', default_value='0.5'),
        DeclareLaunchArgument('altitude_adapt.min_clearance', default_value='0.41'),
        DeclareLaunchArgument('altitude_adapt.max_vertical_speed', default_value='0.6'),
        DeclareLaunchArgument('altitude_adapt.band_ema_alpha', default_value='0.25'),
        DeclareLaunchArgument('altitude_adapt.min_band_height', default_value='0.8'),
        DeclareLaunchArgument('altitude_adapt.vertical_dot_min', default_value='0.65'),
        DeclareLaunchArgument('altitude_adapt.points_stale_sec', default_value='0.5'),
        DeclareLaunchArgument('cloud_map_rate', default_value='5.0'),
        DeclareLaunchArgument('max_cloud_map_points', default_value='500000'),
        DeclareLaunchArgument('stop_scan_when_trajectory_done', default_value='true'),
    ]

    drone_ns = LaunchConfiguration('drone_ns')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')

    return LaunchDescription([
        *declarations,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            namespace=drone_ns,
            condition=IfCondition(LaunchConfiguration('publish_map_to_odom')),
            arguments=['0', '0', '0', '0', '0', '0', map_frame, odom_frame],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar',
            namespace=drone_ns,
            arguments=['0', '0', '0', '0', '0', '0', base_frame, lidar_frame],
        ),
        Node(
            package='drone_scanner',
            executable='fake_odom',
            name='fake_odom',
            namespace=drone_ns,
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
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'motion.mode': LaunchConfiguration('motion.mode'),
                'motion.linear_speed': _f('motion.linear_speed'),
                'motion.yaw_rate': _f('motion.yaw_rate'),
                'altitude_adapt.enable': _b('altitude_adapt.enable'),
                'altitude_adapt.target_fraction': _f('altitude_adapt.target_fraction'),
                'altitude_adapt.min_clearance': _f('altitude_adapt.min_clearance'),
                'altitude_adapt.max_vertical_speed': _f('altitude_adapt.max_vertical_speed'),
                'altitude_adapt.band_ema_alpha': _f('altitude_adapt.band_ema_alpha'),
                'altitude_adapt.min_band_height': _f('altitude_adapt.min_band_height'),
                'altitude_adapt.vertical_dot_min': _f('altitude_adapt.vertical_dot_min'),
                'altitude_adapt.ring_pitch_rad': _f('ring_pitch_rad'),
                'altitude_adapt.points_stale_sec': _f('altitude_adapt.points_stale_sec'),
            }],
        ),
        Node(
            package='drone_scanner',
            executable='fake_lidar',
            name='fake_lidar',
            namespace=drone_ns,
            output='screen',
            emulate_tty=True,
            parameters=[{
                **_cave_params(),
                'map_frame': map_frame,
                'lidar_frame': lidar_frame,
                'scan_rate': _f('scan_rate'),
                'num_beams': _i('num_beams'),
                'max_range': _f('max_range'),
                'range_noise_std': _f('range_noise_std'),
                'noise_seed': _i('noise_seed'),
                'ring_pitch_rad': _f('ring_pitch_rad'),
                'stop_scan_when_trajectory_done': _b('stop_scan_when_trajectory_done'),
            }],
        ),
        Node(
            package='drone_scanner',
            executable='scan_accumulator',
            name='scan_accumulator',
            namespace=drone_ns,
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('enable_scan_accumulator')),
            parameters=[{
                'map_frame': map_frame,
                'points_topic': 'points',
                'cloud_map_topic': 'cloud_map',
                'publish_rate': _f('cloud_map_rate'),
                'max_points': _i('max_cloud_map_points'),
            }],
        ),
    ])
