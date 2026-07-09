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
    cave_mode = DeclareLaunchArgument('cave_mode', default_value='tree')
    seed = DeclareLaunchArgument('seed', default_value='42')
    base_radius = DeclareLaunchArgument('base_radius', default_value='2.5')
    n_segments = DeclareLaunchArgument('n_segments', default_value='200')
    density = DeclareLaunchArgument('density', default_value='400')
    noise_scale = DeclareLaunchArgument('noise_scale', default_value='0.4')
    length = DeclareLaunchArgument('length', default_value='40.0')
    branch_length = DeclareLaunchArgument('branch_length', default_value='20.0')
    branch = DeclareLaunchArgument('branch', default_value='true')
    branch_angle = DeclareLaunchArgument('branch_angle', default_value='0.55')
    chamber_at = DeclareLaunchArgument('chamber_at', default_value='0.5')
    chamber_scale = DeclareLaunchArgument('chamber_scale', default_value='3.0')
    tree_approach_length = DeclareLaunchArgument('tree.approach_length', default_value='12.0')
    tree_loop_yaw = DeclareLaunchArgument('tree.loop_yaw', default_value='0.50')
    tree_loop_direct_length = DeclareLaunchArgument('tree.loop_direct_length', default_value='16.0')
    tree_loop_bulge = DeclareLaunchArgument('tree.loop_bulge', default_value='12.0')
    tree_exit1_length = DeclareLaunchArgument('tree.exit1_length', default_value='14.0')
    tree_right_yaw = DeclareLaunchArgument('tree.right_yaw', default_value='-0.12')
    tree_right_corridor_length = DeclareLaunchArgument('tree.right_corridor_length', default_value='10.0')
    tree_exit_yaw_spread = DeclareLaunchArgument('tree.exit_yaw_spread', default_value='0.35')
    tree_exit_arm_length = DeclareLaunchArgument('tree.exit_arm_length', default_value='14.0')
    tree_vertical_step = DeclareLaunchArgument('tree.vertical_step', default_value='-0.10')
    tree_asymmetry = DeclareLaunchArgument('tree.asymmetry', default_value='0.22')
    tree_chamber_on_approach = DeclareLaunchArgument('tree.chamber_on_approach', default_value='false')
    tree_chamber_at = DeclareLaunchArgument('tree.chamber_at', default_value='0.55')
    tree_chamber_scale = DeclareLaunchArgument('tree.chamber_scale', default_value='2.2')

    line_start_x = DeclareLaunchArgument('line.start_x', default_value='0.0')
    line_start_y = DeclareLaunchArgument('line.start_y', default_value='0.0')
    line_start_z = DeclareLaunchArgument('line.start_z', default_value='1.5')
    line_end_x = DeclareLaunchArgument('line.end_x', default_value='11.0')
    line_end_y = DeclareLaunchArgument('line.end_y', default_value='0.0')
    line_end_z = DeclareLaunchArgument('line.end_z', default_value='1.5')
    line_duration = DeclareLaunchArgument('line.duration_seconds', default_value='60.0')
    publish_rate = DeclareLaunchArgument('publish_rate', default_value='20.0')

    scan_rate = DeclareLaunchArgument('scan_rate', default_value='10.0')
    num_beams = DeclareLaunchArgument('num_beams', default_value='360')
    max_range = DeclareLaunchArgument('max_range', default_value='30.0')
    range_noise_std = DeclareLaunchArgument('range_noise_std', default_value='0.0')
    noise_seed = DeclareLaunchArgument('noise_seed', default_value='0')
    ring_pitch_rad = DeclareLaunchArgument(
        'ring_pitch_rad', default_value='0.35',
        description='扫描环绕机体 +Y 前倾角 (rad)；0=纯 YZ；默认约 20° 消除正前盲区')

    show_cave = DeclareLaunchArgument(
        'show_cave', default_value='true',
        description='启动 cave_publisher；真值 /cave/points 由 drone_sim.rviz 显示（默认开）')
    show_rviz_sim = DeclareLaunchArgument(
        'show_rviz_sim', default_value='true',
        description='启动仿真 RViz（飞机 + 当前扫描环）')
    show_rviz_map = DeclareLaunchArgument(
        'show_rviz_map', default_value='true',
        description='启动地图 RViz（仅累积 cloud_map）')
    cloud_map_rate = DeclareLaunchArgument('cloud_map_rate', default_value='5.0')
    max_cloud_map_points = DeclareLaunchArgument('max_cloud_map_points', default_value='500000')
    stop_scan_when_trajectory_done = DeclareLaunchArgument(
        'stop_scan_when_trajectory_done', default_value='true',
        description='轨迹结束后停止 fake_lidar 扫描，避免停飞空转占满累积上限')

    cave_publisher_launch = PathJoinSubstitution([
        FindPackageShare('cave_world'),
        'launch',
        'cave_publisher_launch.py',
    ])
    rviz_sim_config = PathJoinSubstitution([
        FindPackageShare('drone_scanner'),
        'config',
        'drone_sim.rviz',
    ])
    rviz_map_config = PathJoinSubstitution([
        FindPackageShare('drone_scanner'),
        'config',
        'drone_map.rviz',
    ])

    cave_args = [
        cave_mode, seed, base_radius, n_segments, density, noise_scale,
        length, branch_length, branch, branch_angle, chamber_at, chamber_scale,
        tree_approach_length, tree_loop_yaw, tree_loop_direct_length, tree_loop_bulge,
        tree_exit1_length, tree_right_yaw, tree_right_corridor_length,
        tree_exit_yaw_spread, tree_exit_arm_length, tree_vertical_step,
        tree_asymmetry, tree_chamber_on_approach, tree_chamber_at, tree_chamber_scale,
    ]

    return LaunchDescription([
        *cave_args,
        line_start_x,
        line_start_y,
        line_start_z,
        line_end_x,
        line_end_y,
        line_end_z,
        line_duration,
        publish_rate,
        scan_rate,
        num_beams,
        max_range,
        range_noise_std,
        noise_seed,
        ring_pitch_rad,
        show_cave,
        show_rviz_sim,
        show_rviz_map,
        cloud_map_rate,
        max_cloud_map_points,
        stop_scan_when_trajectory_done,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link'],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cave_publisher_launch),
            condition=IfCondition(LaunchConfiguration('show_cave')),
            launch_arguments={
                key: LaunchConfiguration(key) for key in _cave_params()
            }.items(),
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
            package='drone_scanner',
            executable='fake_lidar',
            name='fake_lidar',
            namespace='drone_0',
            output='screen',
            emulate_tty=True,
            parameters=[{
                **_cave_params(),
                'map_frame': 'map',
                'lidar_frame': 'lidar_link',
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
            namespace='drone_0',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'map_frame': 'map',
                'points_topic': 'points',
                'cloud_map_topic': 'cloud_map',
                'publish_rate': _f('cloud_map_rate'),
                'max_points': _i('max_cloud_map_points'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_sim',
            output='screen',
            condition=IfCondition(LaunchConfiguration('show_rviz_sim')),
            arguments=['-d', rviz_sim_config],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_map',
            output='screen',
            condition=IfCondition(LaunchConfiguration('show_rviz_map')),
            arguments=['-d', rviz_map_config],
        ),
    ])
