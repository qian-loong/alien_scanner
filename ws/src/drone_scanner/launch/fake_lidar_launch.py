"""Phase 2 compatible entry: cave + drone_sensing_stack + optional dual RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _cave_arg_names():
    return [
        'cave_mode', 'seed', 'base_radius', 'n_segments', 'density', 'noise_scale',
        'length', 'branch_length', 'branch', 'branch_angle', 'chamber_at', 'chamber_scale',
        'tree.approach_length', 'tree.loop_yaw', 'tree.loop_direct_length', 'tree.loop_bulge',
        'tree.exit1_length', 'tree.right_yaw', 'tree.right_corridor_length',
        'tree.exit_yaw_spread', 'tree.exit_arm_length', 'tree.vertical_step',
        'tree.asymmetry', 'tree.chamber_on_approach', 'tree.chamber_at', 'tree.chamber_scale',
    ]


def generate_launch_description():
    show_cave = DeclareLaunchArgument(
        'show_cave', default_value='true',
        description='启动 cave_publisher；真值 /cave/points 由 drone_sim.rviz 显示（默认开）')
    show_rviz_sim = DeclareLaunchArgument(
        'show_rviz_sim', default_value='true',
        description='启动仿真 RViz（飞机 + 当前扫描环）')
    show_rviz_map = DeclareLaunchArgument(
        'show_rviz_map', default_value='true',
        description='启动地图 RViz（仅累积 cloud_map）')

    # Sensing-stack args re-declared so this entry remains a drop-in Phase 2 launch.
    sensing_defaults = [
        DeclareLaunchArgument('drone_ns', default_value='drone_0'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('lidar_frame', default_value='lidar_link'),
        DeclareLaunchArgument('publish_map_to_odom', default_value='true'),
        DeclareLaunchArgument('enable_scan_accumulator', default_value='true'),
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
        DeclareLaunchArgument(
            'ring_pitch_rad', default_value='0.35',
            description='扫描环绕机体 +Y 前倾角 (rad)；0=纯 YZ；默认约 20° 消除正前盲区'),
        DeclareLaunchArgument(
            'altitude_adapt.enable', default_value='true',
            description='根据环扫顶/底自适应飞行高度（不读洞穴真值）'),
        DeclareLaunchArgument('altitude_adapt.target_fraction', default_value='0.5'),
        DeclareLaunchArgument('altitude_adapt.min_clearance', default_value='0.41'),
        DeclareLaunchArgument(
            'altitude_adapt.max_vertical_speed', default_value='0.6',
            description='高度跟随 |vz| 上限 (m/s)'),
        DeclareLaunchArgument(
            'altitude_adapt.band_ema_alpha', default_value='0.25',
            description='顶/底估计 EMA 系数，越小越平滑'),
        DeclareLaunchArgument('altitude_adapt.min_band_height', default_value='0.8'),
        DeclareLaunchArgument('altitude_adapt.vertical_dot_min', default_value='0.65'),
        DeclareLaunchArgument(
            'altitude_adapt.points_stale_sec', default_value='0.5',
            description='扫描帧超过该时长(s)则丢弃，不更新顶底'),
        DeclareLaunchArgument('cloud_map_rate', default_value='5.0'),
        DeclareLaunchArgument('max_cloud_map_points', default_value='500000'),
        DeclareLaunchArgument(
            'stop_scan_when_trajectory_done', default_value='true',
            description='轨迹结束后停止 fake_lidar 扫描，避免停飞空转占满累积上限'),
    ]

    cave_publisher_launch = PathJoinSubstitution([
        FindPackageShare('cave_world'),
        'launch',
        'cave_publisher_launch.py',
    ])
    sensing_stack_launch = PathJoinSubstitution([
        FindPackageShare('drone_scanner'),
        'launch',
        'drone_sensing_stack.launch.py',
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

    sensing_forward = {
        key: LaunchConfiguration(key)
        for key in [
            'drone_ns', 'map_frame', 'odom_frame', 'base_frame', 'lidar_frame',
            'publish_map_to_odom', 'enable_scan_accumulator',
            *_cave_arg_names(),
            'line.start_x', 'line.start_y', 'line.start_z',
            'line.end_x', 'line.end_y', 'line.end_z', 'line.duration_seconds',
            'publish_rate', 'motion.mode', 'motion.linear_speed', 'motion.yaw_rate',
            'scan_rate', 'num_beams', 'max_range', 'range_noise_std', 'noise_seed',
            'ring_pitch_rad',
            'altitude_adapt.enable', 'altitude_adapt.target_fraction',
            'altitude_adapt.min_clearance', 'altitude_adapt.max_vertical_speed',
            'altitude_adapt.band_ema_alpha', 'altitude_adapt.min_band_height',
            'altitude_adapt.vertical_dot_min', 'altitude_adapt.points_stale_sec',
            'cloud_map_rate', 'max_cloud_map_points', 'stop_scan_when_trajectory_done',
        ]
    }

    return LaunchDescription([
        show_cave,
        show_rviz_sim,
        show_rviz_map,
        *sensing_defaults,
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(cave_publisher_launch),
                    condition=IfCondition(LaunchConfiguration('show_cave')),
                    launch_arguments={
                        key: LaunchConfiguration(key) for key in _cave_arg_names()
                    }.items(),
                ),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensing_stack_launch),
            launch_arguments=sensing_forward.items(),
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
