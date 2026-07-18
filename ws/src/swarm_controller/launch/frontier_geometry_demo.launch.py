"""Deterministic single-drone observation and frontier geometry replay."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution([
        FindPackageShare('swarm_controller'),
        'config',
        'frontier_geometry_demo.rviz',
    ])

    demo = Node(
        package='swarm_controller',
        executable='frontier_geometry_demo',
        name='frontier_geometry_demo',
        output='screen',
        parameters=[{
            'scene.mode': LaunchConfiguration('mode'),
            'scene.stage': LaunchConfiguration('stage'),
            'scene.compose_stages': LaunchConfiguration('compose_stages'),
            'tunnel.radius': LaunchConfiguration('tunnel_radius'),
            'tunnel.length': LaunchConfiguration('tunnel_length'),
            'tunnel.center_z': LaunchConfiguration('tunnel_center_z'),
            'cave.seed': LaunchConfiguration('cave_seed'),
            'scan.pitch_degrees': LaunchConfiguration('pitch_degrees'),
            'scan.selected_phi_degrees': LaunchConfiguration(
                'selected_phi_degrees'),
            'scan.ray_count': LaunchConfiguration('ray_count'),
            'scan.yaw_steps': LaunchConfiguration('yaw_steps'),
            'scan.max_range': LaunchConfiguration('max_range'),
            'observation.initial_x': LaunchConfiguration('initial_x'),
            'observation.initial_y': LaunchConfiguration('initial_y'),
            'observation.hop_distance': LaunchConfiguration('hop_distance'),
            'scan.ring_segments': LaunchConfiguration('ring_segments'),
            'scan.show_full_ring': LaunchConfiguration('show_full_ring'),
            'voxel.resolution': LaunchConfiguration('voxel_resolution'),
            'voxel.radial_samples': LaunchConfiguration('voxel_radial_samples'),
            'voxel.angular_samples': LaunchConfiguration('voxel_angular_samples'),
            'voxel.slice_layers': LaunchConfiguration('voxel_slice_layers'),
            'voxel.visual_scale': LaunchConfiguration('voxel_visual_scale'),
            'frontier.column_stride_voxels': LaunchConfiguration(
                'column_stride_voxels'),
            'frontier.min_z_layers': LaunchConfiguration('min_z_layers'),
            'frontier.min_z_span': LaunchConfiguration('min_z_span'),
            'frontier.support_depth': LaunchConfiguration('support_depth'),
            'fixture.min_component_columns': LaunchConfiguration(
                'min_component_columns'),
            'frontier.max_trace_candidates': LaunchConfiguration(
                'max_trace_candidates'),
            'frontier.max_trace_support_samples': LaunchConfiguration(
                'max_trace_support_samples'),
            'frontier.max_trace_components': LaunchConfiguration(
                'max_trace_components'),
            'frontier.max_trace_geometry_elements': LaunchConfiguration(
                'max_trace_geometry_elements'),
            'display.show_labels': LaunchConfiguration('show_labels'),
            'display.show_unknown': LaunchConfiguration('show_unknown'),
            'display.show_reference_geometry': LaunchConfiguration(
                'show_reference_geometry'),
            'display.republish_rate_hz': LaunchConfiguration(
                'republish_rate_hz'),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='frontier_geometry_demo_rviz',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='combined'),
        DeclareLaunchArgument('stage', default_value='accumulated_frontier'),
        DeclareLaunchArgument('compose_stages', default_value='true'),
        DeclareLaunchArgument('tunnel_radius', default_value='2.0'),
        DeclareLaunchArgument('tunnel_length', default_value='8.0'),
        DeclareLaunchArgument('tunnel_center_z', default_value='0.0'),
        DeclareLaunchArgument('cave_seed', default_value='42'),
        DeclareLaunchArgument('pitch_degrees', default_value='20.0'),
        DeclareLaunchArgument('selected_phi_degrees', default_value='0.0'),
        DeclareLaunchArgument('ray_count', default_value='144'),
        DeclareLaunchArgument('yaw_steps', default_value='144'),
        DeclareLaunchArgument('max_range', default_value='3.0'),
        DeclareLaunchArgument('initial_x', default_value='1.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('hop_distance', default_value='0.8'),
        DeclareLaunchArgument('ring_segments', default_value='72'),
        DeclareLaunchArgument('show_full_ring', default_value='true'),
        DeclareLaunchArgument('voxel_resolution', default_value='0.1'),
        DeclareLaunchArgument('voxel_radial_samples', default_value='8'),
        DeclareLaunchArgument('voxel_angular_samples', default_value='24'),
        DeclareLaunchArgument('voxel_slice_layers', default_value='3'),
        DeclareLaunchArgument('voxel_visual_scale', default_value='1.0'),
        DeclareLaunchArgument('column_stride_voxels', default_value='2'),
        DeclareLaunchArgument('min_z_layers', default_value='5'),
        DeclareLaunchArgument('min_z_span', default_value='0.4'),
        DeclareLaunchArgument('support_depth', default_value='0.8'),
        DeclareLaunchArgument('min_component_columns', default_value='12'),
        DeclareLaunchArgument('max_trace_candidates', default_value='10000'),
        DeclareLaunchArgument(
            'max_trace_support_samples', default_value='100000'),
        DeclareLaunchArgument('max_trace_components', default_value='10000'),
        DeclareLaunchArgument(
            'max_trace_geometry_elements', default_value='500000'),
        DeclareLaunchArgument('show_labels', default_value='true'),
        DeclareLaunchArgument('show_unknown', default_value='true'),
        DeclareLaunchArgument('show_reference_geometry', default_value='true'),
        DeclareLaunchArgument('republish_rate_hz', default_value='0.0'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        demo,
        rviz,
    ])
