from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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


def generate_launch_description():
    cave_mode = DeclareLaunchArgument('cave_mode', default_value='tree', description='tree | y')
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

    rviz_config = PathJoinSubstitution([
        FindPackageShare('cave_world'),
        'config',
        'cave_world.rviz',
    ])

    cave_publisher = Node(
        package='cave_world',
        executable='cave_publisher',
        name='cave_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[{
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
        }],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        cave_mode,
        seed,
        base_radius,
        n_segments,
        density,
        noise_scale,
        length,
        branch_length,
        branch,
        branch_angle,
        chamber_at,
        chamber_scale,
        tree_approach_length,
        tree_loop_yaw,
        tree_loop_direct_length,
        tree_loop_bulge,
        tree_exit1_length,
        tree_right_yaw,
        tree_right_corridor_length,
        tree_exit_yaw_spread,
        tree_exit_arm_length,
        tree_vertical_step,
        tree_asymmetry,
        tree_chamber_on_approach,
        tree_chamber_at,
        tree_chamber_scale,
        cave_publisher,
        rviz2,
    ])
