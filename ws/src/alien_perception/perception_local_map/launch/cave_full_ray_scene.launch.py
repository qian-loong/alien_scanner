import math
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


sys.path.insert(0, str(Path(__file__).resolve().parent))
from cave_full_ray_scene_config import (  # noqa: E402,I100
    cave_parameters,
    contract_parameters,
    load_scene_config,
)


def _launch_setup(context):
    config_path = LaunchConfiguration("scene_config").perform(context)
    scanner_startup_delay_s = float(
        LaunchConfiguration("scanner_startup_delay_s").perform(context)
    )
    if not math.isfinite(scanner_startup_delay_s) or scanner_startup_delay_s < 0.0:
        raise ValueError("scanner_startup_delay_s must be finite and non-negative")
    map_update_enabled_text = LaunchConfiguration("map_update_enabled").perform(context)
    if map_update_enabled_text.lower() not in ("true", "false"):
        raise ValueError("map_update_enabled must be true or false")
    map_update_enabled = map_update_enabled_text.lower() == "true"
    config = load_scene_config(config_path)
    frames = config["frames"]
    topics = config["topics"]
    cave = cave_parameters(config)
    trajectory = config["trajectory"]
    scan = config["scan"]
    timing = config["timing"]
    contract = config["sensor_contract"]
    map_config = config["map"]
    derived = config["derived"]
    common_contract = contract_parameters(config)
    sensor_id = contract["sensor_id"]
    quaternion = scan["body_from_scan_quaternion_xyzw"]

    cave_publisher = Node(
        package="cave_world",
        executable="cave_publisher",
        name="cave_full_ray_truth",
        output="screen",
        parameters=[
            {
                **cave,
                "frame_id": frames["map"],
                "topic": topics["cave_truth"],
                "publish_rate": 1.0,
            }
        ],
    )

    fake_odom = Node(
        package="drone_scanner",
        executable="fake_odom",
        name="cave_full_ray_odom",
        output="screen",
        parameters=[
            {
                "line.start_x": trajectory["start_m"][0],
                "line.start_y": trajectory["start_m"][1],
                "line.start_z": trajectory["start_m"][2],
                "line.end_x": trajectory["end_m"][0],
                "line.end_y": trajectory["end_m"][1],
                "line.end_z": trajectory["end_m"][2],
                "line.duration_seconds": trajectory["duration_s"],
                "publish_rate": trajectory["odometry_rate_hz"],
                "odom_frame": frames["map"],
                "base_frame": frames["body"],
                "motion.mode": trajectory["motion_mode"],
                "altitude_adapt.enable": trajectory["altitude_adapt"],
            }
        ],
        remappings=[("odom", topics["odometry"]), ("path", topics["path"])],
    )

    scanner = Node(
        package="drone_scanner",
        executable="cave_laser_scan",
        name="cave_full_ray_scanner",
        output="screen",
        parameters=[
            {
                **cave,
                "map_frame": frames["map"],
                "base_frame": frames["body"],
                "scan_frame": frames["scan"],
                "odom_topic": topics["odometry"],
                "scan_topic": topics["raw_scan"],
                "beam_count": scan["beam_count"],
                "range_min_m": scan["range_min_m"],
                "range_max_m": scan["range_max_m"],
                "scan_rate_hz": scan["rate_hz"],
                "range_noise_std": scan["range_noise_std_m"],
                "noise_seed": scan["noise_seed"],
            }
        ],
    )
    scanner_action = (
        scanner
        if scanner_startup_delay_s == 0.0
        else TimerAction(period=scanner_startup_delay_s, actions=[scanner])
    )

    pose_gate = Node(
        package="perception_fixtures",
        executable="pose_gated_laser_scan_relay",
        name="cave_full_ray_pose_gate",
        output="screen",
        parameters=[
            {
                "raw_scan_topic": topics["raw_scan"],
                "released_scan_topic": topics["released_scan"],
                "pose_topic": topics["pose"],
                "expected_pose_frame": frames["map"],
                "expected_pose_source": contract["pose_source_id"],
                "expected_clock_domain": contract["clock_domain"],
                "pose_lead_delay_s": timing["pose_lead_delay_s"],
                "odom_period_s": derived["odom_period_s"],
                "pending_timeout_s": timing["pending_timeout_s"],
                "max_pending_scans": timing["max_pending_scans"],
            }
        ],
    )

    input_parameters = {
        **common_contract,
        "producer_source_id": "cave_scene_perception_input",
        "pose_source_id": contract["pose_source_id"],
        "pose_input_type": "odometry",
        "clock_domain": contract["clock_domain"],
        "sensor_timeout_s": contract["sensor_timeout_s"],
        "health_period_s": contract["health_period_s"],
        "observation_topic": topics["observations"],
        "pose_topic": topics["pose"],
        "health_topic": topics["health"],
        "odom_topic": topics["odometry"],
        f"sensor.{sensor_id}.topic": topics["released_scan"],
    }
    perception_input = Node(
        package="perception_input_node",
        executable="perception_input_node",
        name="cave_full_ray_perception_input",
        output="screen",
        parameters=[input_parameters],
    )

    lattice_origin = map_config["lattice_origin_m"]
    local_map_parameters = {
        **common_contract,
        "backend_type": "octomap",
        "vehicle_id": map_config["vehicle_id"],
        "source_local_map_frame": frames["map"],
        "body_frame": frames["body"],
        "resolution_m": map_config["resolution_m"],
        "lattice_origin_x": lattice_origin[0],
        "lattice_origin_y": lattice_origin[1],
        "lattice_origin_z": lattice_origin[2],
        "observation_topic": topics["observations"],
        "pose_topic": topics["pose"],
        "health_topic": topics["health"],
        "state_topic": topics["local_map_state"],
        "octomap_topic": topics["local_map_octomap"],
        "map_update_enabled": map_update_enabled,
        "map_update_topic": topics["local_map_updates"],
        "map_resync_service": topics["local_map_resync"],
        "sensor_timeout_s": contract["sensor_timeout_s"],
        "health_timeout_s": 1.0,
        "pose_receive_timeout_s": contract["pose_timeout_s"],
        "map_freshness_s": map_config["map_freshness_s"],
        "state_heartbeat_period_s": map_config["state_heartbeat_period_s"],
    }
    local_map = Node(
        package="perception_local_map",
        executable="perception_local_map_node",
        name="cave_full_ray_local_map",
        output="screen",
        parameters=[local_map_parameters],
    )

    scan_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="cave_full_ray_body_to_scan",
        output="screen",
        arguments=[
            "0",
            "0",
            "0",
            str(quaternion[0]),
            str(quaternion[1]),
            str(quaternion[2]),
            str(quaternion[3]),
            frames["body"],
            frames["scan"],
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="cave_full_ray_rviz",
        output="screen",
        arguments=[
            "-d",
            str(
                Path(__file__).resolve().parents[1]
                / "config"
                / "cave_full_ray_scene.rviz"
            ),
        ],
        condition=IfCondition(LaunchConfiguration("show_rviz")),
        additional_env={"LD_PRELOAD": "liboctomap.so"},
    )

    return [
        cave_publisher,
        scan_tf,
        fake_odom,
        scanner_action,
        perception_input,
        pose_gate,
        local_map,
        rviz,
    ]


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [
            FindPackageShare("perception_local_map"),
            "config",
            "cave_full_ray_scene.yaml",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("scene_config", default_value=default_config),
            DeclareLaunchArgument("show_rviz", default_value="true"),
            DeclareLaunchArgument("scanner_startup_delay_s", default_value="0.0"),
            DeclareLaunchArgument("map_update_enabled", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
