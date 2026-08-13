import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _positive_float(context, name):
    value = float(LaunchConfiguration(name).perform(context))
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def _non_negative_float(context, name):
    value = float(LaunchConfiguration(name).perform(context))
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return value


def _cave_parameters():
    return {
        "cave_mode": "tree",
        "seed": 42,
        "base_radius": 2.5,
        "n_segments": 200,
        "density": 400,
        "noise_scale": 0.4,
        "tree.approach_length": 12.0,
        "tree.loop_yaw": 0.50,
        "tree.loop_direct_length": 16.0,
        "tree.loop_bulge": 12.0,
        "tree.exit1_length": 14.0,
        "tree.right_yaw": -0.12,
        "tree.right_corridor_length": 10.0,
        "tree.exit_yaw_spread": 0.35,
        "tree.exit_arm_length": 14.0,
        "tree.vertical_step": -0.10,
        "tree.asymmetry": 0.22,
        "tree.chamber_on_approach": False,
        "tree.chamber_at": 0.55,
        "tree.chamber_scale": 2.2,
    }


def _contract_parameters(label, body_frame, scan_frame, scan_rate_hz):
    sensor_id = f"{label}_scan"
    beam_count = 360
    angle_increment = 2.0 * math.pi / beam_count
    return {
        "requires_pose": True,
        "expected_pose_frame": "map",
        "pose_timeout_s": 1.0,
        "minimum_pose_quality": 0.0,
        "recovery_stability_samples": 3,
        "minimum_lidar_type": "2d",
        "minimum_lidar_count": 1,
        "minimum_lidar_ray_evidence": "full_ray",
        "degraded_lidar_type": "2d",
        "degraded_lidar_count": 1,
        "degraded_lidar_ray_evidence": "full_ray",
        "sensor_ids": [sensor_id],
        f"sensor.{sensor_id}.type": "2d",
        f"sensor.{sensor_id}.frame_id": scan_frame,
        f"sensor.{sensor_id}.ray_evidence": "full_ray",
        f"sensor.{sensor_id}.mounting_x": 0.0,
        f"sensor.{sensor_id}.mounting_y": 0.0,
        f"sensor.{sensor_id}.mounting_z": 0.0,
        f"sensor.{sensor_id}.mounting_qw": 0.5,
        f"sensor.{sensor_id}.mounting_qx": 0.5,
        f"sensor.{sensor_id}.mounting_qy": 0.5,
        f"sensor.{sensor_id}.mounting_qz": 0.5,
        f"sensor.{sensor_id}.fov_horizontal_min_rad": -math.pi,
        f"sensor.{sensor_id}.fov_horizontal_max_rad": (
            -math.pi + (beam_count - 1) * angle_increment
        ),
        f"sensor.{sensor_id}.fov_vertical_min_rad": 0.0,
        f"sensor.{sensor_id}.fov_vertical_max_rad": 0.0,
        f"sensor.{sensor_id}.angular_resolution_rad": angle_increment,
        f"sensor.{sensor_id}.range_min_m": 0.1,
        f"sensor.{sensor_id}.range_max_m": 30.0,
        f"sensor.{sensor_id}.topic": f"/c4/cave/{label}/scan",
        "body_frame": body_frame,
        "sensor_timeout_s": max(0.75, 3.0 / scan_rate_hz),
        "health_period_s": 0.2,
    }


def _source_nodes(
    label,
    y,
    duration_s,
    odom_rate_hz,
    scan_rate_hz,
    scanner_startup_delay_s,
):
    prefix = f"/c4/cave/{label}"
    body_frame = f"{label}/base_link"
    scan_frame = f"{label}/scan_link"
    vehicle_id = f"c4-{label}"
    contract = _contract_parameters(label, body_frame, scan_frame, scan_rate_hz)
    odom_topic = f"{prefix}/odom"
    pose_topic = f"{prefix}/perception/pose"
    observation_topic = f"{prefix}/perception/observations"
    health_topic = f"{prefix}/perception/health"

    odometry = Node(
        package="drone_scanner",
        executable="fake_odom",
        name=f"c4_cave_{label}_odom",
        output="screen",
        parameters=[
            {
                "line.start_x": 0.0,
                "line.start_y": y,
                "line.start_z": 1.5,
                "line.end_x": 8.0,
                "line.end_y": y,
                "line.end_z": 1.5,
                "line.duration_seconds": duration_s,
                "publish_rate": odom_rate_hz,
                "odom_frame": "map",
                "base_frame": body_frame,
                "motion.mode": "line",
                "altitude_adapt.enable": False,
            }
        ],
        remappings=[("odom", odom_topic), ("path", f"{prefix}/path")],
    )

    scanner = Node(
        package="drone_scanner",
        executable="cave_laser_scan",
        name=f"c4_cave_{label}_scanner",
        output="screen",
        parameters=[
            {
                **_cave_parameters(),
                "map_frame": "map",
                "base_frame": body_frame,
                "scan_frame": scan_frame,
                "odom_topic": odom_topic,
                "scan_topic": f"{prefix}/raw_scan",
                "beam_count": 360,
                "range_min_m": 0.1,
                "range_max_m": 30.0,
                "scan_rate_hz": scan_rate_hz,
                "range_noise_std": 0.0,
                "noise_seed": 101 if label == "a1" else 202,
            }
        ],
    )

    perception_input = Node(
        package="perception_input_node",
        executable="perception_input_node",
        name=f"c4_cave_{label}_perception_input",
        output="screen",
        parameters=[
            {
                **contract,
                "producer_source_id": f"c4_cave_{label}_perception_input",
                "pose_source_id": "odom",
                "pose_input_type": "odometry",
                "clock_domain": "vehicle_steady_clock",
                "observation_topic": observation_topic,
                "pose_topic": pose_topic,
                "health_topic": health_topic,
                "odom_topic": odom_topic,
            }
        ],
    )

    pose_gate = Node(
        package="perception_fixtures",
        executable="pose_gated_laser_scan_relay",
        name=f"c4_cave_{label}_pose_gate",
        output="screen",
        parameters=[
            {
                "raw_scan_topic": f"{prefix}/raw_scan",
                "released_scan_topic": f"{prefix}/scan",
                "pose_topic": pose_topic,
                "expected_pose_frame": "map",
                "expected_pose_source": "odom",
                "expected_clock_domain": "vehicle_steady_clock",
                "pose_lead_delay_s": 0.1,
                "odom_period_s": 1.0 / odom_rate_hz,
                "pending_timeout_s": 1.0,
                "max_pending_scans": 64,
            }
        ],
    )

    local_map = Node(
        package="perception_local_map",
        executable="perception_local_map_node",
        name=f"c4_cave_{label}_local_map",
        output="screen",
        parameters=[
            {
                **contract,
                "backend_type": "octomap",
                "vehicle_id": vehicle_id,
                "source_local_map_frame": "map",
                "body_frame": body_frame,
                "resolution_m": 0.2,
                "lattice_origin_x": 0.0,
                "lattice_origin_y": 0.0,
                "lattice_origin_z": 0.0,
                "observation_topic": observation_topic,
                "pose_topic": pose_topic,
                "health_topic": health_topic,
                "state_topic": f"{prefix}/local_map/state",
                "octomap_topic": f"{prefix}/local_map/octomap",
                "map_update_enabled": True,
                "map_update_topic": f"{prefix}/local_map/updates",
                "map_resync_service": f"{prefix}/local_map/request_resync",
                "sensor_timeout_s": max(0.75, 3.0 / scan_rate_hz),
                "health_timeout_s": 1.0,
                "pose_receive_timeout_s": 1.0,
                "map_freshness_s": 1.5,
                "state_heartbeat_period_s": 0.2,
            }
        ],
    )

    route_source = Node(
        package="swarm_data_plane",
        executable="map_update_route_source_node",
        name=f"c4_cave_{label}_route_source",
        output="screen",
        parameters=[
            {
                "input_topic": f"{prefix}/local_map/updates",
                "output_topic": f"{prefix}/routed_map_updates",
                "diagnostic_topic": f"{prefix}/data_plane_diagnostics",
                "producer_id": f"c4-cave-{label}-map-endpoint",
                "producer_session_boot_time_ns": 4001 if label == "a1" else 4002,
                "producer_session_random_suffix": 1 if label == "a1" else 2,
                "origin_clock_domain": "vehicle_steady_clock",
                "origin_clock_session_boot_time_ns": 4101 if label == "a1" else 4102,
                "origin_clock_session_random_suffix": 1 if label == "a1" else 2,
                "route_epoch": 1,
                "ttl_hops": 4,
                "validity_budget_ns": 10_000_000_000,
                "qos_depth": 8,
            }
        ],
    )

    scan_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"c4_cave_{label}_body_to_scan",
        output="screen",
        arguments=[
            "0",
            "0",
            "0",
            "0.5",
            "0.5",
            "0.5",
            "0.5",
            body_frame,
            scan_frame,
        ],
    )
    return [
        scan_tf,
        odometry,
        perception_input,
        pose_gate,
        local_map,
        route_source,
        TimerAction(period=scanner_startup_delay_s, actions=[scanner]),
    ]


def _launch_setup(context):
    a_duration_s = _positive_float(context, "a_duration_s")
    b_duration_s = _positive_float(context, "b_duration_s")
    odom_rate_hz = _positive_float(context, "odom_rate_hz")
    scan_rate_hz = _positive_float(context, "scan_rate_hz")
    resync_hold_s = _non_negative_float(context, "resync_hold_s")
    scanner_startup_delay_s = _non_negative_float(
        context, "scanner_startup_delay_s"
    )

    cave_truth = Node(
        package="cave_world",
        executable="cave_publisher",
        name="c4_cave_truth",
        output="screen",
        parameters=[
            {
                **_cave_parameters(),
                "frame_id": "map",
                "topic": "/c4/cave/truth",
                "publish_rate": 1.0,
            }
        ],
    )

    b_odometry = Node(
        package="drone_scanner",
        executable="fake_odom",
        name="c4_cave_b_odom",
        output="screen",
        parameters=[
            {
                "line.start_x": 0.0,
                "line.start_y": 0.0,
                "line.start_z": 1.5,
                "line.end_x": 3.5,
                "line.end_y": 0.0,
                "line.end_z": 1.5,
                "line.duration_seconds": b_duration_s,
                "publish_rate": odom_rate_hz,
                "odom_frame": "map",
                "base_frame": "b/base_link",
                "motion.mode": "line",
                "altitude_adapt.enable": False,
            }
        ],
        remappings=[("odom", "/c4/cave/b/odom"), ("path", "/c4/cave/b/path")],
    )

    receiver = Node(
        package="swarm_data_plane",
        executable="c4_cave_receiver_fixture",
        name="c4_cave_receiver_fixture",
        output="screen",
        parameters=[
            {
                "frame_id": "map",
                "resync_hold_s": resync_hold_s,
                "publish_rate_hz": 5.0,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="c4_cave_visual_validation_rviz",
        output="screen",
        arguments=[
            "-d",
            str(
                Path(__file__).resolve().parents[1]
                / "config"
                / "c4_cave_visual_validation.rviz"
            ),
        ],
        condition=IfCondition(LaunchConfiguration("show_rviz")),
        additional_env={"LD_PRELOAD": "liboctomap.so"},
    )

    return [
        cave_truth,
        *_source_nodes(
            "a1",
            1.0,
            a_duration_s,
            odom_rate_hz,
            scan_rate_hz,
            scanner_startup_delay_s,
        ),
        *_source_nodes(
            "a2",
            -1.0,
            a_duration_s,
            odom_rate_hz,
            scan_rate_hz,
            scanner_startup_delay_s,
        ),
        b_odometry,
        receiver,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("show_rviz", default_value="true"),
            DeclareLaunchArgument("a_duration_s", default_value="20.0"),
            DeclareLaunchArgument("b_duration_s", default_value="9.0"),
            DeclareLaunchArgument("odom_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("scan_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("resync_hold_s", default_value="4.0"),
            DeclareLaunchArgument("scanner_startup_delay_s", default_value="1.0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
