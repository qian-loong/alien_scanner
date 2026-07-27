import math
from pathlib import Path

import yaml


SCHEMA_VERSION = 1


def default_config_path():
    return Path(__file__).resolve().parents[1] / "config" / "cave_full_ray_scene.yaml"


def _require(mapping, key, expected_type):
    if key not in mapping or not isinstance(mapping[key], expected_type):
        raise ValueError(f"scene config field '{key}' has the wrong type")
    return mapping[key]


def _positive(value, name):
    if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"scene config field '{name}' must be finite and positive")
    return value


def load_scene_config(path=None):
    config_path = Path(path) if path is not None else default_config_path()
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or config.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("unsupported cave FullRay scene config schema")

    frames = _require(config, "frames", dict)
    topics = _require(config, "topics", dict)
    cave = _require(config, "cave", dict)
    trajectory = _require(config, "trajectory", dict)
    scan = _require(config, "scan", dict)
    timing = _require(config, "timing", dict)
    contract = _require(config, "sensor_contract", dict)
    map_config = _require(config, "map", dict)

    for name in ("map", "body", "scan"):
        if not isinstance(frames.get(name), str) or not frames[name]:
            raise ValueError(f"scene frame '{name}' must not be empty")
    for name, value in topics.items():
        if not isinstance(value, str) or not value.startswith("/"):
            raise ValueError(f"scene topic '{name}' must be absolute")

    if cave.get("mode") != "tree" or cave.get("seed") != 42:
        raise ValueError("scene cave must remain tree/seed=42")
    if not math.isclose(cave.get("base_radius_m", 0.0), 2.5):
        raise ValueError("scene cave base radius must remain 2.5 m")
    if (
        trajectory.get("start_m") != [1.0, 0.0, 1.5]
        or trajectory.get("end_m") != [11.0, 0.0, 1.5]
        or not math.isclose(trajectory.get("duration_s", 0.0), 20.0)
        or not math.isclose(trajectory.get("odometry_rate_hz", 0.0), 20.0)
        or trajectory.get("motion_mode") != "line"
        or trajectory.get("altitude_adapt") is not False
    ):
        raise ValueError("scene trajectory differs from the frozen 10 m/20 s line")
    if (
        scan.get("beam_count") != 360
        or not math.isclose(scan.get("rate_hz", 0.0), 10.0)
        or not math.isclose(scan.get("range_min_m", 0.0), 0.1)
        or not math.isclose(scan.get("range_max_m", 0.0), 30.0)
        or not math.isclose(scan.get("range_noise_std_m", -1.0), 0.0)
        or scan.get("body_from_scan_quaternion_xyzw") != [0.5, 0.5, 0.5, 0.5]
    ):
        raise ValueError("scene scan differs from the frozen FullRay geometry")
    _positive(timing.get("pose_lead_delay_s"), "timing.pose_lead_delay_s")
    odom_period = 1.0 / trajectory["odometry_rate_hz"]
    if timing["pose_lead_delay_s"] < 2.0 * odom_period:
        raise ValueError("pose lead delay must cover at least two odometry periods")
    if (
        contract.get("sensor_type") != "2d"
        or contract.get("ray_evidence") != "full_ray"
        or contract.get("requires_pose") is not True
        or contract.get("expected_pose_frame") != frames["map"]
    ):
        raise ValueError("scene sensor contract must remain one pose-gated 2D FullRay sensor")
    _positive(map_config.get("resolution_m"), "map.resolution_m")

    angle_increment = 2.0 * math.pi / scan["beam_count"]
    config["derived"] = {
        "angle_min_rad": -math.pi,
        "angle_increment_rad": angle_increment,
        "angle_max_rad": -math.pi + (scan["beam_count"] - 1) * angle_increment,
        "odom_period_s": odom_period,
        "scan_time_s": 1.0 / scan["rate_hz"],
    }
    return config


def cave_parameters(config):
    cave = config["cave"]
    tree = cave["tree"]
    return {
        "cave_mode": cave["mode"],
        "seed": cave["seed"],
        "base_radius": cave["base_radius_m"],
        "n_segments": cave["n_segments"],
        "density": cave["density"],
        "noise_scale": cave["noise_scale"],
        "tree.approach_length": tree["approach_length_m"],
        "tree.loop_yaw": tree["loop_yaw_rad"],
        "tree.loop_direct_length": tree["loop_direct_length_m"],
        "tree.loop_bulge": tree["loop_bulge_m"],
        "tree.exit1_length": tree["exit1_length_m"],
        "tree.right_yaw": tree["right_yaw_rad"],
        "tree.right_corridor_length": tree["right_corridor_length_m"],
        "tree.exit_yaw_spread": tree["exit_yaw_spread_rad"],
        "tree.exit_arm_length": tree["exit_arm_length_m"],
        "tree.vertical_step": tree["vertical_step_rad"],
        "tree.asymmetry": tree["asymmetry"],
        "tree.chamber_on_approach": tree["chamber_on_approach"],
        "tree.chamber_at": tree["chamber_at"],
        "tree.chamber_scale": tree["chamber_scale"],
    }


def contract_parameters(config):
    frames = config["frames"]
    scan = config["scan"]
    derived = config["derived"]
    contract = config["sensor_contract"]
    sensor_id = contract["sensor_id"]
    quaternion = scan["body_from_scan_quaternion_xyzw"]
    return {
        "requires_pose": contract["requires_pose"],
        "expected_pose_frame": contract["expected_pose_frame"],
        "pose_timeout_s": contract["pose_timeout_s"],
        "minimum_pose_quality": contract["minimum_pose_quality"],
        "recovery_stability_samples": contract["recovery_stability_samples"],
        "minimum_lidar_type": contract["sensor_type"],
        "minimum_lidar_count": 1,
        "minimum_lidar_ray_evidence": contract["ray_evidence"],
        "degraded_lidar_type": contract["sensor_type"],
        "degraded_lidar_count": 1,
        "degraded_lidar_ray_evidence": contract["ray_evidence"],
        "sensor_ids": [sensor_id],
        f"sensor.{sensor_id}.type": contract["sensor_type"],
        f"sensor.{sensor_id}.frame_id": frames["scan"],
        f"sensor.{sensor_id}.ray_evidence": contract["ray_evidence"],
        f"sensor.{sensor_id}.mounting_x": 0.0,
        f"sensor.{sensor_id}.mounting_y": 0.0,
        f"sensor.{sensor_id}.mounting_z": 0.0,
        f"sensor.{sensor_id}.mounting_qw": quaternion[3],
        f"sensor.{sensor_id}.mounting_qx": quaternion[0],
        f"sensor.{sensor_id}.mounting_qy": quaternion[1],
        f"sensor.{sensor_id}.mounting_qz": quaternion[2],
        f"sensor.{sensor_id}.fov_horizontal_min_rad": derived["angle_min_rad"],
        f"sensor.{sensor_id}.fov_horizontal_max_rad": derived["angle_max_rad"],
        f"sensor.{sensor_id}.fov_vertical_min_rad": 0.0,
        f"sensor.{sensor_id}.fov_vertical_max_rad": 0.0,
        f"sensor.{sensor_id}.angular_resolution_rad": derived["angle_increment_rad"],
        f"sensor.{sensor_id}.range_min_m": scan["range_min_m"],
        f"sensor.{sensor_id}.range_max_m": scan["range_max_m"],
    }
