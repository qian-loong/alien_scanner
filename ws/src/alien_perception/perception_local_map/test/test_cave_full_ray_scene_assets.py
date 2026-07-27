import ast
import importlib.util
import math
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE_ROOT / "launch"
CONFIG_DIR = PACKAGE_ROOT / "config"


def _load_config_module():
    module_path = LAUNCH_DIR / "cave_full_ray_scene_config.py"
    spec = importlib.util.spec_from_file_location("cave_full_ray_scene_config", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _display_by_class(rviz_config, class_name):
    displays = rviz_config["Visualization Manager"]["Displays"]
    matches = [display for display in displays if display.get("Class") == class_name]
    assert len(matches) == 1
    return matches[0]


def test_canonical_loader_derives_frozen_geometry_and_contract():
    loader = _load_config_module()
    config = loader.load_scene_config(CONFIG_DIR / "cave_full_ray_scene.yaml")
    scan = config["scan"]
    derived = config["derived"]
    contract = loader.contract_parameters(config)
    sensor_id = config["sensor_contract"]["sensor_id"]

    expected_increment = 2.0 * math.pi / scan["beam_count"]
    assert math.isclose(derived["angle_min_rad"], -math.pi)
    assert math.isclose(derived["angle_increment_rad"], expected_increment)
    assert math.isclose(
        derived["angle_max_rad"],
        math.pi - expected_increment,
    )
    assert math.isclose(derived["scan_time_s"], 1.0 / scan["rate_hz"])
    assert config["timing"]["pose_lead_delay_s"] >= 2.0 * derived["odom_period_s"]

    quaternion_xyzw = scan["body_from_scan_quaternion_xyzw"]
    assert [
        contract[f"sensor.{sensor_id}.mounting_qx"],
        contract[f"sensor.{sensor_id}.mounting_qy"],
        contract[f"sensor.{sensor_id}.mounting_qz"],
        contract[f"sensor.{sensor_id}.mounting_qw"],
    ] == quaternion_xyzw
    assert math.isclose(
        contract[f"sensor.{sensor_id}.fov_horizontal_max_rad"],
        derived["angle_max_rad"],
    )
    assert math.isclose(
        contract[f"sensor.{sensor_id}.angular_resolution_rad"],
        derived["angle_increment_rad"],
    )
    assert contract["minimum_lidar_ray_evidence"] == "full_ray"
    assert contract["degraded_lidar_ray_evidence"] == "full_ray"


def test_rviz_contains_the_five_scene_views_from_canonical_topics():
    loader = _load_config_module()
    config = loader.load_scene_config(CONFIG_DIR / "cave_full_ray_scene.yaml")
    topics = config["topics"]
    with (CONFIG_DIR / "cave_full_ray_scene.rviz").open("r", encoding="utf-8") as stream:
        rviz_config = yaml.safe_load(stream)

    cave = _display_by_class(rviz_config, "rviz_default_plugins/PointCloud2")
    path = _display_by_class(rviz_config, "rviz_default_plugins/Path")
    scan = _display_by_class(rviz_config, "rviz_default_plugins/LaserScan")
    tf_display = _display_by_class(rviz_config, "rviz_default_plugins/TF")
    octomap = _display_by_class(rviz_config, "octomap_rviz_plugins/OccupancyGrid")

    assert cave["Topic"]["Value"] == topics["cave_truth"]
    assert path["Topic"]["Value"] == topics["path"]
    assert scan["Topic"]["Value"] == topics["released_scan"]
    assert octomap["Topic"]["Value"] == topics["local_map_octomap"]
    assert tf_display["Enabled"] is True
    fixed_frame = rviz_config["Visualization Manager"]["Global Options"]["Fixed Frame"]
    assert fixed_frame == config["frames"]["map"]


def test_launch_loads_once_and_scopes_the_only_octomap_preload_to_rviz():
    launch_path = LAUNCH_DIR / "cave_full_ray_scene.launch.py"
    tree = ast.parse(launch_path.read_text(encoding="utf-8"), filename=str(launch_path))

    loader_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "load_scene_config"
    ]
    assert len(loader_calls) == 1

    preload_keywords = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        for keyword in node.keywords:
            if keyword.arg != "additional_env":
                continue
            assert isinstance(node.func, ast.Name) and node.func.id == "Node"
            keyword_map = {
                entry.arg: entry.value
                for entry in node.keywords
                if entry.arg is not None
            }
            assert ast.literal_eval(keyword_map["package"]) == "rviz2"
            assert ast.literal_eval(keyword_map["executable"]) == "rviz2"
            assert ast.literal_eval(keyword.value) == {"LD_PRELOAD": "liboctomap.so"}
            preload_keywords.append(keyword)

    assert len(preload_keywords) == 1
    assert "cave_full_ray_scene" not in (LAUNCH_DIR / "local_map_debug.launch.xml").read_text(
        encoding="utf-8"
    )
