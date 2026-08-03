import importlib.util
import os
from pathlib import Path
import sys
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_testing
from nav_msgs.msg import Path as PathMessage
from octomap_msgs.msg import Octomap
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage


TEST_DOMAIN_ID = str(220 + (os.getpid() % 10))
RVIZ_NODE_NAME = "cave_full_ray_rviz_smoke"
os.environ["ROS_DOMAIN_ID"] = TEST_DOMAIN_ID


def _package_share():
    return Path(get_package_share_directory("perception_local_map"))


def _load_scene_config():
    module_path = _package_share() / "launch" / "cave_full_ray_scene_config.py"
    spec = importlib.util.spec_from_file_location("cave_full_ray_scene_config", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module.load_scene_config(
        _package_share() / "config" / "cave_full_ray_scene.yaml"
    )


@pytest.mark.launch_test
def generate_test_description():
    rviz = launch.actions.ExecuteProcess(
        cmd=[
            sys.executable,
            str(Path(__file__).resolve().parent / "run_managed_rviz.py"),
            "rviz2",
            "-d",
            str(_package_share() / "config" / "cave_full_ray_scene.rviz"),
            "--ros-args",
            "-r",
            f"__node:={RVIZ_NODE_NAME}",
        ],
        name="managed_cave_full_ray_rviz_smoke",
        output="screen",
        additional_env={
            "DISPLAY": "host.docker.internal:0.0",
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "LD_PRELOAD": "liboctomap.so",
        },
    )
    return (
        launch.LaunchDescription(
            [
                launch.actions.SetEnvironmentVariable("ROS_DOMAIN_ID", TEST_DOMAIN_ID),
                rviz,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"rviz": rviz},
    )


class TestCaveFullRaySceneRvizSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = _load_scene_config()
        rclpy.init()
        cls.node = rclpy.create_node("cave_full_ray_rviz_smoke_test")
        topics = cls.config["topics"]
        reliable_transient = QoSProfile(depth=1)
        reliable_transient.reliability = ReliabilityPolicy.RELIABLE
        reliable_transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        tf_qos = QoSProfile(depth=100)
        tf_qos.reliability = ReliabilityPolicy.RELIABLE
        tf_qos.durability = DurabilityPolicy.VOLATILE
        cls.publishers = [
            cls.node.create_publisher(
                PointCloud2, topics["cave_truth"], reliable_transient
            ),
            cls.node.create_publisher(
                PathMessage, topics["path"], reliable_transient
            ),
            cls.node.create_publisher(
                LaserScan, topics["released_scan"], sensor_qos
            ),
            cls.node.create_publisher(
                Octomap, topics["local_map_octomap"], reliable_transient
            ),
            cls.node.create_publisher(TFMessage, "/tf", tf_qos),
            cls.node.create_publisher(
                TFMessage, "/tf_static", reliable_transient
            ),
        ]

    @classmethod
    def tearDownClass(cls):
        for publisher in cls.publishers:
            cls.node.destroy_publisher(publisher)
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout=12.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            value = predicate()
            if value:
                return value
        return None

    @staticmethod
    def _rviz_pid():
        needle = f"__node:={RVIZ_NODE_NAME}"
        for process_dir in Path("/proc").iterdir():
            if not process_dir.name.isdigit():
                continue
            try:
                command = (process_dir / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                    errors="replace"
                )
                executable = (process_dir / "exe").resolve().name
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
            if executable == "rviz2" and needle in command:
                return int(process_dir.name)
        return None

    def _rviz_subscribes(self, topic):
        return any(
            endpoint.node_name == RVIZ_NODE_NAME
            or (
                topic in ("/tf", "/tf_static")
                and endpoint.node_name.startswith("transform_listener_impl_")
            )
            for endpoint in self.node.get_subscriptions_info_by_topic(topic)
        )

    def test_real_process_loads_plugin_and_subscribes_configured_topics(
        self, proc_output, rviz
    ):
        proc_output.assertWaitFor("OpenGl version", process=rviz, timeout=12)
        pid = self._spin_until(self._rviz_pid)
        self.assertIsNotNone(pid, "could not resolve the live RViz process PID")

        plugin_loaded = self._spin_until(
            lambda: "liboctomap_rviz_plugins.so"
            in Path(f"/proc/{pid}/maps").read_text(encoding="utf-8")
        )
        self.assertTrue(plugin_loaded, "OctoMap RViz plugin is absent from process maps")
        environment = Path(f"/proc/{pid}/environ").read_bytes().split(b"\0")
        self.assertIn(b"LD_PRELOAD=liboctomap.so", environment)

        topics = self.config["topics"]
        expected_topics = {
            topics["cave_truth"],
            topics["path"],
            topics["released_scan"],
            topics["local_map_octomap"],
            "/tf",
            "/tf_static",
        }

        def missing_topics():
            return sorted(
                topic for topic in expected_topics if not self._rviz_subscribes(topic)
            )
        self.assertTrue(
            self._spin_until(
                lambda: not missing_topics()
            ),
            f"RViz did not establish configured subscriptions: {missing_topics()}",
        )
        scan_subscriptions = [
            endpoint
            for endpoint in self.node.get_subscriptions_info_by_topic(
                topics["released_scan"]
            )
            if endpoint.node_name == RVIZ_NODE_NAME
        ]
        self.assertTrue(
            any(
                endpoint.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
                and endpoint.qos_profile.durability == DurabilityPolicy.VOLATILE
                for endpoint in scan_subscriptions
            ),
            "RViz has no SensorDataQoS-compatible released LaserScan subscription",
        )

        output = b"".join(event.text for event in proc_output[rviz]).decode(
            errors="replace"
        )
        self.assertNotIn("undefined symbol", output.lower())


@launch_testing.post_shutdown_test()
class TestCaveFullRaySceneRvizExit(unittest.TestCase):
    def test_launch_completed_without_plugin_crash(self, proc_info, rviz):
        launch_testing.asserts.assertExitCodes(proc_info, process=rviz)
