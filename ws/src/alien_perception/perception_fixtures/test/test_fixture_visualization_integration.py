import math
import time
import unittest
from pathlib import Path

import launch
import launch.actions
import launch.launch_description_sources
import launch_testing
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


@pytest.mark.launch_test
def generate_test_description():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "fixture_visualization.launch.py"
    )
    visualization = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(launch_path)
        ),
        launch_arguments={"show_rviz": "false"}.items(),
    )
    return launch.LaunchDescription(
        [visualization, launch_testing.actions.ReadyToTest()]
    )


class TestFixtureVisualizationIntegration(unittest.TestCase):
    sensor_topics = (
        "/fixture/scan/flat",
        "/fixture/scan/tilted",
        "/fixture/points",
    )
    sensor_frames = (
        "fixture_scan_flat_link",
        "fixture_scan_tilted_link",
        "fixture_lidar_link",
    )
    publisher_nodes = {
        "fixture_scan_flat_publisher",
        "fixture_scan_tilted_publisher",
        "fixture_lidar_3d_publisher",
    }

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("fixture_visualization_integration_test")
        cls.static_transforms = {}

        static_tf_qos = QoSProfile(depth=10)
        static_tf_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        static_tf_qos.reliability = ReliabilityPolicy.RELIABLE
        cls.static_tf_subscription = cls.node.create_subscription(
            TFMessage,
            "/tf_static",
            cls._on_static_tf,
            static_tf_qos,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_subscription(cls.static_tf_subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_static_tf(cls, message):
        for transform in message.transforms:
            if transform.child_frame_id in cls.sensor_frames:
                cls.static_transforms[transform.child_frame_id] = transform

    def _wait_for_scene(self, timeout_seconds=8.0):
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            discovered_nodes = {
                name for name, _ in self.node.get_node_names_and_namespaces()
            }
            if (
                all(frame in self.static_transforms for frame in self.sensor_frames)
                and all(
                    self.node.get_publishers_info_by_topic(topic)
                    for topic in self.sensor_topics
                )
                and self.publisher_nodes <= discovered_nodes
            ):
                return True
        return False

    def test_tilted_scan_points_toward_positive_z(self):
        self.assertTrue(self._wait_for_scene())

        for frame in self.sensor_frames:
            self.assertEqual(
                self.static_transforms[frame].header.frame_id,
                "base_link",
            )

        transform = self.static_transforms["fixture_scan_tilted_link"].transform
        rotation = transform.rotation
        norm = math.sqrt(
            rotation.x * rotation.x
            + rotation.y * rotation.y
            + rotation.z * rotation.z
            + rotation.w * rotation.w
        )
        local_x_in_base_z = 2.0 * (
            rotation.x * rotation.z - rotation.w * rotation.y
        )

        self.assertAlmostEqual(norm, 1.0, places=6)
        self.assertAlmostEqual(rotation.y, -0.258819, places=5)
        self.assertAlmostEqual(rotation.w, 0.965926, places=5)
        self.assertAlmostEqual(local_x_in_base_z, 0.5, places=5)
        self.assertGreater(local_x_in_base_z, 0.0)
