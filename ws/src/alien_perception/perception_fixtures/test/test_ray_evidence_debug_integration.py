import copy
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
from geometry_msgs.msg import Point
from perception_interfaces.msg import LidarObservation
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


@pytest.mark.launch_test
def generate_test_description():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "ray_evidence_debug.launch.py"
    )
    debug_view = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(launch_path)
        ),
        launch_arguments={"show_rviz": "false", "beam_stride": "1"}.items(),
    )
    return launch.LaunchDescription(
        [debug_view, launch_testing.actions.ReadyToTest()]
    )


class TestRayEvidenceDebugIntegration(unittest.TestCase):
    marker_topic = "/perception/debug/ray_evidence"
    observation_topic = "/perception/debug/observations"

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("ray_evidence_debug_integration_test")
        cls.marker_batches = {}
        cls.observation_batches = {}
        cls.static_transforms = {}

        cls.marker_subscription = cls.node.create_subscription(
            MarkerArray,
            cls.marker_topic,
            cls._on_markers,
            QoSProfile(depth=20),
        )
        observation_qos = QoSProfile(depth=10)
        observation_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        cls.observation_publisher = cls.node.create_publisher(
            LidarObservation,
            cls.observation_topic,
            observation_qos,
        )
        cls.observation_subscription = cls.node.create_subscription(
            LidarObservation,
            cls.observation_topic,
            cls._on_observation,
            observation_qos,
        )
        static_tf_qos = QoSProfile(depth=10)
        static_tf_qos.reliability = ReliabilityPolicy.RELIABLE
        static_tf_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        cls.static_tf_subscription = cls.node.create_subscription(
            TFMessage,
            "/tf_static",
            cls._on_static_tf,
            static_tf_qos,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_subscription(cls.marker_subscription)
        cls.node.destroy_subscription(cls.observation_subscription)
        cls.node.destroy_subscription(cls.static_tf_subscription)
        cls.node.destroy_publisher(cls.observation_publisher)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_markers(cls, message):
        if message.markers:
            cls.marker_batches[message.markers[0].header.frame_id] = message

    @classmethod
    def _on_observation(cls, message):
        cls.observation_batches[message.sensor_id] = message

    @classmethod
    def _on_static_tf(cls, message):
        for transform in message.transforms:
            cls.static_transforms[transform.child_frame_id] = transform

    def _spin_until(self, predicate, timeout_seconds=10.0):
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def _publish_observation(self, message, repeat_count=3):
        for _ in range(repeat_count):
            self.observation_publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _publish_until(self, message, predicate, timeout_seconds=10.0):
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            self.observation_publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    @staticmethod
    def _markers_by_namespace(message):
        return {marker.ns: marker for marker in message.markers}

    @staticmethod
    def _rotate_vector(rotation, vector):
        x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        vx, vy, vz = vector
        return (
            (1.0 - 2.0 * (y * y + z * z)) * vx
            + 2.0 * (x * y - z * w) * vy
            + 2.0 * (x * z + y * w) * vz,
            2.0 * (x * y + z * w) * vx
            + (1.0 - 2.0 * (x * x + z * z)) * vy
            + 2.0 * (y * z - x * w) * vz,
            2.0 * (x * z - y * w) * vx
            + 2.0 * (y * z + x * w) * vy
            + (1.0 - 2.0 * (x * x + y * y)) * vz,
        )

    def test_tunnel_observation_and_marker_semantics(self):
        self.assertTrue(
            self._spin_until(
                lambda: "debug_scan_link" in self.marker_batches
                and "debug_scan" in self.observation_batches
            )
        )

        observation = self.observation_batches["debug_scan"]
        self.assertEqual(observation.data_type, LidarObservation.DATA_TYPE_SCAN_2D)
        self.assertEqual(
            observation.ray_evidence,
            LidarObservation.RAY_EVIDENCE_FULL_RAY,
        )
        self.assertEqual(len(observation.ranges), 360)
        self.assertAlmostEqual(observation.angle_min, -math.pi, places=6)
        self.assertAlmostEqual(
            observation.angle_increment,
            2.0 * math.pi / 360.0,
            places=7,
        )
        self.assertAlmostEqual(
            observation.angle_max,
            math.pi - 2.0 * math.pi / 360.0,
            places=6,
        )
        self.assertAlmostEqual(observation.range_min, 0.1, places=7)
        self.assertAlmostEqual(observation.range_max, 10.0, places=7)

        no_return_indices = [
            index
            for index, value in enumerate(observation.ranges)
            if math.isinf(value) and value > 0.0
        ]
        self.assertEqual(no_return_indices, list(range(255, 286)))
        invalid_indices = [
            index
            for index, value in enumerate(observation.ranges)
            if math.isnan(value)
            or (math.isinf(value) and value < 0.0)
            or (
                math.isfinite(value)
                and not observation.range_min
                <= value
                <= observation.range_max
            )
        ]
        self.assertEqual(invalid_indices, [44, 136, 180, 316])

        scan_batch = self.marker_batches["debug_scan_link"]
        scan = self._markers_by_namespace(scan_batch)
        self.assertEqual(
            set(scan),
            {
                "ray_evidence/hit_endpoints",
                "ray_evidence/hit_free",
                "ray_evidence/no_return_free",
                "ray_evidence/invalid",
            },
        )
        red_points = len(scan["ray_evidence/hit_endpoints"].points)
        green_points = len(scan["ray_evidence/hit_free"].points)
        cyan_points = len(scan["ray_evidence/no_return_free"].points)
        self.assertEqual(red_points, 325)
        self.assertGreater(red_points, 300)
        self.assertEqual(green_points, 2 * red_points)
        self.assertEqual(cyan_points, 2 * len(no_return_indices))
        self.assertEqual(len(scan["ray_evidence/invalid"].points), 8)

        expected_colors = {
            "ray_evidence/hit_endpoints": (1.0, 0.0, 0.0),
            "ray_evidence/hit_free": (0.0, 1.0, 0.0),
            "ray_evidence/no_return_free": (0.0, 1.0, 1.0),
            "ray_evidence/invalid": (0.6, 0.6, 0.6),
        }
        for namespace, marker in scan.items():
            self.assertEqual(marker.header.frame_id, "debug_scan_link")
            self.assertNotEqual(
                (marker.header.stamp.sec, marker.header.stamp.nanosec),
                (0, 0),
            )
            self.assertEqual(marker.action, Marker.ADD)
            for actual, expected in zip(
                (marker.color.r, marker.color.g, marker.color.b),
                expected_colors[namespace],
            ):
                self.assertAlmostEqual(actual, expected, places=6)
            self.assertEqual(marker.lifetime.sec, 0)
            self.assertEqual(marker.lifetime.nanosec, 500_000_000)
        self.assertEqual(scan["ray_evidence/hit_endpoints"].type, Marker.POINTS)
        self.assertEqual(scan["ray_evidence/hit_free"].type, Marker.LINE_LIST)
        self.assertEqual(
            scan["ray_evidence/no_return_free"].type,
            Marker.LINE_LIST,
        )
        self.assertEqual(scan["ray_evidence/invalid"].type, Marker.LINE_LIST)

    def test_cloud_hit_only_is_injected_on_formal_observation_topic(self):
        self.assertTrue(
            self._spin_until(
                lambda: self.observation_publisher.get_subscription_count() > 0
            )
        )

        cloud_observation = LidarObservation()
        cloud_observation.header.frame_id = "direct_cloud_link"
        cloud_observation.header.stamp = self.node.get_clock().now().to_msg()
        cloud_observation.sensor_id = "direct_debug_cloud"
        cloud_observation.session_boot_time_ns = 1
        cloud_observation.session_random_suffix = 2
        cloud_observation.clock_domain = "test"
        cloud_observation.data_type = LidarObservation.DATA_TYPE_CLOUD_3D
        cloud_observation.ray_evidence = LidarObservation.RAY_EVIDENCE_HIT_ONLY
        cloud_observation.points = [Point(x=1.0, y=2.0, z=3.0)]
        cloud_observation.point_intensities = [4.0]
        self.assertTrue(
            self._publish_until(
                cloud_observation,
                lambda: "direct_cloud_link" in self.marker_batches,
            )
        )

        cloud = self._markers_by_namespace(self.marker_batches["direct_cloud_link"])
        self.assertEqual(len(cloud["ray_evidence/hit_endpoints"].points), 1)
        self.assertEqual(len(cloud["ray_evidence/hit_free"].points), 0)
        self.assertEqual(len(cloud["ray_evidence/no_return_free"].points), 0)
        self.assertEqual(len(cloud["ray_evidence/invalid"].points), 0)

    def test_static_transform_maps_scan_basis_into_tunnel_cross_section(self):
        self.assertTrue(
            self._spin_until(
                lambda: "debug_scan_link" in self.static_transforms
            )
        )
        transform = self.static_transforms["debug_scan_link"]
        self.assertEqual(transform.header.frame_id, "map")
        self.assertEqual(transform.child_frame_id, "debug_scan_link")
        self.assertAlmostEqual(transform.transform.translation.x, 0.0, places=9)
        self.assertAlmostEqual(transform.transform.translation.y, 0.0, places=9)
        self.assertAlmostEqual(transform.transform.translation.z, 0.0, places=9)

        expected_basis_mappings = (
            ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
            ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0)),
            ((0.0, 1.0, 0.0), (0.0, 1.0, 0.0)),
        )
        for local_basis, expected_map_basis in expected_basis_mappings:
            actual = self._rotate_vector(
                transform.transform.rotation,
                local_basis,
            )
            for component, expected in zip(actual, expected_map_basis):
                self.assertAlmostEqual(component, expected, places=6)

    def test_invalid_batches_fail_closed(self):
        self.assertTrue(
            self._spin_until(
                lambda: self.observation_publisher.get_subscription_count() > 0
            )
        )

        valid = LidarObservation()
        valid.header.stamp = self.node.get_clock().now().to_msg()
        valid.sensor_id = "malformed_debug_sensor"
        valid.session_boot_time_ns = 1
        valid.session_random_suffix = 2
        valid.clock_domain = "test"
        valid.data_type = LidarObservation.DATA_TYPE_SCAN_2D
        valid.ray_evidence = LidarObservation.RAY_EVIDENCE_FULL_RAY
        valid.angle_min = -1.0
        valid.angle_max = 1.0
        valid.angle_increment = 1.0
        valid.range_min = 0.1
        valid.range_max = 10.0
        valid.ranges = [1.0, 2.0, 3.0]

        malformed = []
        unknown_three = copy.deepcopy(valid)
        unknown_three.header.frame_id = "invalid_ray_evidence_3"
        unknown_three.ray_evidence = 3
        malformed.append(unknown_three)

        unknown_255 = copy.deepcopy(valid)
        unknown_255.header.frame_id = "invalid_ray_evidence_255"
        unknown_255.ray_evidence = 255
        malformed.append(unknown_255)

        unknown_type = copy.deepcopy(valid)
        unknown_type.header.frame_id = "invalid_data_type"
        unknown_type.data_type = 255
        malformed.append(unknown_type)

        mixed_payload = copy.deepcopy(valid)
        mixed_payload.header.frame_id = "invalid_mixed_payload"
        mixed_payload.points = [Point(x=1.0, y=2.0, z=3.0)]
        malformed.append(mixed_payload)

        invalid_metadata = copy.deepcopy(valid)
        invalid_metadata.header.frame_id = "invalid_scan_metadata"
        invalid_metadata.angle_increment = 0.0
        malformed.append(invalid_metadata)

        empty_clock_domain = copy.deepcopy(valid)
        empty_clock_domain.header.frame_id = "invalid_empty_clock_domain"
        empty_clock_domain.clock_domain = ""
        malformed.append(empty_clock_domain)

        zero_session = copy.deepcopy(valid)
        zero_session.header.frame_id = "invalid_zero_session"
        zero_session.session_boot_time_ns = 0
        malformed.append(zero_session)

        cloud_with_scan_metadata = copy.deepcopy(valid)
        cloud_with_scan_metadata.header.frame_id = "invalid_cloud_scan_metadata"
        cloud_with_scan_metadata.data_type = LidarObservation.DATA_TYPE_CLOUD_3D
        cloud_with_scan_metadata.ray_evidence = (
            LidarObservation.RAY_EVIDENCE_HIT_ONLY
        )
        cloud_with_scan_metadata.ranges = []
        cloud_with_scan_metadata.points = [Point(x=1.0, y=2.0, z=3.0)]
        malformed.append(cloud_with_scan_metadata)

        cloud_with_high_capability = copy.deepcopy(cloud_with_scan_metadata)
        cloud_with_high_capability.header.frame_id = "invalid_cloud_full_ray"
        cloud_with_high_capability.angle_min = 0.0
        cloud_with_high_capability.angle_max = 0.0
        cloud_with_high_capability.angle_increment = 0.0
        cloud_with_high_capability.range_min = 0.0
        cloud_with_high_capability.range_max = 0.0
        cloud_with_high_capability.ray_evidence = (
            LidarObservation.RAY_EVIDENCE_FULL_RAY
        )
        malformed.append(cloud_with_high_capability)

        for _ in range(3):
            for message in malformed:
                self.observation_publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self._spin_until(lambda: False, timeout_seconds=0.8)
        for message in malformed:
            self.assertNotIn(message.header.frame_id, self.marker_batches)

    def test_marker_identity_is_stable_and_empty_adds_clear_old_categories(self):
        self.assertTrue(
            self._spin_until(
                lambda: self.observation_publisher.get_subscription_count() > 0
            )
        )

        full_ray = LidarObservation()
        full_ray.header.frame_id = "marker_update_link"
        full_ray.header.stamp.sec = 100
        full_ray.sensor_id = "marker_update_sensor"
        full_ray.session_boot_time_ns = 1
        full_ray.session_random_suffix = 2
        full_ray.clock_domain = "test"
        full_ray.data_type = LidarObservation.DATA_TYPE_SCAN_2D
        full_ray.ray_evidence = LidarObservation.RAY_EVIDENCE_FULL_RAY
        full_ray.angle_min = 0.0
        full_ray.angle_max = 2.0
        full_ray.angle_increment = 1.0
        full_ray.range_min = 0.1
        full_ray.range_max = 10.0
        full_ray.ranges = [1.0, float("inf"), float("nan")]

        self._publish_observation(full_ray)
        self.assertTrue(
            self._spin_until(lambda: "marker_update_link" in self.marker_batches)
        )
        first = self._markers_by_namespace(
            self.marker_batches["marker_update_link"]
        )
        first_ids = {namespace: marker.id for namespace, marker in first.items()}
        self.assertEqual(len(first["ray_evidence/hit_free"].points), 2)
        self.assertEqual(len(first["ray_evidence/no_return_free"].points), 2)

        hit_only = copy.deepcopy(full_ray)
        hit_only.header.stamp.sec = 101
        hit_only.ray_evidence = LidarObservation.RAY_EVIDENCE_HIT_ONLY
        self._publish_observation(hit_only)
        self.assertTrue(
            self._spin_until(
                lambda: self.marker_batches["marker_update_link"]
                .markers[0]
                .header.stamp.sec
                == 101
            )
        )
        second = self._markers_by_namespace(
            self.marker_batches["marker_update_link"]
        )
        self.assertEqual(
            {namespace: marker.id for namespace, marker in second.items()},
            first_ids,
        )
        self.assertEqual(len(second["ray_evidence/hit_endpoints"].points), 1)
        self.assertEqual(len(second["ray_evidence/hit_free"].points), 0)
        self.assertEqual(len(second["ray_evidence/no_return_free"].points), 0)
        self.assertEqual(len(second["ray_evidence/invalid"].points), 2)
        for marker in second.values():
            self.assertEqual(marker.action, Marker.ADD)
            self.assertEqual(marker.lifetime.nanosec, 500_000_000)

    def test_legacy_scan_returns_is_absent_from_ros_graph(self):
        self.assertTrue(
            self._spin_until(
                lambda: self.node.get_publishers_info_by_topic(self.marker_topic)
            )
        )
        legacy_topic = "/drone_0/scan_returns"
        topic_names = {
            name for name, _ in self.node.get_topic_names_and_types()
        }
        self.assertNotIn(legacy_topic, topic_names)
        self.assertEqual(
            self.node.get_publishers_info_by_topic(legacy_topic), []
        )
        self.assertEqual(
            self.node.get_subscriptions_info_by_topic(legacy_topic), []
        )
        forbidden_topic_tokens = ("scan_returns", "octomap", "occupancy")
        for topic_name in topic_names:
            lowered = topic_name.lower()
            self.assertFalse(
                any(token in lowered for token in forbidden_topic_tokens),
                msg=f"forbidden debug topic present: {topic_name}",
            )

        forbidden_node_tokens = ("fake_lidar", "octomap_builder")
        for node_name in self.node.get_node_names():
            lowered = node_name.lower()
            self.assertFalse(
                any(token in lowered for token in forbidden_node_tokens),
                msg=f"forbidden debug node present: {node_name}",
            )
        self.assertNotIn("/fixture/debug/points", topic_names)
        self.assertNotIn("debug_cloud_link", self.static_transforms)
