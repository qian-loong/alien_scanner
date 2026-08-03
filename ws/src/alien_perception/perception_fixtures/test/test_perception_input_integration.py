import math
import re
import time
import unittest
from threading import Event

import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from perception_interfaces.msg import HealthState, LidarObservation
from rclpy.qos import QoSProfile, ReliabilityPolicy


@pytest.mark.launch_test
def generate_test_description():
    fixture = launch_ros.actions.Node(
        package="perception_fixtures",
        executable="perception_fixture_publisher",
        name="perception_fixture_publisher",
        parameters=[
            {
                "mode": "2d",
                "scan_frame": "front_link",
                "scan_topic_front": "fixture/scan/front",
                "publish_period_s": 0.05,
            }
        ],
        output="screen",
    )
    input_node = launch_ros.actions.Node(
        package="perception_input_node",
        executable="perception_input_node",
        name="perception_input_node",
        parameters=[
            {
                "sensor_ids": ["front"],
                "requires_pose": False,
                "minimum_lidar_type": "2d",
                "minimum_lidar_count": 1,
                "minimum_lidar_ray_evidence": "full_ray",
                "degraded_lidar_type": "2d",
                "degraded_lidar_count": 1,
                "degraded_lidar_ray_evidence": "hit_ray",
                "recovery_stability_samples": 3,
                "sensor.front.type": "2d",
                "sensor.front.frame_id": "front_link",
                "sensor.front.topic": "fixture/scan/front",
                "sensor.front.ray_evidence": "hit_ray",
                "sensor.front.fov_horizontal_min_rad": -math.pi / 2.0,
                "sensor.front.fov_horizontal_max_rad": math.pi / 2.0,
                "sensor.front.angular_resolution_rad": math.pi / 180.0,
            }
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription(
            [fixture, input_node, launch_testing.actions.ReadyToTest()]
        ),
        {"fixture": fixture, "input_node": input_node},
    )


class TestPerceptionInputIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("perception_input_integration_test")
        cls.observation_event = Event()
        cls.health_event = Event()
        cls.latest_observation = None
        cls.latest_health = None
        observation_qos = QoSProfile(depth=10)
        observation_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        cls.node.create_subscription(
            LidarObservation,
            "perception/observations",
            cls._on_observation,
            observation_qos,
        )
        cls.node.create_subscription(
            HealthState,
            "perception/health",
            cls._on_health,
            10,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_observation(cls, message):
        cls.latest_observation = message
        cls.observation_event.set()

    @classmethod
    def _on_health(cls, message):
        cls.latest_health = message
        cls.health_event.set()

    def _spin_until(self, event, timeout_seconds):
        deadline = time.monotonic() + timeout_seconds
        while not event.is_set() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return event.is_set()

    def test_observation_is_published_with_sensor_identity(self):
        self.assertTrue(self._spin_until(self.observation_event, 8.0))
        self.assertEqual(self.latest_observation.sensor_id, "front")
        self.assertEqual(
            self.latest_observation.data_type,
            LidarObservation.DATA_TYPE_SCAN_2D,
        )
        self.assertEqual(len(self.latest_observation.ranges), 181)
        self.assertEqual(self.latest_observation.header.frame_id, "front_link")
        self.assertEqual(
            self.latest_observation.ray_evidence,
            LidarObservation.RAY_EVIDENCE_HIT_RAY,
        )

    def test_health_reaches_degraded_after_recovery_window(self):
        self.assertTrue(self._spin_until(self.health_event, 8.0))
        deadline = time.monotonic() + 8.0
        while (
            self.latest_health.state != HealthState.STATE_DEGRADED
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertEqual(self.latest_health.state, HealthState.STATE_DEGRADED)
        self.assertEqual(self.latest_health.active_sensor_count, 1)
        self.assertTrue(self.latest_health.has_free_space_hit_rays)
        self.assertFalse(self.latest_health.has_full_no_return_rays)
        self.assertEqual(self.latest_health.producer_source_id, "perception_input")
        self.assertGreater(self.latest_health.producer_session_boot_time_ns, 0)
        self.assertEqual(self.latest_health.mapper_contract_schema_version, 1)
        self.assertIsNotNone(
            re.fullmatch(r"[0-9a-f]{64}", self.latest_health.mapper_contract_fingerprint)
        )
