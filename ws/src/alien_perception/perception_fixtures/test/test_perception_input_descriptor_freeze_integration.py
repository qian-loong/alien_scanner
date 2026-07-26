"""Descriptor inventory freeze integration for perception_input_node.

Covers the runtime contract after the first valid observation freezes the
vehicle-session inventory:

1. session identity stays stable for the rest of the process lifetime
2. matching descriptor / frame_id may reconnect after a publish gap
3. frame_id that no longer matches the frozen descriptor is rejected and
   does not produce observations
"""

from __future__ import annotations

import time
import unittest
from threading import Event

import launch
import launch_ros.actions
import launch_testing
import rclpy
from perception_interfaces.msg import HealthState, LidarObservation
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def generate_test_description():
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
                "minimum_lidar_ray_evidence": "hit_ray",
                "degraded_lidar_type": "2d",
                "degraded_lidar_count": 1,
                "degraded_lidar_ray_evidence": "hit_ray",
                "recovery_stability_samples": 1,
                "sensor_timeout_s": 0.4,
                "health_period_s": 0.1,
                "sensor.front.type": "2d",
                "sensor.front.frame_id": "front_link",
                "sensor.front.topic": "fixture/scan/front",
                "sensor.front.range_min_m": 0.1,
                "sensor.front.range_max_m": 30.0,
            }
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription([input_node, launch_testing.actions.ReadyToTest()]),
        {"input_node": input_node},
    )


class TestPerceptionInputDescriptorFreezeIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("perception_input_descriptor_freeze_integration_test")
        cls.observations = []
        cls.observation_event = Event()
        cls.health_event = Event()
        cls.latest_health = None
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        cls.scan_publisher = cls.node.create_publisher(
            LaserScan, "fixture/scan/front", sensor_qos
        )
        observation_qos = QoSProfile(depth=20)
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
        cls.observations.append(message)
        cls.observation_event.set()

    @classmethod
    def _on_health(cls, message):
        cls.latest_health = message
        if message.active_sensor_count == 1:
            cls.health_event.set()

    def _spin(self, duration_seconds):
        deadline = time.monotonic() + duration_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _publish_scan(self, frame_id="front_link"):
        message = LaserScan()
        message.header.frame_id = frame_id
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.angle_min = -1.57
        message.angle_max = 1.57
        message.angle_increment = 3.14 / 2.0
        message.range_min = 0.1
        message.range_max = 30.0
        message.ranges = [4.0, 4.2, 4.0]
        message.intensities = [1.0, 1.0, 1.0]
        self.scan_publisher.publish(message)

    def _wait_for_observation_count(self, minimum_count, timeout_seconds=5.0):
        deadline = time.monotonic() + timeout_seconds
        while len(self.observations) < minimum_count and time.monotonic() < deadline:
            self._publish_scan()
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return len(self.observations) >= minimum_count

    def _spin_until(self, event, timeout_seconds):
        deadline = time.monotonic() + timeout_seconds
        while not event.is_set() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return event.is_set()

    def _session_key(self, message):
        return (message.session_boot_time_ns, message.session_random_suffix)

    def test_freeze_rejects_frame_mismatch_and_allows_matching_reconnect(self):
        # 1) First valid observation freezes the inventory and establishes session.
        self.assertTrue(
            self._wait_for_observation_count(1),
            "expected at least one observation after matching descriptor input",
        )
        baseline_count = len(self.observations)
        baseline_session = self._session_key(self.observations[0])
        self.assertEqual(self.observations[0].sensor_id, "front")
        self.assertEqual(self.observations[0].header.frame_id, "front_link")
        self.assertEqual(
            self.observations[0].data_type,
            LidarObservation.DATA_TYPE_SCAN_2D,
        )
        self.assertEqual(
            self.observations[0].ray_evidence,
            LidarObservation.RAY_EVIDENCE_HIT_ONLY,
        )
        self.assertTrue(
            self._spin_until(self.health_event, 5.0),
            "expected health after the default hit_only sensor became active",
        )
        self.assertEqual(
            self.latest_health.state,
            HealthState.STATE_UNAVAILABLE,
            "hit_only must satisfy neither the hit_ray minimum nor degraded contract",
        )
        self.assertFalse(self.latest_health.has_free_space_hit_rays)
        self.assertFalse(self.latest_health.has_full_no_return_rays)

        # Keep publishing matching scans so freeze is durable and session stays stable.
        for _ in range(5):
            self._publish_scan("front_link")
            self._spin(0.05)
        self.assertGreater(len(self.observations), baseline_count)
        for observation in self.observations:
            self.assertEqual(self._session_key(observation), baseline_session)
            self.assertEqual(observation.sensor_id, "front")
            self.assertEqual(observation.header.frame_id, "front_link")

        # 2) Changed frame_id is a descriptor semantic break: reject, no new obs.
        mismatched_start = len(self.observations)
        for _ in range(8):
            self._publish_scan("side_link")
            self._spin(0.05)
        mismatched = [
            observation
            for observation in self.observations[mismatched_start:]
            if observation.header.frame_id == "side_link"
        ]
        self.assertEqual(
            mismatched,
            [],
            "frozen descriptor must reject frame_id changes without publishing",
        )
        # Even if late matching deliveries arrive, none may carry the bad frame.
        for observation in self.observations[mismatched_start:]:
            self.assertNotEqual(observation.header.frame_id, "side_link")
            self.assertEqual(self._session_key(observation), baseline_session)

        # 3) Matching reconnect after a quiet gap keeps the same vehicle session.
        quiet_count = len(self.observations)
        self._spin(0.6)  # exceed sensor_timeout_s so health would see a gap
        self.assertEqual(
            len(self.observations),
            quiet_count,
            "no observations should appear without matching publisher traffic",
        )

        reconnect_start = len(self.observations)
        self.assertTrue(
            self._wait_for_observation_count(reconnect_start + 1, timeout_seconds=5.0),
            "matching descriptor must reconnect after freeze",
        )
        for observation in self.observations[reconnect_start:]:
            self.assertEqual(self._session_key(observation), baseline_session)
            self.assertEqual(observation.sensor_id, "front")
            self.assertEqual(observation.header.frame_id, "front_link")
