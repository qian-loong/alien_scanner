import math
import time
import unittest
from threading import Event

import launch
import launch_ros.actions
import launch_testing
import rclpy
from perception_interfaces.msg import LidarObservation
from rclpy.qos import QoSProfile, ReliabilityPolicy


def generate_test_description():
    fixture = launch_ros.actions.Node(
        package="perception_fixtures",
        executable="perception_fixture_publisher",
        name="perception_fixture_publisher",
        parameters=[
            {
                "mode": "mixed",
                "scan_frame": "fixture_scan_link",
                "cloud_frame": "fixture_lidar_link",
                "scan_topic_front": "fixture/scan/front",
                "scan_topic_rear": "fixture/scan/rear",
                "cloud_topic": "fixture/points",
                "cloud_azimuth_sample_count": 4,
                "elevation_angles_rad": [-0.2, 0.0, 0.2],
                "cloud_range_m": 5.0,
                "seed": 17,
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
                "sensor_ids": ["front", "rear", "top"],
                "requires_pose": False,
                "minimum_lidar_type": "3d",
                "minimum_lidar_count": 1,
                "degraded_lidar_type": "2d",
                "degraded_lidar_count": 1,
                "sensor.front.type": "2d",
                "sensor.front.frame_id": "fixture_scan_link",
                "sensor.front.topic": "fixture/scan/front",
                "sensor.rear.type": "2d",
                "sensor.rear.frame_id": "fixture_scan_link",
                "sensor.rear.topic": "fixture/scan/rear",
                "sensor.top.type": "3d",
                "sensor.top.frame_id": "fixture_lidar_link",
                "sensor.top.topic": "fixture/points",
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


class TestPerceptionInputMixedIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("perception_input_mixed_integration_test")
        cls.observation_event = Event()
        cls.observations = {}
        observation_qos = QoSProfile(depth=10)
        observation_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        cls.node.create_subscription(
            LidarObservation,
            "perception/observations",
            cls._on_observation,
            observation_qos,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_observation(cls, message):
        cls.observations[message.sensor_id] = message
        if {"front", "rear", "top"}.issubset(cls.observations):
            cls.observation_event.set()

    def test_mixed_sensor_identity_and_payloads_are_preserved(self):
        deadline = time.monotonic() + 8.0
        while not self.observation_event.is_set() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(self.observation_event.is_set())
        self.assertEqual(set(self.observations), {"front", "rear", "top"})
        for sensor_id in ("front", "rear"):
            observation = self.observations[sensor_id]
            self.assertEqual(
                observation.data_type,
                LidarObservation.DATA_TYPE_SCAN_2D,
            )
            self.assertEqual(observation.header.frame_id, "fixture_scan_link")
            self.assertEqual(len(observation.ranges), 181)
            self.assertAlmostEqual(observation.angle_min, -math.pi / 2.0, places=6)
            self.assertAlmostEqual(observation.angle_max, math.pi / 2.0, places=6)
            self.assertAlmostEqual(
                observation.angle_increment, math.pi / 180.0, places=7
            )
            zero_index = round(
                -observation.angle_min / observation.angle_increment
            )
            self.assertEqual(zero_index, 90)
            self.assertAlmostEqual(
                observation.angle_min + zero_index * observation.angle_increment,
                0.0,
                delta=1.0e-7,
            )
            self.assertGreater(observation.ranges[zero_index], 0.0)

        cloud = self.observations["top"]
        self.assertEqual(cloud.data_type, LidarObservation.DATA_TYPE_CLOUD_3D)
        self.assertEqual(cloud.header.frame_id, "fixture_lidar_link")
        self.assertEqual(len(cloud.points), 12)
        self.assertEqual(len(cloud.point_intensities), 12)

        expected_points = [
            (5.145349534, 0.0, -1.043013987, 17.0),
            (0.0, 4.655316245, -0.943679321, 18.0),
            (-5.145349534, 0.0, -1.043013987, 19.0),
            (0.0, -4.655316245, -0.943679321, 20.0),
            (5.25, 0.0, 0.0, 21.0),
            (0.0, 4.75, 0.0, 22.0),
            (-5.25, 0.0, 0.0, 23.0),
            (0.0, -4.75, 0.0, 24.0),
            (5.145349534, 0.0, 1.043013987, 25.0),
            (0.0, 4.655316245, 0.943679321, 26.0),
            (-5.145349534, 0.0, 1.043013987, 27.0),
            (0.0, -4.655316245, 0.943679321, 28.0),
        ]
        for point, expected in zip(cloud.points, expected_points):
            self.assertAlmostEqual(point.x, expected[0], places=5)
            self.assertAlmostEqual(point.y, expected[1], places=5)
            self.assertAlmostEqual(point.z, expected[2], places=5)
        for intensity, expected in zip(cloud.point_intensities, expected_points):
            self.assertAlmostEqual(intensity, expected[3], places=5)
