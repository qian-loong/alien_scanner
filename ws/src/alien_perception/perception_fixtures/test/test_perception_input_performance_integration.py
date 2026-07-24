import statistics
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from perception_interfaces.msg import LidarObservation
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


@pytest.mark.launch_test
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
                "degraded_lidar_type": "2d",
                "degraded_lidar_count": 1,
                "recovery_stability_samples": 1,
                "health_period_s": 0.1,
                "sensor.front.type": "2d",
                "sensor.front.frame_id": "front_link",
                "sensor.front.topic": "fixture/scan/front",
            }
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription([input_node, launch_testing.actions.ReadyToTest()]),
        {"input_node": input_node},
    )


class TestPerceptionInputPerformanceIntegration(unittest.TestCase):
    SAMPLE_COUNT = 120

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("perception_input_performance_test")
        cls.pending = {}
        cls.latencies_ms = []
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        cls.scan_publisher = cls.node.create_publisher(
            LaserScan, "fixture/scan/front", sensor_qos
        )
        cls.node.create_subscription(
            LidarObservation,
            "perception/observations",
            cls._on_observation,
            sensor_qos,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def _stamp_key(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    @classmethod
    def _on_observation(cls, message):
        key = cls._stamp_key(message.header.stamp)
        start_time = cls.pending.pop(key, None)
        if start_time is not None:
            cls.latencies_ms.append((time.perf_counter() - start_time) * 1000.0)

    def _make_scan(self, stamp):
        message = LaserScan()
        message.header.frame_id = "front_link"
        message.header.stamp = stamp
        message.angle_min = -3.14
        message.angle_max = 3.14
        message.angle_increment = 6.28 / 180.0
        message.range_min = 0.1
        message.range_max = 30.0
        message.ranges = [5.0] * 181
        message.intensities = [1.0] * 181
        return message

    def _wait_for_receipt(self, expected_count, message, timeout_seconds):
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            self.scan_publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if len(self.latencies_ms) >= expected_count:
                return True
        return len(self.latencies_ms) >= expected_count

    def test_reports_120_frame_publish_pipeline_baseline(self):
        discovery_deadline = time.monotonic() + 5.0
        while (
            self.scan_publisher.get_subscription_count() == 0
            and time.monotonic() < discovery_deadline
        ):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertGreater(self.scan_publisher.get_subscription_count(), 0)

        for index in range(self.SAMPLE_COUNT):
            stamp = self.node.get_clock().now().to_msg()
            key = self._stamp_key(stamp)
            self.pending[key] = time.perf_counter()
            message = self._make_scan(stamp)
            self.assertTrue(
                self._wait_for_receipt(index + 1, message, 1.0),
                "observation %d/%d was not received" % (index + 1, self.SAMPLE_COUNT),
            )

        sorted_latencies = sorted(self.latencies_ms)
        p95_index = min(
            len(sorted_latencies) - 1,
            int(0.95 * len(sorted_latencies)),
        )
        average_ms = statistics.fmean(sorted_latencies)
        p95_ms = sorted_latencies[p95_index]
        maximum_ms = sorted_latencies[-1]
        print(
            "[PERF] ros_publish_pipeline samples=%d lost=0 average_ms=%.3f "
            "p95_ms=%.3f max_ms=%.3f"
            % (self.SAMPLE_COUNT, average_ms, p95_ms, maximum_ms)
        )

        self.assertEqual(len(self.latencies_ms), self.SAMPLE_COUNT)
        self.assertEqual(self.pending, {})
        self.assertLess(p95_ms, 500.0)
