import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray
import launch
import launch_ros.actions
import launch_testing
from perception_interfaces.msg import PoseEstimate
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


@pytest.mark.launch_test
def generate_test_description():
    relay = launch_ros.actions.Node(
        package="perception_fixtures",
        executable="pose_gated_laser_scan_relay",
        name="pose_gated_laser_scan_relay_test",
        output="screen",
        parameters=[
            {
                "raw_scan_topic": "/gate/raw",
                "released_scan_topic": "/gate/released",
                "pose_topic": "/gate/pose",
                "expected_pose_frame": "map",
                "expected_pose_source": "odom",
                "expected_clock_domain": "vehicle_steady_clock",
                "pose_lead_delay_s": 0.1,
                "odom_period_s": 0.05,
                "pending_timeout_s": 0.5,
                "max_pending_scans": 8,
            }
        ],
    )
    return launch.LaunchDescription(
        [relay, launch_testing.actions.ReadyToTest()]
    )


class TestPoseGatedLaserScanRelay(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("pose_gated_laser_scan_relay_integration_test")
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        cls.raw_publisher = cls.node.create_publisher(LaserScan, "/gate/raw", sensor_qos)
        cls.pose_publisher = cls.node.create_publisher(
            PoseEstimate, "/gate/pose", QoSProfile(depth=10)
        )
        cls.released = []
        cls.diagnostics = []
        cls.released_subscription = cls.node.create_subscription(
            LaserScan, "/gate/released", cls.released.append, sensor_qos
        )
        cls.diagnostic_subscription = cls.node.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            cls.diagnostics.append,
            QoSProfile(depth=20),
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_subscription(cls.released_subscription)
        cls.node.destroy_subscription(cls.diagnostic_subscription)
        cls.node.destroy_publisher(cls.raw_publisher)
        cls.node.destroy_publisher(cls.pose_publisher)
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def _wait_for_graph(self):
        ready = self._spin_until(
            lambda: self.node.count_subscribers("/gate/raw") == 1
            and self.node.count_subscribers("/gate/pose") == 1
            and self.node.count_publishers("/gate/released") == 1
        )
        if ready:
            settle_deadline = time.monotonic() + 0.25
            while time.monotonic() < settle_deadline:
                rclpy.spin_once(self.node, timeout_sec=0.02)
        return ready

    @staticmethod
    def _scan(stamp_sec):
        message = LaserScan()
        message.header.stamp.sec = stamp_sec
        message.header.frame_id = "scan_link"
        message.ranges = [1.0]
        return message

    @staticmethod
    def _pose(stamp_sec, session_boot_time_ns=1234, reset_epoch=0):
        message = PoseEstimate()
        message.header.stamp.sec = stamp_sec
        message.header.frame_id = "map"
        message.source_id = "odom"
        message.clock_domain = "vehicle_steady_clock"
        message.session_boot_time_ns = session_boot_time_ns
        message.session_random_suffix = 5
        message.reset_epoch = reset_epoch
        return message

    def test_01_scan_waits_for_matching_pose_and_lead_delay(self):
        self.assertTrue(self._wait_for_graph())
        before = len(self.released)
        self.raw_publisher.publish(self._scan(10))
        for _ in range(3):
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertEqual(len(self.released), before)

        pose_publish_time = time.monotonic()
        self.pose_publisher.publish(self._pose(10))
        self.assertTrue(self._spin_until(lambda: len(self.released) == before + 1))
        self.assertGreaterEqual(time.monotonic() - pose_publish_time, 0.09)
        self.assertEqual(self.released[0].header.stamp.sec, 10)

    def test_02_pose_watermark_may_arrive_before_raw_scan(self):
        before = len(self.released)
        self.pose_publisher.publish(self._pose(11))
        for _ in range(3):
            rclpy.spin_once(self.node, timeout_sec=0.02)
        scan_publish_time = time.monotonic()
        self.raw_publisher.publish(self._scan(11))
        self.assertTrue(self._spin_until(lambda: len(self.released) == before + 1))
        self.assertGreaterEqual(time.monotonic() - scan_publish_time, 0.02)
        self.assertEqual(self.released[-1].header.stamp.sec, 11)

    def test_03_stamp_rollback_is_diagnosed_and_not_released(self):
        before = len(self.released)
        self.raw_publisher.publish(self._scan(9))
        self.assertTrue(
            self._spin_until(
                lambda: any(
                    "non-increasing acquisition stamp" in status.message
                    for array in self.diagnostics
                    for status in array.status
                )
            )
        )
        self.assertEqual(len(self.released), before)

    def test_04_new_lineage_is_accepted_and_old_lineage_is_retired(self):
        before = len(self.released)
        self.pose_publisher.publish(self._pose(12, session_boot_time_ns=5678))
        for _ in range(3):
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.raw_publisher.publish(self._scan(12))
        self.assertTrue(self._spin_until(lambda: len(self.released) == before + 1))
        self.assertEqual(self.released[-1].header.stamp.sec, 12)

        self.pose_publisher.publish(self._pose(13, session_boot_time_ns=1234))
        self.assertTrue(
            self._spin_until(
                lambda: any(
                    "retired lineage" in status.message
                    for array in self.diagnostics
                    for status in array.status
                )
            )
        )
