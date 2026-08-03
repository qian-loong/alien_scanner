import struct
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import rclpy
from nav_msgs.msg import Odometry
from perception_interfaces.msg import HealthState, PoseEstimate
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


def generate_test_description():
    input_node = launch_ros.actions.Node(
        package="perception_input_node",
        executable="perception_input_node",
        name="perception_input_node",
        parameters=[
            {
                "sensor_ids": ["front", "top"],
                "requires_pose": True,
                "minimum_lidar_type": "3d",
                "minimum_lidar_count": 1,
                "degraded_lidar_type": "2d",
                "degraded_lidar_count": 1,
                "recovery_stability_samples": 3,
                "sensor_timeout_s": 0.5,
                "pose_timeout_s": 0.5,
                "health_period_s": 0.1,
                "expected_pose_frame": "odom",
                "minimum_pose_quality": 0.5,
                "sensor.front.type": "2d",
                "sensor.front.frame_id": "front_link",
                "sensor.front.topic": "fixture/scan/front",
                "sensor.top.type": "3d",
                "sensor.top.frame_id": "fixture_lidar_link",
                "sensor.top.topic": "fixture/points",
            }
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription([input_node, launch_testing.actions.ReadyToTest()]),
        {"input_node": input_node},
    )


class TestPerceptionInputHealthIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("perception_input_health_integration_test")
        cls.health_states = []
        cls.health_messages = []
        cls.reset_epochs = []
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        cls.scan_publisher = cls.node.create_publisher(
            LaserScan, "fixture/scan/front", sensor_qos
        )
        cls.cloud_publisher = cls.node.create_publisher(
            PointCloud2, "fixture/points", sensor_qos
        )
        cls.odom_publisher = cls.node.create_publisher(
            Odometry, "odom", sensor_qos
        )
        health_qos = QoSProfile(depth=10)
        cls.node.create_subscription(
            HealthState,
            "perception/health",
            cls._on_health,
            health_qos,
        )
        cls.node.create_subscription(
            PoseEstimate,
            "perception/pose",
            cls._on_pose,
            health_qos,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_health(cls, message):
        cls.health_states.append(message.state)
        cls.health_messages.append(message)

    @classmethod
    def _on_pose(cls, message):
        cls.reset_epochs.append(message.reset_epoch)

    def _spin(self, duration_seconds):
        deadline = time.monotonic() + duration_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _publish_scan(self):
        message = LaserScan()
        message.header.frame_id = "front_link"
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.angle_min = -1.57
        message.angle_max = 1.57
        message.angle_increment = 3.14 / 2.0
        message.range_min = 0.1
        message.range_max = 30.0
        message.ranges = [4.0, 4.2, 4.0]
        self.scan_publisher.publish(message)

    def _publish_cloud(self):
        message = PointCloud2()
        message.header.frame_id = "fixture_lidar_link"
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.height = 1
        message.width = 1
        message.is_bigendian = False
        message.is_dense = True
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]
        message.point_step = 16
        message.row_step = 16
        message.data = list(struct.pack("<ffff", 1.0, 2.0, 3.0, 1.0))
        self.cloud_publisher.publish(message)

    def _publish_pose(self, frame_id="odom", stamp_ns=None, covariance_diagonal=0.0):
        if stamp_ns is None:
            stamp_ns = self.node.get_clock().now().nanoseconds
        message = Odometry()
        message.header.frame_id = frame_id
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        message.pose.pose.orientation.w = 1.0
        for index in (0, 7, 14):
            message.pose.covariance[index] = covariance_diagonal
        self.odom_publisher.publish(message)

    def _publish_samples(
        self,
        include_cloud,
        include_front=True,
        include_pose=True,
        pose_frame="odom",
        pose_covariance=0.0,
    ):
        if include_front:
            self._publish_scan()
        if include_cloud:
            self._publish_cloud()
        if include_pose:
            self._publish_pose(
                frame_id=pose_frame,
                covariance_diagonal=pose_covariance,
            )
        self._spin(0.1)

    def _wait_for_state(self, state, start_index, timeout_seconds=3.0):
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            if state in self.health_states[start_index:]:
                return True
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return state in self.health_states[start_index:]

    def _wait_for_health_reason(self, reason, start_index, timeout_seconds=3.0):
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            if any(
                message.state == HealthState.STATE_UNAVAILABLE
                and reason in message.degradation_reason
                for message in self.health_messages[start_index:]
            ):
                return True
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return False

    def _reach_state(self, state, include_cloud, include_front=True, include_pose=True):
        start_index = len(self.health_states)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            self._publish_samples(include_cloud, include_front, include_pose)
            if state in self.health_states[start_index:]:
                return
        self.fail(
            "health state %s was not reached; observed %s"
            % (state, self.health_states[start_index:])
        )

    def test_health_transitions_pose_timeout_and_reset_epoch(self):
        self._reach_state(HealthState.STATE_DEGRADED, include_cloud=False)
        self._reach_state(HealthState.STATE_HEALTHY, include_cloud=True)

        start_index = len(self.health_states)
        for _ in range(8):
            self._publish_samples(include_cloud=False)
        self.assertTrue(
            self._wait_for_state(HealthState.STATE_DEGRADED, start_index),
            self.health_states[start_index:],
        )

        start_index = len(self.health_states)
        self._spin(0.9)
        self.assertTrue(
            self._wait_for_state(HealthState.STATE_UNAVAILABLE, start_index),
            self.health_states[start_index:],
        )

        self._reach_state(HealthState.STATE_DEGRADED, include_cloud=False)
        self._reach_state(HealthState.STATE_HEALTHY, include_cloud=True)

        health_start = len(self.health_messages)
        for _ in range(4):
            self._publish_samples(
                include_cloud=True,
                pose_covariance=1.0,
            )
        self.assertTrue(
            self._wait_for_health_reason("Pose quality below threshold", health_start),
            [message.degradation_reason for message in self.health_messages[health_start:]],
        )

        self._reach_state(HealthState.STATE_DEGRADED, include_cloud=False)
        self._reach_state(HealthState.STATE_HEALTHY, include_cloud=True)

        health_start = len(self.health_messages)
        for _ in range(4):
            self._publish_samples(include_cloud=True, pose_frame="map")
        self.assertTrue(
            self._wait_for_health_reason("Pose frame mismatch", health_start),
            [message.degradation_reason for message in self.health_messages[health_start:]],
        )

        base_stamp = self.node.get_clock().now().nanoseconds
        self._publish_pose("odom", base_stamp)
        self._spin(0.2)
        self._publish_pose("map", base_stamp + 1_000_000)
        self._spin(0.2)
        self._publish_pose("map", base_stamp)
        self._spin(0.3)
        self.assertGreaterEqual(max(self.reset_epochs), 2)
