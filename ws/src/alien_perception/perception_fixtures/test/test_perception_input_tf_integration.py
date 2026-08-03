import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from perception_interfaces.msg import HealthState, PoseEstimate
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage


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
                "requires_pose": True,
                "minimum_lidar_type": "2d",
                "minimum_lidar_count": 1,
                "degraded_lidar_type": "2d",
                "degraded_lidar_count": 1,
                "recovery_stability_samples": 1,
                "sensor_timeout_s": 0.5,
                "pose_timeout_s": 0.5,
                "health_period_s": 0.1,
                "pose_input_type": "tf",
                "pose_source_id": "tf_localization",
                "tf_topic": "fixture/tf",
                "tf_child_frame": "base_link",
                "expected_pose_frame": "map",
                "minimum_pose_quality": 0.9,
                "sensor.front.type": "2d",
                "sensor.front.frame_id": "front_link",
                "sensor.front.topic": "fixture/scan/front",
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


class TestPerceptionInputTfIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("perception_input_tf_integration_test")
        cls.poses = []
        cls.health_messages = []
        cls.tf_publisher = cls.node.create_publisher(
            TFMessage, "fixture/tf", QoSProfile(depth=100)
        )
        cls.node.create_subscription(PoseEstimate, "perception/pose", cls._on_pose, 10)
        cls.node.create_subscription(
            HealthState, "perception/health", cls._on_health, 10
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_pose(cls, message):
        cls.poses.append(message)

    @classmethod
    def _on_health(cls, message):
        cls.health_messages.append(message)

    def _publish_tf(self, parent_frame):
        transform = TransformStamped()
        transform.header.frame_id = parent_frame
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = 1.0
        transform.transform.rotation.w = 1.0
        self.tf_publisher.publish(TFMessage(transforms=[transform]))

    def _wait_while_publishing(self, predicate, parent_frame, timeout_seconds=5.0):
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            self._publish_tf(parent_frame)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def test_tf_pose_input_and_frame_gate(self):
        self.assertTrue(
            self._wait_while_publishing(
                lambda: any(pose.header.frame_id == "map" for pose in self.poses)
                and any(
                    health.state == HealthState.STATE_HEALTHY
                    for health in self.health_messages
                ),
                "map",
            )
        )

        valid_pose = next(pose for pose in self.poses if pose.header.frame_id == "map")
        self.assertEqual(valid_pose.source_id, "tf_localization")
        self.assertEqual(valid_pose.quality, 1.0)
        self.assertGreater(valid_pose.session_boot_time_ns, 0)

        pose_start = len(self.poses)
        health_start = len(self.health_messages)
        self.assertTrue(
            self._wait_while_publishing(
                lambda: any(
                    health.state == HealthState.STATE_UNAVAILABLE
                    and "Pose frame mismatch" in health.degradation_reason
                    for health in self.health_messages[health_start:]
                ),
                "odom",
            )
        )
        self.assertTrue(
            self._wait_while_publishing(
                lambda: any(
                    pose.reset_epoch >= 1 for pose in self.poses[pose_start:]
                ),
                "odom",
            )
        )
