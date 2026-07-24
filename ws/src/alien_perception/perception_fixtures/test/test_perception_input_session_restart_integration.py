import signal
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from launch.events import matches_action
from launch.events.process import SignalProcess
from perception_interfaces.msg import LidarObservation
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
                "degraded_lidar_type": "2d",
                "degraded_lidar_count": 1,
                "sensor.front.type": "2d",
                "sensor.front.frame_id": "front_link",
                "sensor.front.topic": "fixture/scan/front",
            }
        ],
        output="screen",
        respawn=True,
        respawn_delay=0.2,
    )
    return (
        launch.LaunchDescription(
            [fixture, input_node, launch_testing.actions.ReadyToTest()]
        ),
        {"fixture": fixture, "input_node": input_node},
    )


class TestPerceptionInputSessionRestartIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("perception_input_session_restart_test")
        cls.sessions = []
        observation_qos = QoSProfile(depth=100)
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
        cls.sessions.append(
            (message.session_boot_time_ns, message.session_random_suffix)
        )

    def _wait_for_distinct_sessions(self, count, timeout_seconds):
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if len(set(self.sessions)) >= count:
                return True
        return len(set(self.sessions)) >= count

    def test_restart_creates_new_session(self, launch_service, input_node):
        self.assertTrue(self._wait_for_distinct_sessions(1, 8.0))
        first_session = self.sessions[-1]
        self.assertGreater(first_session[0], 0)

        launch_service.emit_event(
            SignalProcess(
                signal_number=signal.SIGKILL,
                process_matcher=matches_action(input_node),
            )
        )

        self.assertTrue(self._wait_for_distinct_sessions(2, 10.0), self.sessions)
        distinct_sessions = list(dict.fromkeys(self.sessions))
        second_session = distinct_sessions[1]
        self.assertNotEqual(second_session, first_session)

        second_start = self.sessions.index(second_session)
        self.assertTrue(
            all(session == second_session for session in self.sessions[second_start:]),
            "the restarted producer must not emit the previous process session",
        )
