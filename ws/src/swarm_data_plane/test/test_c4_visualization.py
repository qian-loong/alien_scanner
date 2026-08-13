import os
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_testing
import launch_testing.asserts
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import rclpy
from visualization_msgs.msg import MarkerArray


def generate_test_description():
    domain_id = str(150 + (os.getpid() % 50))
    os.environ["ROS_DOMAIN_ID"] = domain_id
    share = Path(get_package_share_directory("swarm_data_plane"))
    return (
        launch.LaunchDescription(
            [
                SetEnvironmentVariable("ROS_DOMAIN_ID", domain_id),
                IncludeLaunchDescription(
                    FrontendLaunchDescriptionSource(
                        str(share / "launch" / "c4_visualization.launch.xml")
                    ),
                    launch_arguments={
                        "show_rviz": "false",
                        "publish_rate_hz": "10.0",
                        "scenario_step_period_s": "0.15",
                    }.items(),
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestC4Visualization(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)
        cls.node = rclpy.create_node("test_c4_visualization")
        cls.messages = {name: [] for name in cls.expected_titles}
        qos = QoSProfile(depth=4)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        cls.subscriptions = [
            cls.node.create_subscription(
                MarkerArray,
                f"/c4/visualization/{name}",
                cls.messages[name].append,
                qos,
            )
            for name in cls.expected_titles
        ]

    @classmethod
    def tearDownClass(cls):
        for subscription in cls.subscriptions:
            cls.node.destroy_subscription(subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    expected_titles = {
        "edge_recovery": {
            "READY BASELINE",
            "DELTA DROPPED",
            "GAP REJECTED",
            "RESYNC RECOVERED",
        },
        "edge_aggregation": {
            "CONTRIBUTORS READY",
            "A2 GAP",
            "A2 RESYNC",
            "AGGREGATE COMMITTED",
        },
        "upstream_recovery": {
            "AGGREGATE BASELINE",
            "UPSTREAM GAP",
            "CENTRAL RESYNC",
            "CENTRAL RECOVERED",
        },
        "multihop_ttl": {
            "SOURCE HOP 0",
            "RELAY B1 HOP 1",
            "B2 TTL REJECTED",
            "ROUTE SAFE",
        },
    }

    def spin_until(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    @staticmethod
    def stage_titles(messages):
        return {
            marker.text.splitlines()[0]
            for message in messages
            for marker in message.markers
            if marker.ns.endswith("/status") and marker.text
        }

    @staticmethod
    def markers_for_namespace(message, suffix):
        return [marker for marker in message.markers if marker.ns.endswith(suffix)]

    def test_four_visualization_groups_publish_core_backed_stages(self):
        self.assertTrue(
            self.spin_until(
                lambda: all(self.messages[name] for name in self.expected_titles)
            ),
            "C4 visualization fixture did not publish all four topics",
        )
        for name, expected in self.expected_titles.items():
            self.assertTrue(
                self.spin_until(
                    lambda name=name, expected=expected: self.stage_titles(
                        self.messages[name]
                    ) >= expected,
                    timeout=5.0,
                ),
                f"visualization scene {name} did not complete all stages",
            )
            latest = self.messages[name][-1]
            self.assertTrue(self.markers_for_namespace(latest, "/status"))
            self.assertTrue(self.markers_for_namespace(latest, "/timeline"))
            self.assertTrue(self.markers_for_namespace(latest, "/events"))

        edge_gap = next(
            message
            for message in self.messages["edge_recovery"]
            if "GAP REJECTED"
            in self.stage_titles([message])
        )
        edge_gap_status = self.markers_for_namespace(edge_gap, "/status")[0]
        self.assertIn("expected sequence 2", edge_gap_status.text)
        self.assertIn("received sequence 3", edge_gap_status.text)
        self.assertIn("RESYNC_REQUIRED", edge_gap_status.text)
        self.assertGreater(edge_gap_status.color.r, edge_gap_status.color.g)

        edge_recovered = next(
            message
            for message in self.messages["edge_recovery"]
            if "RESYNC RECOVERED"
            in self.stage_titles([message])
        )
        self.assertIn(
            "correlation edge-recovery",
            self.markers_for_namespace(edge_recovered, "/status")[0].text,
        )
        self.assertGreater(
            self.markers_for_namespace(edge_recovered, "/status")[0].color.g,
            self.markers_for_namespace(edge_recovered, "/status")[0].color.r,
        )

        aggregate = next(
            message
            for message in self.messages["edge_aggregation"]
            if "AGGREGATE COMMITTED"
            in self.stage_titles([message])
        )
        self.assertGreaterEqual(
            len(self.markers_for_namespace(aggregate, "/A1")), 1
        )
        self.assertGreaterEqual(
            len(self.markers_for_namespace(aggregate, "/A2")), 1
        )
        self.assertGreaterEqual(
            len(self.markers_for_namespace(aggregate, "/aggregate_map")), 1
        )

        ttl = next(
            message
            for message in self.messages["multihop_ttl"]
            if "B2 TTL REJECTED" in self.stage_titles([message])
        )
        self.assertIn(
            "exhaust TTL",
            self.markers_for_namespace(ttl, "/status")[0].text,
        )
        self.assertGreater(
            self.markers_for_namespace(ttl, "/status")[0].color.r,
            self.markers_for_namespace(ttl, "/status")[0].color.g,
        )


@launch_testing.post_shutdown_test()
class TestC4VisualizationProcessesExit(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
