import os
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.asserts
from nav_msgs.msg import Path as NavPath
from octomap_msgs.msg import Octomap
from perception_interfaces.msg import MapUpdate
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from swarm_data_interfaces.msg import DeliveryAck
from visualization_msgs.msg import MarkerArray


def generate_test_description():
    domain_id = str(100 + (os.getpid() % 80))
    os.environ["ROS_DOMAIN_ID"] = domain_id
    share = Path(get_package_share_directory("swarm_data_plane"))
    scene = share / "launch" / "c4_cave_visual_validation.launch.py"
    return (
        launch.LaunchDescription(
            [
                SetEnvironmentVariable("ROS_DOMAIN_ID", domain_id),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(scene)),
                    launch_arguments={
                        "show_rviz": "false",
                        "a_duration_s": "7.0",
                        "b_duration_s": "3.2",
                        "odom_rate_hz": "20.0",
                        "scan_rate_hz": "6.0",
                        "resync_hold_s": "0.8",
                        "scanner_startup_delay_s": "2.0",
                    }.items(),
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestC4CaveVisualValidation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)
        cls.node = rclpy.create_node("test_c4_cave_visual_validation")
        cls.acks = []
        cls.a1_updates = []
        cls.a2_updates = []
        cls.a1_maps = []
        cls.a2_maps = []
        cls.markers = []
        cls.paths = {"a1": [], "a2": [], "b": []}

        volatile = QoSProfile(depth=128)
        volatile.reliability = ReliabilityPolicy.RELIABLE
        volatile.durability = DurabilityPolicy.VOLATILE
        transient = QoSProfile(depth=4)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL

        cls.subscriptions = [
            cls.node.create_subscription(
                DeliveryAck,
                "/c4/cave/b/delivery_ack",
                cls.acks.append,
                volatile,
            ),
            cls.node.create_subscription(
                MapUpdate,
                "/c4/cave/b/a1/accepted_updates",
                cls.a1_updates.append,
                volatile,
            ),
            cls.node.create_subscription(
                MapUpdate,
                "/c4/cave/b/a2/accepted_updates",
                cls.a2_updates.append,
                volatile,
            ),
            cls.node.create_subscription(
                Octomap,
                "/c4/cave/b/a1/accepted_octomap",
                cls.a1_maps.append,
                transient,
            ),
            cls.node.create_subscription(
                Octomap,
                "/c4/cave/b/a2/accepted_octomap",
                cls.a2_maps.append,
                transient,
            ),
            cls.node.create_subscription(
                MarkerArray,
                "/c4/cave/b/communication_status",
                cls.markers.append,
                transient,
            ),
        ]
        for label in cls.paths:
            cls.subscriptions.append(
                cls.node.create_subscription(
                    NavPath,
                    f"/c4/cave/{label}/path",
                    cls.paths[label].append,
                    transient,
                )
            )

    @classmethod
    def tearDownClass(cls):
        for subscription in cls.subscriptions:
            cls.node.destroy_subscription(subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    @staticmethod
    def last_path_x(messages):
        if not messages or not messages[-1].poses:
            return float("-inf")
        return messages[-1].poses[-1].pose.position.x

    def test_real_cave_chain_freezes_a2_then_recovers_without_blocking_a1(self):
        def a2_trace():
            return [
                (
                    ack.status,
                    ack.resync_required,
                    ack.source_revision,
                    ack.correlation_id,
                    ack.diagnostic,
                )
                for ack in self.acks
                if ack.source_vehicle_id == "c4-a2"
            ]

        def drop_ack():
            return next(
                (
                    ack
                    for ack in self.acks
                    if ack.source_vehicle_id == "c4-a2"
                    and ack.status == DeliveryAck.STATUS_REJECTED
                    and "deterministic delta drop" in ack.diagnostic
                ),
                None,
            )

        def gap_ack():
            return next(
                (
                    ack
                    for ack in self.acks
                    if ack.source_vehicle_id == "c4-a2"
                    and ack.status == DeliveryAck.STATUS_REJECTED
                    and ack.resync_required
                    and "REJECTED_GAP" in ack.diagnostic
                ),
                None,
            )

        def recovered_ack():
            return next(
                (
                    ack
                    for ack in self.acks
                    if ack.source_vehicle_id == "c4-a2"
                    and ack.status == DeliveryAck.STATUS_DELIVERED
                    and ack.correlation_id
                    and "APPLIED_KEYFRAME" in ack.diagnostic
                ),
                None,
            )

        self.assertTrue(
            self.spin_until(
                lambda: drop_ack() is not None
                and gap_ack() is not None
                and recovered_ack() is not None,
                timeout=25.0,
            ),
            "A2 did not complete deterministic drop, gap, and correlated recovery; "
            f"received {len(self.acks)} acknowledgements; A2 trace="
            f"{a2_trace()}",
        )
        dropped = drop_ack()
        gap = gap_ack()
        recovered = recovered_ack()
        self.assertIsNotNone(dropped)
        self.assertIsNotNone(gap)
        self.assertIsNotNone(recovered)
        self.assertGreater(gap.receive_monotonic_ns, dropped.receive_monotonic_ns)
        self.assertGreater(recovered.receive_monotonic_ns, gap.receive_monotonic_ns)
        self.assertGreater(recovered.source_revision, dropped.source_revision)

        self.assertTrue(
            self.spin_until(
                lambda: len(
                    {
                        ack.source_revision
                        for ack in self.acks
                        if ack.source_vehicle_id == "c4-a1"
                        and ack.status == DeliveryAck.STATUS_DELIVERED
                        and ack.receive_monotonic_ns > gap.receive_monotonic_ns
                    }
                )
                >= 2,
                timeout=5.0,
            ),
            "A1 did not advance independently after the A2 gap was detected",
        )

        self.assertTrue(
            self.spin_until(
                lambda: self.a1_updates
                and any(
                    update.correlation_id == recovered.correlation_id
                    for update in self.a2_updates
                )
                and self.a1_maps
                and self.a1_maps[-1].data
                and self.a2_maps
                and self.a2_maps[-1].data,
                timeout=5.0,
            ),
            "accepted updates or reconstructed maps did not arrive after ack",
        )

        self.assertTrue(
            self.spin_until(
                lambda: self.last_path_x(self.paths["a1"]) >= 7.8
                and self.last_path_x(self.paths["a2"]) >= 7.8
                and self.last_path_x(self.paths["b"]) >= 3.4,
                timeout=15.0,
            ),
            "A1/A2 did not finish 8 m paths or B did not finish its 3.5 m path",
        )
        self.assertNotEqual(
            bytes(self.a1_maps[-1].data),
            bytes(self.a2_maps[-1].data),
            "different Y tracks unexpectedly produced identical accepted maps",
        )

        self.assertTrue(
            self.spin_until(
                lambda: any(
                    marker.text.startswith("A2 -> B  RECOVERED")
                    for message in self.markers
                    for marker in message.markers
                ),
                timeout=3.0,
            ),
            "the manual-validation status layer did not expose A2 recovery",
        )
        recovered_marker_message = next(
            message
            for message in reversed(self.markers)
            if any(
                marker.text.startswith("A2 -> B  RECOVERED")
                for marker in message.markers
            )
        )
        link_markers = [
            marker
            for marker in recovered_marker_message.markers
            if marker.ns == "c4_cave/links"
        ]
        vehicle_markers = [
            marker
            for marker in recovered_marker_message.markers
            if marker.ns == "c4_cave/vehicles"
        ]
        self.assertEqual(len(link_markers), 2)
        self.assertTrue(all(len(marker.points) == 16 for marker in link_markers))
        self.assertEqual(len(vehicle_markers), 3)


@launch_testing.post_shutdown_test()
class TestC4CaveVisualValidationProcessesExit(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
