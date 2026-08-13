import os
import time
import unittest

import launch
import launch_testing
import launch_testing.asserts
import rclpy
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from perception_interfaces.msg import MapUpdate
from rclpy.qos import QoSProfile, ReliabilityPolicy
from swarm_data_interfaces.msg import DeliveryAck, RoutedMapUpdate


def generate_test_description():
    domain_id = str(100 + (os.getpid() % 100))
    common = {
        "output": "screen",
        "emulate_tty": True,
    }
    producer = Node(
        package="swarm_data_plane",
        executable="map_update_route_producer_fixture",
        name="c4_route_producer_fixture",
        parameters=[
            {
                "output_topic": "/c4/source_map",
                "resync_service": "/c4/resync",
            }
        ],
        **common,
    )
    source = Node(
        package="swarm_data_plane",
        executable="map_update_route_source_node",
        name="c4_route_source",
        parameters=[
            {
                "input_topic": "/c4/source_map",
                "output_topic": "/c4/routed_before_fault",
                "producer_id": "mapper_endpoint",
                "producer_session_boot_time_ns": 300,
                "producer_session_random_suffix": 11,
                "origin_clock_domain": "steady-sim",
                "origin_clock_session_boot_time_ns": 400,
                "origin_clock_session_random_suffix": 12,
                "route_epoch": 1,
                "ttl_hops": 8,
            }
        ],
        **common,
    )
    link = Node(
        package="swarm_data_plane",
        executable="map_update_route_link_fixture",
        name="c4_route_link_fixture",
        parameters=[
            {
                "input_topic": "/c4/routed_before_fault",
                "output_topic": "/c4/routed_after_fault",
            }
        ],
        **common,
    )
    receiver = Node(
        package="swarm_data_plane",
        executable="map_update_route_receiver_node",
        name="c4_route_receiver",
        parameters=[
            {
                "input_topic": "/c4/routed_after_fault",
                "accepted_topic": "/c4/accepted_map",
                "ack_topic": "/c4/acks",
                "resync_service": "/c4/resync",
                "requester_id": "receiver",
                "requester_session_boot_time_ns": 500,
                "requester_session_random_suffix": 3,
                "expected_producer_id": "mapper_endpoint",
                "expected_producer_session_boot_time_ns": 300,
                "expected_producer_session_random_suffix": 11,
                "expected_vehicle_id": "drone_0",
                "expected_mapper_session_boot_time_ns": 100,
                "expected_mapper_session_random_suffix": 7,
                "expected_map_epoch": 1,
            }
        ],
        **common,
    )
    return (
        launch.LaunchDescription(
            [
                SetEnvironmentVariable("ROS_DOMAIN_ID", domain_id),
                producer,
                source,
                link,
                receiver,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "producer": producer,
            "source": source,
            "link": link,
            "receiver": receiver,
        },
    )


class TestSwarmDataPlaneClosedLoop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)
        cls.node = rclpy.create_node("c4_closed_loop_test")
        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.RELIABLE
        cls.accepted = []
        cls.acks = []
        cls.routed_before = []
        cls.routed_after = []
        cls.node.create_subscription(
            MapUpdate, "/c4/accepted_map", lambda msg: cls.accepted.append(msg), qos
        )
        cls.node.create_subscription(
            DeliveryAck, "/c4/acks", lambda msg: cls.acks.append(msg), qos
        )
        cls.node.create_subscription(
            RoutedMapUpdate,
            "/c4/routed_before_fault",
            lambda msg: cls.routed_before.append(msg),
            qos,
        )
        cls.node.create_subscription(
            RoutedMapUpdate,
            "/c4/routed_after_fault",
            lambda msg: cls.routed_after.append(msg),
            qos,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def spin_until(cls, predicate, timeout=25.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def test_source_link_receiver_resync_closed_loop(self):
        self.assertTrue(
            self.spin_until(
                lambda: any(msg.new_revision == 3 for msg in self.accepted)
                and any(
                    ack.source_revision == 3
                    and ack.status == DeliveryAck.STATUS_DELIVERED
                    for ack in self.acks
                )
                and any(msg.sequence == 4 for msg in self.routed_after)
            ),
            "receiver did not recover revision 3 through correlated keyframe",
        )
        self.assertGreaterEqual(len(self.routed_before), 4)
        self.assertTrue(any(msg.sequence == 2 for msg in self.routed_before))
        self.assertFalse(any(msg.sequence == 2 for msg in self.routed_after))
        self.assertTrue(any(msg.sequence == 4 for msg in self.routed_after))
        self.assertEqual(
            [msg.new_revision for msg in self.accepted],
            [1, 3],
        )
        self.assertTrue(
            any(
                ack.status == DeliveryAck.STATUS_REJECTED
                and ack.resync_required
                and ack.source_revision == 3
                for ack in self.acks
            )
        )
        recovery = [msg for msg in self.accepted if msg.new_revision == 3]
        self.assertEqual(len(recovery), 1)
        self.assertNotEqual(recovery[0].correlation_id, "")
        self.assertTrue(
            all(msg.hop_count == 1 for msg in self.routed_after),
            "logical link must account for one forwarding hop",
        )


@launch_testing.post_shutdown_test()
class TestSwarmDataPlaneProcessesExit(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
