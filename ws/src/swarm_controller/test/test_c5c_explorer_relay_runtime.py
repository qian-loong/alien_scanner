import copy
import os
import time
import unittest

import launch
import launch_testing
import launch_testing.asserts
import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import AnyLaunchDescriptionSource
from perception_interfaces.msg import MapUpdate
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool, Trigger
from swarm_data_interfaces.msg import (
    DeliveryAck,
    LinkDiagnostic,
    MemberRecord,
    RoleAssignment,
    RoleSnapshot,
    RoleTransitionAck,
    RoleTransitionDescriptor,
    RoutedMapUpdate,
    TopologySnapshot,
)


def generate_test_description():
    domain_id = str(200 + (os.getpid() % 20))
    launch_file = os.path.join(
        get_package_share_directory("swarm_controller"),
        "launch",
        "c5c_explorer_relay.launch.xml",
    )
    system = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(launch_file),
        launch_arguments={
            "show_rviz": "false",
            "show_cave_truth": "false",
            "enable_explorers": "true",
        }.items(),
    )
    return launch.LaunchDescription(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", domain_id),
            system,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestC5cExplorerRelayRuntime(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)
        cls.node = rclpy.create_node("c5c_explorer_relay_runtime_test")
        stream_qos = QoSProfile(depth=100)
        stream_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        transition_qos = QoSProfile(depth=20)
        transition_qos.reliability = ReliabilityPolicy.RELIABLE
        transition_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        cls.topologies = []
        cls.roles = []
        cls.transitions = []
        cls.relay_diagnostics = []
        cls.receiver_inputs = {0: [], 1: []}
        cls.accepted = {0: [], 1: []}
        cls.acks = {0: [], 1: []}
        cls.relay_0_inputs = {0: [], 1: []}
        cls.explorer_diagnostics = []

        cls.node.create_subscription(
            TopologySnapshot,
            "/swarm/runtime/topology",
            lambda msg: cls.topologies.append(msg),
            state_qos,
        )
        cls.node.create_subscription(
            RoleSnapshot,
            "/swarm/runtime/roles",
            lambda msg: cls.roles.append(msg),
            state_qos,
        )
        cls.node.create_subscription(
            RoleTransitionDescriptor,
            "/swarm/runtime/transition",
            lambda msg: cls.transitions.append(msg),
            transition_qos,
        )
        cls.node.create_subscription(
            LinkDiagnostic,
            "/swarm/runtime/relay_diagnostics",
            lambda msg: cls.relay_diagnostics.append(msg),
            stream_qos,
        )
        for index in range(2):
            cls.node.create_subscription(
                RoutedMapUpdate,
                f"/c5c/receiver_{index}/input",
                lambda msg, source=index: cls.receiver_inputs[source].append(msg),
                stream_qos,
            )
            cls.node.create_subscription(
                MapUpdate,
                f"/c5c/receiver_{index}/accepted",
                lambda msg, source=index: cls.accepted[source].append(msg),
                stream_qos,
            )
            cls.node.create_subscription(
                DeliveryAck,
                f"/c5c/receiver_{index}/acks",
                lambda msg, source=index: cls.acks[source].append(msg),
                stream_qos,
            )
            cls.node.create_subscription(
                RoutedMapUpdate,
                f"/c5c/relay_0/route_map_{index}/input",
                lambda msg, source=index: cls.relay_0_inputs[source].append(msg),
                stream_qos,
            )
        cls.node.create_subscription(
            DiagnosticArray,
            "/drone_0/exploration_diagnostics",
            lambda msg: cls.explorer_diagnostics.append(msg),
            state_qos,
        )
        cls.old_route_publisher = cls.node.create_publisher(
            RoutedMapUpdate, "/c5c/relay_1/route_map_0/input", stream_qos
        )
        cls.fault_client = cls.node.create_client(
            SetBool, "/c5c/relay_0/set_faulted"
        )
        cls.transition_service = cls.node.create_client(
            Trigger, "/swarm/runtime/quiesce_explorer_0"
        )
        cls.initial_topology_epoch = None
        cls.initial_role_epoch = None
        cls.old_route_message = None

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def spin_until(cls, predicate, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    @staticmethod
    def route(snapshot, route_id):
        return next(
            (route for route in snapshot.routes if route.route_id == route_id), None
        )

    @staticmethod
    def member(snapshot, vehicle_id):
        return next(
            (
                member
                for member in snapshot.members
                if member.identity.vehicle_id == vehicle_id
            ),
            None,
        )

    @staticmethod
    def assignment(snapshot, vehicle_id):
        return next(
            (
                assignment
                for assignment in snapshot.assignments
                if assignment.identity.vehicle_id == vehicle_id
            ),
            None,
        )

    def test_01_two_sources_arrive_through_active_relay(self):
        self.assertTrue(
            self.spin_until(
                lambda: self.topologies
                and self.roles
                and all(self.accepted[index] for index in range(2))
                and all(self.relay_0_inputs[index] for index in range(2))
            ),
            "N=4 runtime did not deliver both source identities through Relay 0",
        )
        topology = self.topologies[-1]
        roles = self.roles[-1]
        type(self).initial_topology_epoch = topology.topology_epoch
        type(self).initial_role_epoch = roles.role_epoch
        for index in range(2):
            route = self.route(topology, f"route-map-{index}")
            self.assertIsNotNone(route)
            self.assertEqual(route.target.vehicle_id, "relay-0")
            self.assertEqual(route.route_epoch, 1)
            self.assertTrue(
                any(
                    message.map_update.vehicle_id == f"explorer-{index}"
                    and message.route_epoch == 1
                    and message.hop_count == 1
                    for message in self.receiver_inputs[index]
                )
            )
        type(self).old_route_message = copy.deepcopy(self.relay_0_inputs[0][0])
        self.assertEqual(type(self).old_route_message.route_epoch, 1)
        self.assertEqual(type(self).old_route_message.hop_count, 0)

    def test_02_relay_timeout_commits_failover_and_resync(self):
        self.assertIsNotNone(type(self).old_route_message)
        self.assertTrue(self.fault_client.wait_for_service(timeout_sec=10.0))
        request = SetBool.Request()
        request.data = True
        future = self.fault_client.call_async(request)
        self.assertTrue(self.spin_until(lambda: future.done(), timeout=10.0))
        self.assertTrue(future.result().success)

        def failover_committed():
            if not self.topologies or not self.roles:
                return False
            topology = self.topologies[-1]
            roles = self.roles[-1]
            route_0 = self.route(topology, "route-map-0")
            route_1 = self.route(topology, "route-map-1")
            relay_0 = self.member(topology, "relay-0")
            relay_1_role = self.assignment(roles, "relay-1")
            return (
                topology.topology_epoch > type(self).initial_topology_epoch
                and roles.role_epoch > type(self).initial_role_epoch
                and route_0 is not None
                and route_1 is not None
                and route_0.target.vehicle_id == "relay-1"
                and route_1.target.vehicle_id == "relay-1"
                and route_0.route_epoch == 2
                and route_1.route_epoch == 2
                and relay_0 is not None
                and relay_0.state == MemberRecord.STATE_LOST
                and relay_1_role is not None
                and relay_1_role.primary_role == RoleAssignment.PRIMARY_RELAY
            )

        self.assertTrue(
            self.spin_until(failover_committed),
            "authority did not commit the Relay 1 route and role snapshot",
        )
        self.assertTrue(
            any(
                diagnostic.endpoint_id.startswith("relay-0:")
                and diagnostic.event == LinkDiagnostic.EVENT_LINK_DOWN
                for diagnostic in self.relay_diagnostics
            ),
            "fault injection did not produce a real Relay link-down diagnostic",
        )

        self.old_route_publisher.publish(type(self).old_route_message)
        self.assertTrue(
            self.spin_until(
                lambda: any(
                    diagnostic.endpoint_id == "relay-1:route-map-0"
                    and diagnostic.event == LinkDiagnostic.EVENT_REJECTED
                    and diagnostic.route_epoch == 1
                    for diagnostic in self.relay_diagnostics
                ),
                timeout=10.0,
            ),
            "Relay 1 did not reject the delayed old-route message",
        )

        self.assertTrue(
            self.spin_until(
                lambda: all(
                    any(
                        message.route_epoch == 2
                        and message.hop_count == 1
                        and message.map_update.vehicle_id == f"explorer-{index}"
                        for message in self.receiver_inputs[index]
                    )
                    for index in range(2)
                )
                and all(
                    any(ack.resync_required for ack in self.acks[index])
                    for index in range(2)
                )
                and all(
                    any(message.correlation_id for message in self.accepted[index])
                    for index in range(2)
                ),
                timeout=30.0,
            ),
            "both sources did not converge through route epoch 2 correlated keyframes",
        )

    def test_03_explorer_transition_reaches_runtime_hold(self):
        self.assertTrue(
            self.transition_service.wait_for_service(timeout_sec=15.0),
            "authority Explorer transition service is unavailable",
        )
        future = self.transition_service.call_async(Trigger.Request())
        self.assertTrue(self.spin_until(lambda: future.done(), timeout=10.0))
        self.assertTrue(future.result().success, future.result().message)
        self.assertTrue(
            self.spin_until(
                lambda: any(
                    transition.transition_id.startswith("explorer-quiesce-")
                    and transition.state == RoleTransitionDescriptor.STATE_COMMITTED
                    for transition in self.transitions
                ),
                timeout=15.0,
            ),
            "authority did not publish a committed Explorer transition",
        )
        self.assertEqual(
            {
                ack.kind
                for transition in self.transitions
                if transition.transition_id.startswith("explorer-quiesce-")
                for ack in transition.acknowledgements
            },
            {
                RoleTransitionAck.ACK_QUIESCED,
                RoleTransitionAck.ACK_HANDOFF_READY,
            },
        )
        self.assertTrue(
            any(
                self.assignment(snapshot, "explorer-0") is not None
                and self.assignment(snapshot, "explorer-0").lifecycle
                == RoleAssignment.LIFECYCLE_DRAINING
                for snapshot in self.roles
            )
        )
        self.assertTrue(
            self.spin_until(
                lambda: any(
                    status.message.startswith("RuntimeHold:")
                    for message in self.explorer_diagnostics
                    for status in message.status
                ),
                timeout=10.0,
            ),
            "Explorer did not publish RuntimeHold diagnostics while quiescing",
        )


@launch_testing.post_shutdown_test()
class TestC5cProcessesExit(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
