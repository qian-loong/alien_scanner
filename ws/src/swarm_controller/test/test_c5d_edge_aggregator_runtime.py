import copy
import os
import time
import unittest

import launch
import launch_testing
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import AnyLaunchDescriptionSource
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool
from visualization_msgs.msg import MarkerArray
from swarm_data_interfaces.msg import (
    AggregateMapUpdate,
    LinkDiagnostic,
    RoleAssignment,
    RoleSnapshot,
    TopologySnapshot,
    RoutedMapUpdate,
)


def generate_test_description():
    domain_id = str(220 + (os.getpid() % 10))
    launch_file = os.path.join(
        get_package_share_directory("swarm_controller"),
        "launch",
        "c5d_edge_aggregator.launch.xml",
    )
    system = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(launch_file),
        launch_arguments={"show_rviz": "false", "show_cave_truth": "false"}.items(),
    )
    return launch.LaunchDescription(
        [SetEnvironmentVariable("ROS_DOMAIN_ID", domain_id), system, launch_testing.actions.ReadyToTest()]
    )


class TestC5dEdgeAggregatorRuntime(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)
        cls.node = rclpy.create_node("c5d_edge_aggregator_runtime_test")
        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        cls.topologies = []
        cls.roles = []
        cls.accepted = []
        cls.diagnostics = []
        cls.edge_inputs = [[], []]
        cls.relay_0_inputs = []
        cls.relay_diagnostics = []
        cls.marker_texts = []
        cls.node.create_subscription(TopologySnapshot, "/swarm/runtime/topology", cls.topologies.append, state_qos)
        cls.node.create_subscription(RoleSnapshot, "/swarm/runtime/roles", cls.roles.append, state_qos)
        cls.node.create_subscription(AggregateMapUpdate, "/c5d/aggregate/accepted", cls.accepted.append, qos)
        cls.node.create_subscription(LinkDiagnostic, "/swarm/runtime/aggregate_diagnostics", cls.diagnostics.append, qos)
        for index in range(2):
            cls.node.create_subscription(
                RoutedMapUpdate,
                f"/c5d/edge/input_{index}",
                lambda message, source=index: cls.edge_inputs[source].append(message),
                qos,
            )
        cls.node.create_subscription(
            RoutedMapUpdate,
            "/c5d/relay_0/route_map_0/input",
            cls.relay_0_inputs.append,
            qos,
        )
        cls.node.create_subscription(
            LinkDiagnostic, "/swarm/runtime/relay_diagnostics", cls.relay_diagnostics.append, qos
        )
        cls.node.create_subscription(
            MarkerArray,
            "/swarm/runtime/markers",
            lambda message: cls.marker_texts.append(
                {marker.ns: marker.text for marker in message.markers if marker.text}
            ),
            qos,
        )
        cls.fault_client = cls.node.create_client(SetBool, "/c5d/relay_0/set_faulted")
        cls.old_route_publisher = cls.node.create_publisher(
            RoutedMapUpdate, "/c5d/relay_1/route_map_0/input", qos
        )
        cls.initial_topology_epoch = None
        cls.initial_contributor_revisions = {}
        cls.old_route_message = None

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def spin_until(cls, predicate, timeout=45.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def test_01_two_sources_converge_to_one_aggregate_source(self):
        self.assertTrue(
            self.spin_until(
                lambda: self.topologies
                and self.roles
                and self.accepted
                and self.relay_0_inputs
                and all(self.edge_inputs)
                and any(
                    assignment.identity.vehicle_id == "edge-aggregator"
                    and assignment.primary_role == RoleAssignment.PRIMARY_EDGE_AGGREGATOR
                        for assignment in self.roles[-1].assignments
                )
                and any(
                    len(message.manifest.contributors) >= 2
                    for message in self.accepted
                )
                and any(
                    "active 2/2" in markers.get("aggregate_status", "")
                    and "resync no" in markers.get("aggregate_status", "")
                    and "explorer-0 active" in markers.get("aggregate_contributors", "")
                    and "explorer-1 active" in markers.get("aggregate_contributors", "")
                    for markers in self.marker_texts
                )
            ),
            "N=5 topology/role/aggregate chain did not converge",
        )
        self.assertTrue(
            all(message.aggregate_update.map_update.vehicle_id == "edge-aggregator" for message in self.accepted),
            "central receiver observed a non-aggregate source",
        )
        type(self).initial_topology_epoch = self.topologies[-1].topology_epoch
        complete = next(
            message
            for message in reversed(self.accepted)
            if len(message.manifest.contributors) >= 2
        )
        type(self).initial_contributor_revisions = {
            contributor.vehicle_id: contributor.revision
            for contributor in complete.manifest.contributors
        }
        type(self).old_route_message = copy.deepcopy(self.relay_0_inputs[0])
        self.assertEqual(type(self).old_route_message.route_epoch, 1)
        self.assertEqual(type(self).old_route_message.hop_count, 0)

    def test_02_relay_failover_rejects_old_route_and_recovers_contributors(self):
        self.assertIsNotNone(type(self).old_route_message)
        self.assertTrue(self.fault_client.wait_for_service(timeout_sec=10.0))
        request = SetBool.Request()
        request.data = True
        future = self.fault_client.call_async(request)
        self.assertTrue(self.spin_until(lambda: future.done(), timeout=10.0))
        self.assertTrue(future.result().success)

        def failover_committed():
            return bool(self.topologies) and self.topologies[-1].topology_epoch > self.initial_topology_epoch and any(
                route.route_epoch == 2 and route.target.vehicle_id == "edge-aggregator"
                for route in self.topologies[-1].routes
                if route.route_id == "route-map-0"
            )

        self.assertTrue(self.spin_until(failover_committed, timeout=30.0))
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
            "standby Relay did not reject the delayed old-route message",
        )

        def aggregate_recovered():
            recovery_revisions = {}
            for index in range(2):
                matching = [
                    message
                    for message in self.edge_inputs[index]
                    if message.route_epoch == 2
                    and message.correlation_id
                    and message.map_update.vehicle_id == f"explorer-{index}"
                ]
                if not matching:
                    return False
                recovery_revisions[f"explorer-{index}"] = matching[-1].map_update.new_revision
            for aggregate in reversed(self.accepted):
                contributors = {
                    contributor.vehicle_id: contributor
                    for contributor in aggregate.manifest.contributors
                }
                if all(
                    vehicle_id in contributors
                    and contributors[vehicle_id].active
                    and contributors[vehicle_id].revision >= revision
                    and contributors[vehicle_id].revision
                    > type(self).initial_contributor_revisions[vehicle_id]
                    for vehicle_id, revision in recovery_revisions.items()
                ):
                    return True
            return False

        self.assertTrue(
            self.spin_until(aggregate_recovered, timeout=30.0),
            "aggregate did not recover both contributors after route failover",
        )
        self.assertTrue(
            any(
                "resync yes" in markers.get("aggregate_status", "")
                for markers in self.marker_texts
            ),
            "runtime markers never exposed the contributor resync barrier; "
            f"diagnostics={[(item.event, item.fault_reason, item.diagnostic) for item in self.diagnostics[-10:]]}; "
            f"markers={self.marker_texts[-5:]}",
        )
        self.assertTrue(
            self.spin_until(
                lambda: any(
                    "active 2/2" in markers.get("aggregate_status", "")
                    and "degraded no" in markers.get("aggregate_status", "")
                    and "resync no" in markers.get("aggregate_status", "")
                    for markers in self.marker_texts[-10:]
                ),
                timeout=5.0,
            ),
            "runtime markers did not return to healthy aggregate state",
        )


@launch_testing.post_shutdown_test()
class TestC5dShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
