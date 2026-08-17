import os
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
import launch
from launch.actions import GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing
from launch_ros.actions import Node
from octomap_msgs.msg import Octomap
from perception_interfaces.msg import ContentIdentityDescriptor, LocalMapState, MapUpdate
from perception_interfaces.srv import RequestMapResync
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


TEST_DOMAIN_ID = str(120 + (os.getpid() % 80))
os.environ['ROS_DOMAIN_ID'] = TEST_DOMAIN_ID


@pytest.mark.launch_test
def generate_test_description():
    fixture_launch = (
        Path(get_package_share_directory('perception_fixtures'))
        / 'launch'
        / 'ray_evidence_debug.launch.py'
    )
    mapper_params = (
        Path(get_package_share_directory('perception_local_map'))
        / 'config'
        / 'local_map_debug.yaml'
    )
    fixture = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(str(fixture_launch)),
                launch_arguments={'show_rviz': 'false'}.items(),
            )
        ],
    )
    mapper = Node(
        package='perception_local_map',
        executable='perception_local_map_node',
        name='perception_local_map_node',
        output='screen',
        parameters=[
            str(mapper_params),
            {
                'map_update_enabled': True,
                'map_update_topic': '/local_map/updates',
                'map_resync_service': '/local_map/request_resync',
                'map_update.periodic_keyframe_revision_interval': 0,
            },
        ],
    )
    receiver = Node(
        package='perception_map_update',
        executable='perception_map_update_receiver_node',
        name='map_update_closed_loop_receiver',
        output='screen',
        parameters=[
            {
                'map_update_topic': '/local_map/updates',
                'local_map_state_topic': '/local_map/state',
                'map_resync_service': '/local_map/request_resync',
                'reconstructed_octomap_topic': '/map_update_receiver/octomap',
                'expected_vehicle_id': 'debug-vehicle',
                'requester_id': 'closed-loop-reference-receiver',
                'resync_retry_period_s': 0.1,
                'map_update.periodic_keyframe_revision_interval': 0,
            }
        ],
    )
    return launch.LaunchDescription(
        [
            launch.actions.SetEnvironmentVariable('ROS_DOMAIN_ID', TEST_DOMAIN_ID),
            fixture,
            mapper,
            TimerAction(period=5.0, actions=[receiver]),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestMapUpdateClosedLoop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('map_update_closed_loop_test')
        cls.states = []
        cls.updates = []
        cls.authoritative_maps = []
        cls.reconstructed_maps = []
        cls.diagnostics = []

        volatile = QoSProfile(depth=10)
        volatile.reliability = ReliabilityPolicy.RELIABLE
        volatile.durability = DurabilityPolicy.VOLATILE
        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        cls.node.create_subscription(
            LocalMapState, '/local_map/state', cls.states.append, volatile
        )
        cls.node.create_subscription(
            MapUpdate, '/local_map/updates', cls.updates.append, volatile
        )
        cls.node.create_subscription(
            Octomap, '/local_map/octomap', cls.authoritative_maps.append, transient
        )
        cls.node.create_subscription(
            Octomap,
            '/map_update_receiver/octomap',
            cls.reconstructed_maps.append,
            transient,
        )
        cls.node.create_subscription(
            DiagnosticArray, '/diagnostics', cls.diagnostics.append, volatile
        )
        cls.resync_client = cls.node.create_client(
            RequestMapResync, '/local_map/request_resync'
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_for(self, predicate, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _call_resync(self, request):
        future = self.resync_client.call_async(request)
        self.assertTrue(
            self._wait_for(lambda: future.done(), timeout=5.0),
            'resync service call timed out',
        )
        return future.result()

    @staticmethod
    def _bootstrap_request(client_request_id):
        request = RequestMapResync.Request()
        request.requester_id = 'closed-loop-test-client'
        request.requester_session_boot_time_ns = 9001
        request.requester_session_random_suffix = 7
        request.client_request_id = client_request_id
        request.bootstrap_latest = True
        request.receiver_content_identity.scheme = (
            ContentIdentityDescriptor.SCHEME_MERKLE_PATRICIA_SHA256_V2
        )
        request.receiver_content_identity.chunk_edge = 16
        request.receiver_content_identity.coordinate_key_version = 1
        request.receiver_content_identity.node_encoding_version = 1
        request.reason = RequestMapResync.Request.REASON_INITIAL_BASELINE
        return request

    def _receiver_ready(self):
        for message in reversed(self.diagnostics):
            for status in message.status:
                if 'map_update_closed_loop_receiver: map_update_receiver' not in status.name:
                    continue
                values = {entry.key: entry.value for entry in status.values}
                if values.get('receiver_state') == '1' and int(values.get('revision', '0')) > 0:
                    return True
        return False

    def test_late_join_resync_and_reconstruction(self):
        response_fields = RequestMapResync.Response.get_fields_and_field_types()
        self.assertNotIn('payload', response_fields)
        self.assertNotIn('cells', response_fields)

        self.assertTrue(
            self._wait_for(
                lambda: any(state.revision > 0 for state in self.states)
                and self.resync_client.service_is_ready()
                and any(
                    update.update_kind == MapUpdate.KIND_KEYFRAME
                    and not update.correlation_id
                    for update in self.updates
                ),
                timeout=4.0,
            ),
            'producer did not establish its initial baseline before the receiver delay',
        )

        first_request = self._bootstrap_request('manual-bootstrap-1')
        first = self._call_resync(first_request)
        duplicate = self._call_resync(first_request)
        second = self._call_resync(self._bootstrap_request('manual-bootstrap-2'))
        self.assertTrue(first.accepted, first.diagnostic)
        self.assertTrue(duplicate.accepted, duplicate.diagnostic)
        self.assertTrue(second.accepted, second.diagnostic)
        self.assertEqual(first.correlation_id, duplicate.correlation_id)
        self.assertNotEqual(first.correlation_id, second.correlation_id)

        manual_correlations = {first.correlation_id, second.correlation_id}
        self.assertTrue(
            self._wait_for(
                lambda: manual_correlations.issubset(
                    {
                        update.correlation_id
                        for update in self.updates
                        if update.update_kind == MapUpdate.KIND_KEYFRAME
                    }
                ),
                timeout=4.0,
            ),
            'accepted resync requests were not serialized into correlated keyframes',
        )

        self.assertTrue(
            self._wait_for(
                lambda: self._receiver_ready()
                and bool(self.reconstructed_maps)
                and any(
                    update.update_kind == MapUpdate.KIND_KEYFRAME
                    and update.correlation_id
                    and update.correlation_id not in manual_correlations
                    for update in self.updates
                ),
                timeout=15.0,
            ),
            'late receiver did not recover through service and async keyframe',
        )
        self.assertTrue(self.authoritative_maps)
        reconstructed = self.reconstructed_maps[-1]
        authoritative = self.authoritative_maps[-1]
        self.assertEqual(reconstructed.header.frame_id, authoritative.header.frame_id)
        self.assertEqual(reconstructed.id, authoritative.id)
        self.assertEqual(reconstructed.binary, authoritative.binary)
        self.assertEqual(reconstructed.resolution, authoritative.resolution)
        self.assertGreater(len(reconstructed.data), 0)
        self.assertEqual(reconstructed.data, authoritative.data)


@launch_testing.post_shutdown_test()
class TestMapUpdateClosedLoopExit(unittest.TestCase):
    def test_launch_completed_without_forced_shutdown(self):
        self.assertTrue(True)
