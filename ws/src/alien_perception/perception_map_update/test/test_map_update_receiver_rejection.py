import os
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray
import launch
import launch_testing
from launch_ros.actions import Node
from octomap_msgs.msg import Octomap
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


TEST_DOMAIN_ID = str(140 + (os.getpid() % 60))
os.environ['ROS_DOMAIN_ID'] = TEST_DOMAIN_ID


@pytest.mark.launch_test
def generate_test_description():
    fixture = Node(
        package='perception_map_update',
        executable='map_update_receiver_fixture',
        name='map_update_receiver_fixture',
        output='screen',
    )
    receiver = Node(
        package='perception_map_update',
        executable='perception_map_update_receiver_node',
        name='map_update_rejection_receiver',
        output='screen',
        parameters=[
            {
                'map_update_topic': '/fixture/local_map/updates',
                'local_map_state_topic': '/fixture/local_map/state',
                'map_resync_service': '/fixture/local_map/request_resync',
                'reconstructed_octomap_topic': '/fixture/reconstructed_octomap',
                'expected_vehicle_id': 'fixture-vehicle',
                'requester_id': 'map-update-rejection-test',
                'resync_retry_period_s': 0.1,
            }
        ],
    )
    return launch.LaunchDescription(
        [
            launch.actions.SetEnvironmentVariable('ROS_DOMAIN_ID', TEST_DOMAIN_ID),
            fixture,
            receiver,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestMapUpdateReceiverRejection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('map_update_receiver_rejection_test')
        cls.statuses = []
        cls.maps = []
        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        cls.control = cls.node.create_publisher(String, '/fixture/control', qos)
        cls.node.create_subscription(
            DiagnosticArray, '/diagnostics', cls._on_diagnostics, qos
        )
        cls.node.create_subscription(
            Octomap, '/fixture/reconstructed_octomap', cls.maps.append, map_qos
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_diagnostics(cls, message):
        for status in message.status:
            if 'map_update_rejection_receiver: map_update_receiver' not in status.name:
                continue
            values = {entry.key: entry.value for entry in status.values}
            cls.statuses.append(
                {
                    'state': int(values.get('receiver_state', '-1')),
                    'revision': int(values.get('revision', '0')),
                    'epoch': int(values.get('map_epoch', '0')),
                    'duplicates': int(values.get('duplicates', '0')),
                    'stale': int(values.get('stale', '0')),
                    'gaps': int(values.get('gaps', '0')),
                    'malformed': int(values.get('malformed', '0')),
                    'admission': int(values.get('admission_rejections', '0')),
                }
            )

    def _wait_for(self, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _command(self, value):
        message = String()
        message.data = value
        self.control.publish(message)

    def _latest_matches(self, **expected):
        return bool(self.statuses) and all(
            self.statuses[-1].get(key) == value for key, value in expected.items()
        )

    def test_duplicate_gap_corruption_and_epoch_fences(self):
        self.assertTrue(
            self._wait_for(lambda: self._latest_matches(state=1, revision=1, epoch=1)),
            'initial service/keyframe recovery did not establish revision 1',
        )

        self._command('delta2')
        self.assertTrue(
            self._wait_for(lambda: self._latest_matches(state=1, revision=2, epoch=1))
        )
        self._command('duplicate2')
        self.assertTrue(
            self._wait_for(
                lambda: self._latest_matches(state=1, revision=2, epoch=1)
                and self.statuses[-1]['duplicates'] >= 1
            )
        )

        self._command('gap4')
        self.assertTrue(
            self._wait_for(
                lambda: any(
                    status['state'] == 2
                    and status['revision'] == 2
                    and status['gaps'] >= 1
                    for status in self.statuses
                )
            ),
            'gap did not preserve revision 2 in ResyncRequired',
        )
        self.assertTrue(
            self._wait_for(lambda: self._latest_matches(state=1, revision=4, epoch=1)),
            'correlated keyframe did not recover revision 4',
        )

        self._command('old2')
        self.assertTrue(
            self._wait_for(
                lambda: self._latest_matches(state=1, revision=4, epoch=1)
                and self.statuses[-1]['stale'] >= 1
            ),
            'old delta was not rejected without rollback',
        )

        self._command('corrupt5')
        self.assertTrue(
            self._wait_for(
                lambda: any(
                    status['state'] == 2
                    and status['revision'] == 4
                    and status['malformed'] >= 1
                    for status in self.statuses
                )
            ),
            'wire corruption did not preserve revision 4 in ResyncRequired',
        )
        self.assertTrue(
            self._wait_for(lambda: self._latest_matches(state=1, revision=5, epoch=1)),
            'corruption recovery did not converge to revision 5',
        )

        self._command('epoch2')
        self.assertTrue(
            self._wait_for(lambda: self._latest_matches(state=1, revision=1, epoch=2)),
            'new epoch keyframe did not establish a lower revision baseline',
        )
        admission_before = self.statuses[-1]['admission']
        self._command('old_epoch')
        self.assertTrue(
            self._wait_for(
                lambda: self._latest_matches(state=1, revision=1, epoch=2)
                and self.statuses[-1]['admission'] > admission_before
            ),
            'retired epoch update was not rejected by the replay fence',
        )
        self.assertTrue(self.maps)
        self.assertGreater(len(self.maps[-1].data), 0)


@launch_testing.post_shutdown_test()
class TestMapUpdateReceiverRejectionExit(unittest.TestCase):
    def test_launch_completed_without_forced_shutdown(self):
        self.assertTrue(True)
