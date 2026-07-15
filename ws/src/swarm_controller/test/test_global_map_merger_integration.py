import copy
import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from octomap_msgs.msg import Octomap
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty, Header


def _builder(namespace):
    return launch_ros.actions.Node(
        package='swarm_controller',
        executable='octomap_builder',
        name='octomap_builder',
        namespace=namespace,
        output='screen',
        parameters=[{
            'map_frame': 'map',
            'input_topic': 'scan_returns',
            'output_topic': 'octomap',
            'resolution': 0.1,
            'publish_rate': 10.0,
            'max_range': 5.0,
        }],
    )


def generate_test_description():
    merger = launch_ros.actions.Node(
        package='swarm_controller',
        executable='global_map_merger',
        name='global_map_merger',
        output='screen',
        parameters=[{
            'map_frame': 'map',
            'source_topics': ['/source_a/octomap', '/source_b/octomap'],
            'output_topic': 'global_map',
            'diagnostics_topic': 'global_map_diagnostics',
            'resolution': 0.1,
            'merge_rate': 2.0,
            'source_stale_timeout': 0.5,
            'max_serialized_bytes_per_source': 4 * 1024 * 1024,
            'max_voxels_per_source': 100000,
            'max_global_voxels': 200000,
        }],
    )
    late_probe = launch_ros.actions.Node(
        package='swarm_controller',
        executable='global_map_late_subscriber_probe',
        name='global_map_late_subscriber_probe',
        output='screen',
    )
    return (
        launch.LaunchDescription([
            _builder('source_a'),
            _builder('source_b'),
            merger,
            late_probe,
            launch.actions.TimerAction(
                period=1.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {},
    )


class TestGlobalMapMergerIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_global_map_merger')
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.scan_a = self.node.create_publisher(
            PointCloud2, '/source_a/scan_returns', scan_qos)
        self.scan_b = self.node.create_publisher(
            PointCloud2, '/source_b/scan_returns', scan_qos)
        self.replay_a = self.node.create_publisher(
            Octomap, '/source_a/octomap', transient_qos)
        self.replay_b = self.node.create_publisher(
            Octomap, '/source_b/octomap', transient_qos)
        self.probe_start = self.node.create_publisher(
            Empty, '/test/start_global_map_late_probe', 1)
        self.source_maps = {'a': [], 'b': []}
        self.global_maps = []
        self.diagnostics = []
        self.probe_diagnostics = []
        self.subscriptions = [
            self.node.create_subscription(
                Octomap, '/source_a/octomap', self.source_maps['a'].append,
                transient_qos),
            self.node.create_subscription(
                Octomap, '/source_b/octomap', self.source_maps['b'].append,
                transient_qos),
            self.node.create_subscription(
                Octomap, '/global_map', self.global_maps.append, transient_qos),
            self.node.create_subscription(
                DiagnosticArray,
                '/global_map_diagnostics',
                self.diagnostics.append,
                transient_qos,
            ),
            self.node.create_subscription(
                DiagnosticArray,
                '/test/global_map_late_probe_diagnostics',
                self.probe_diagnostics.append,
                transient_qos,
            ),
        ]
        self.stamp_ns = self.node.get_clock().now().nanoseconds

    def tearDown(self):
        self.node.destroy_node()

    @staticmethod
    def _fields():
        return [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='range', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='hit', offset=16, datatype=PointField.UINT8, count=1),
            PointField(
                name='intensity', offset=20, datatype=PointField.FLOAT32, count=1),
        ]

    def _scan(self, endpoint):
        self.stamp_ns += 1_000_000
        header = Header()
        header.frame_id = 'map'
        header.stamp = rclpy.time.Time(nanoseconds=self.stamp_ns).to_msg()
        x, y, z = endpoint
        distance = (x * x + y * y + z * z) ** 0.5
        return point_cloud2.create_cloud(
            header,
            self._fields(),
            [(x, y, z, distance, 1, 1.0)],
        )

    def _spin_until(self, predicate, timeout=10.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not predicate():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    @staticmethod
    def _empty_map(stamp_ns):
        message = Octomap()
        message.header.frame_id = 'map'
        message.header.stamp = rclpy.time.Time(
            nanoseconds=stamp_ns).to_msg()
        message.binary = False
        message.id = 'OcTree'
        message.resolution = 0.1
        message.data = []
        return message

    def _values(self):
        if not self.diagnostics:
            return {}
        for status in self.diagnostics[-1].status:
            if status.name == 'global_map_merger':
                return {item.key: item.value for item in status.values}
        return {}

    def _probe_status(self):
        for message in reversed(self.probe_diagnostics):
            for status in message.status:
                if status.name == 'global_map_late_subscriber_probe':
                    return status
        return None

    def _publish_until_ready(self):
        deadline = time.time() + 12.0
        while time.time() < deadline:
            self.scan_a.publish(self._scan((1.0, 0.0, 0.0)))
            self.scan_b.publish(self._scan((0.0, 1.0, 0.0)))
            rclpy.spin_once(self.node, timeout_sec=0.1)
            values = self._values()
            if (self.source_maps['a'] and self.source_maps['b']
                    and self.global_maps
                    and values.get('accepted_sources') == '2'):
                return
        self.fail('merger did not accept both controlled source maps')

    def test_merge_coalescing_rejection_stale_and_latched_output(self):
        self._publish_until_ready()
        message = self.global_maps[-1]
        self.assertEqual(message.header.frame_id, 'map')
        self.assertEqual(message.id, 'OcTree')
        self.assertFalse(message.binary)
        self.assertTrue(message.data)

        previous_diagnostic_count = len(self.diagnostics)
        self._spin_until(
            lambda: len(self.diagnostics) > previous_diagnostic_count,
            timeout=2.0,
        )
        template = self.source_maps['a'][-1]
        template_stamp = rclpy.time.Time.from_msg(
            template.header.stamp).nanoseconds
        for offset in range(1, 6):
            replay = copy.deepcopy(template)
            replay.header.stamp = rclpy.time.Time(
                nanoseconds=template_stamp + offset).to_msg()
            self.replay_a.publish(replay)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self._spin_until(
            lambda: int(self._values().get('coalesced_updates', '0')) > 0,
            timeout=3.0,
        )
        self.assertGreater(
            int(self._values().get('coalesced_updates', '0')), 0)

        values = self._values()
        source_a_key = 'source_voxels:/source_a/octomap'
        source_b_key = 'source_voxels:/source_b/octomap'
        self.assertGreater(int(values.get(source_a_key, '0')), 0)
        self.assertGreater(int(values.get(source_b_key, '0')), 0)
        self.assertEqual(
            int(values['total_source_voxels']),
            int(values[source_a_key]) + int(values[source_b_key]),
        )
        for key in (
                'max_source_voxels',
                'max_serialized_bytes',
                'source_voxel_utilization',
                'global_voxel_utilization',
                'serialized_byte_utilization',
                'max_resource_utilization'):
            self.assertIn(key, values)

        rejected_before = int(self._values().get('rejected_updates', '0'))
        invalid = copy.deepcopy(template)
        invalid.header.frame_id = 'wrong_frame'
        invalid.header.stamp = rclpy.time.Time(
            nanoseconds=template_stamp + 100).to_msg()
        self.replay_a.publish(invalid)
        self._spin_until(
            lambda: int(self._values().get('rejected_updates', '0'))
            > rejected_before,
            timeout=3.0,
        )
        self.assertGreater(
            int(self._values().get('rejected_updates', '0')), rejected_before)

        deep_rejected_before = int(
            self._values().get('rejected_updates', '0'))
        unsupported_tree = copy.deepcopy(template)
        unsupported_tree.id = 'UnsupportedTree'
        unsupported_tree.header.stamp = rclpy.time.Time(
            nanoseconds=template_stamp + 1000).to_msg()
        self.replay_a.publish(unsupported_tree)
        self._spin_until(
            lambda: int(self._values().get('rejected_updates', '0'))
            > deep_rejected_before,
            timeout=3.0,
        )
        self.assertGreater(
            int(self._values().get('rejected_updates', '0')),
            deep_rejected_before,
        )

        accepted_before_recovery = int(
            self._values().get('accepted_updates', '0'))
        recovery = copy.deepcopy(template)
        recovery.header.stamp = rclpy.time.Time(
            nanoseconds=template_stamp + 500).to_msg()
        self.replay_a.publish(recovery)
        self._spin_until(
            lambda: int(self._values().get('accepted_updates', '0'))
            > accepted_before_recovery,
            timeout=3.0,
        )
        self.assertGreater(
            int(self._values().get('accepted_updates', '0')),
            accepted_before_recovery,
        )

        self._spin_until(
            lambda: int(self._values().get('stale_sources', '0')) == 2,
            timeout=3.0,
        )
        self.assertEqual(self._values().get('stale_sources'), '2')

        before_empty = self._values()
        accepted_before_empty = int(before_empty['accepted_updates'])
        revision_before_empty = int(before_empty['global_revision'])
        known_before_empty = int(before_empty['known_voxels'])
        global_map_count_before_empty = len(self.global_maps)
        self.assertGreater(known_before_empty, 0)

        template_b = self.source_maps['b'][-1]
        template_b_stamp = rclpy.time.Time.from_msg(
            template_b.header.stamp).nanoseconds
        empty_a_stamp = template_stamp + 2000
        empty_b_stamp = template_b_stamp + 2000
        self.replay_a.publish(self._empty_map(empty_a_stamp))
        self.replay_b.publish(self._empty_map(empty_b_stamp))
        self._spin_until(
            lambda: (
                int(self._values().get('accepted_updates', '0'))
                >= accepted_before_empty + 2
                and self._values().get(source_a_key) == '0'
                and self._values().get(source_b_key) == '0'
                and self._values().get('known_voxels') == '0'
                and len(self.global_maps) > global_map_count_before_empty
            ),
            timeout=4.0,
        )
        after_empty = self._values()
        self.assertGreaterEqual(
            int(after_empty['accepted_updates']), accepted_before_empty + 2)
        self.assertEqual(after_empty[source_a_key], '0')
        self.assertEqual(after_empty[source_b_key], '0')
        self.assertEqual(after_empty['total_source_voxels'], '0')
        self.assertEqual(after_empty['known_voxels'], '0')
        self.assertGreater(
            int(after_empty['global_revision']), revision_before_empty)
        self.assertGreater(len(self.global_maps), global_map_count_before_empty)
        empty_global_data = list(self.global_maps[-1].data)
        self.assertEqual(
            rclpy.time.Time.from_msg(
                self.global_maps[-1].header.stamp).nanoseconds,
            max(empty_a_stamp, empty_b_stamp),
        )

        self._spin_until(
            lambda: self.probe_start.get_subscription_count() > 0,
            timeout=2.0,
        )
        self.assertGreater(self.probe_start.get_subscription_count(), 0)
        self.probe_start.publish(Empty())
        self._spin_until(lambda: self._probe_status() is not None, timeout=3.0)
        probe_status = self._probe_status()
        self.assertIsNotNone(probe_status)
        self.assertEqual(probe_status.level, DiagnosticStatus.OK)
        probe_values = {item.key: item.value for item in probe_status.values}
        self.assertEqual(probe_values.get('message_id'), 'OcTree')
        self.assertEqual(probe_values.get('binary'), 'false')
        self.assertEqual(probe_values.get('leaf_count'), '0')

        late_maps = []
        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        late_subscription = self.node.create_subscription(
            Octomap, '/global_map', late_maps.append, transient_qos)
        self._spin_until(lambda: bool(late_maps), timeout=2.0)
        self.assertTrue(late_maps)
        self.assertEqual(late_maps[-1].id, 'OcTree')
        self.assertFalse(late_maps[-1].binary)
        self.assertEqual(list(late_maps[-1].data), empty_global_data)
        self.node.destroy_subscription(late_subscription)


@launch_testing.post_shutdown_test()
class TestGlobalMapMergerShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
