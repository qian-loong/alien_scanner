import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener


def generate_test_description():
    return (
        launch.LaunchDescription([
            launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            ),
            launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_lidar',
                arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link'],
            ),
            launch_ros.actions.Node(
                package='drone_scanner',
                executable='fake_odom',
                name='fake_odom',
                namespace='drone_0',
                output='screen',
                parameters=[{
                    'line.start_x': 0.0,
                    'line.start_y': 0.0,
                    'line.start_z': 1.5,
                    'line.end_x': 10.0,
                    'line.end_y': 0.0,
                    'line.end_z': 1.5,
                    'line.duration_seconds': 20.0,
                    'publish_rate': 20.0,
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                }],
            ),
            launch_ros.actions.Node(
                package='drone_scanner',
                executable='fake_lidar',
                name='fake_lidar',
                namespace='drone_0',
                output='screen',
                parameters=[{
                    'cave_mode': 'tree',
                    'seed': 42,
                    'tree.loop_bulge': 12.0,
                    'tree.loop_direct_length': 16.0,
                    'density': 200,
                    'map_frame': 'map',
                    'lidar_frame': 'lidar_link',
                    'scan_rate': 10.0,
                    'num_beams': 180,
                    'max_range': 30.0,
                    'range_noise_std': 0.0,
                    'noise_seed': 0,
                }],
            ),
            launch.actions.TimerAction(
                period=2.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {},
    )


class TestFakeLidarIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_fake_lidar')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def tearDown(self):
        self.node.destroy_node()

    def _collect_points(self, seconds):
        return self._collect_topic('/drone_0/points', seconds)

    def _collect_scan_returns(self, seconds):
        return self._collect_topic('/drone_0/scan_returns', seconds)

    def _collect_topic(self, topic, seconds):
        msgs = []
        sub = self.node.create_subscription(
            PointCloud2,
            topic,
            lambda msg: msgs.append(msg),
            qos_profile_sensor_data,
        )
        try:
            discovery_deadline = time.time() + 2.0
            while time.time() < discovery_deadline and sub.get_publisher_count() == 0:
                rclpy.spin_once(self.node, timeout_sec=0.05)
            end_time = time.time() + seconds
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        finally:
            self.node.destroy_subscription(sub)
        return msgs

    def _wait_for_tf(self, parent, child, timeout_sec=8.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            try:
                return self.tf_buffer.lookup_transform(
                    parent,
                    child,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5),
                )
            except Exception:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        self.fail(f'TF {parent} -> {child} not available within {timeout_sec}s')

    def test_receives_scan_points(self, proc_output):
        msgs = self._collect_points(4.0)
        self.assertGreater(len(msgs), 0, 'expected /drone_0/points messages')
        cloud = msgs[-1]
        self.assertEqual(cloud.header.frame_id, 'lidar_link')
        self.assertGreater(cloud.width * cloud.height, 0)

    def test_stable_beam_count(self, proc_output):
        msgs = self._collect_points(3.0)
        self.assertGreaterEqual(len(msgs), 2, 'need multiple scan frames')
        count_a = msgs[-2].width * msgs[-2].height
        count_b = msgs[-1].width * msgs[-1].height
        self.assertEqual(count_a, count_b, 'hit count should be stable between frames')
        self.assertGreater(count_b, 0, 'expected at least one ray hit per frame')

    def test_tf_map_to_lidar_link(self, proc_output):
        transform = self._wait_for_tf('map', 'lidar_link')
        self.assertEqual(transform.header.frame_id, 'map')
        self.assertEqual(transform.child_frame_id, 'lidar_link')

    def test_tf_tree_complete(self, proc_output):
        for parent, child in (
            ('map', 'odom'),
            ('odom', 'base_link'),
            ('base_link', 'lidar_link'),
        ):
            transform = self._wait_for_tf(parent, child)
            self.assertEqual(transform.header.frame_id, parent)
            self.assertEqual(transform.child_frame_id, child)

    def test_scan_rate(self, proc_output):
        msgs = self._collect_points(2.0)
        self.assertGreaterEqual(len(msgs), 12, f'expected ~20 scans in 2s, got {len(msgs)}')
        self.assertLessEqual(len(msgs), 30)

    def test_scan_returns_full_beam_contract(self, proc_output):
        msgs = self._collect_scan_returns(4.0)
        self.assertGreater(len(msgs), 0, 'expected /drone_0/scan_returns messages')
        cloud = msgs[-1]
        self.assertEqual(cloud.header.frame_id, 'lidar_link')
        self.assertEqual(cloud.width * cloud.height, 180)
        fields = {field.name: field for field in cloud.fields}
        expected = {
            'x': PointField.FLOAT32,
            'y': PointField.FLOAT32,
            'z': PointField.FLOAT32,
            'range': PointField.FLOAT32,
            'hit': PointField.UINT8,
            'intensity': PointField.FLOAT32,
        }
        for name, datatype in expected.items():
            self.assertIn(name, fields)
            self.assertEqual(fields[name].datatype, datatype)
            self.assertEqual(fields[name].count, 1)


@launch_testing.post_shutdown_test()
class TestFakeLidarShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
