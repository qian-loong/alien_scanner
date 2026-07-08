import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


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
                    'scan_rate': 10.0,
                    'num_beams': 180,
                }],
            ),
            launch_ros.actions.Node(
                package='drone_scanner',
                executable='scan_accumulator',
                name='scan_accumulator',
                namespace='drone_0',
                output='screen',
                parameters=[{
                    'map_frame': 'map',
                    'points_topic': 'points',
                    'cloud_map_topic': 'cloud_map',
                    'publish_rate': 5.0,
                    'max_points': 500000,
                }],
            ),
            launch.actions.TimerAction(
                period=3.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {},
    )


class TestCloudMapIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_cloud_map')

    def tearDown(self):
        self.node.destroy_node()

    def _collect_cloud_maps(self, seconds):
        msgs = []
        sub = self.node.create_subscription(
            PointCloud2,
            '/drone_0/cloud_map',
            lambda msg: msgs.append(msg),
            10,
        )
        try:
            end_time = time.time() + seconds
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        finally:
            self.node.destroy_subscription(sub)
        return msgs

    def test_receives_cloud_map_in_map_frame(self, proc_output):
        msgs = self._collect_cloud_maps(6.0)
        self.assertGreater(len(msgs), 0, 'expected /drone_0/cloud_map messages')
        cloud = msgs[-1]
        self.assertEqual(cloud.header.frame_id, 'map')
        self.assertGreater(cloud.width * cloud.height, 0)

    def test_cloud_map_grows_while_drone_moves(self, proc_output):
        msgs = self._collect_cloud_maps(10.0)
        self.assertGreaterEqual(len(msgs), 2, 'need multiple cloud_map updates')
        first_count = msgs[0].width * msgs[0].height
        last_count = msgs[-1].width * msgs[-1].height
        self.assertGreater(last_count, first_count, 'accumulated map should grow over time')
        self.assertGreater(last_count, first_count * 2, 'map should accumulate multiple scans')


@launch_testing.post_shutdown_test()
class TestCloudMapShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
