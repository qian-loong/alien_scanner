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


TRAJECTORY_DURATION_SEC = 5.0
READY_DELAY_SEC = 2.0


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
                    'line.end_x': 8.0,
                    'line.end_y': 0.0,
                    'line.end_z': 1.5,
                    'line.duration_seconds': TRAJECTORY_DURATION_SEC,
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
                    'stop_scan_when_trajectory_done': True,
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
                period=READY_DELAY_SEC,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {},
    )


class TestStopScanIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_stop_scan')

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds):
        end_time = time.time() + seconds
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def test_scan_and_cloud_map_stop_after_trajectory_end(self, proc_output):
        scan_count = {'n': 0}
        cloud_sizes = []

        scan_sub = self.node.create_subscription(
            PointCloud2,
            '/drone_0/points',
            lambda _msg: scan_count.__setitem__('n', scan_count['n'] + 1),
            qos_profile_sensor_data,
        )
        map_sub = self.node.create_subscription(
            PointCloud2,
            '/drone_0/cloud_map',
            lambda msg: cloud_sizes.append(msg.width * msg.height),
            10,
        )
        try:
            # Collect during flight (before trajectory ends at TRAJECTORY_DURATION_SEC).
            self._spin_for(2.0)
            scans_during_flight = scan_count['n']
            self.assertGreater(
                scans_during_flight, 0,
                'expected scans while drone is moving',
            )

            # Wait until after trajectory end, then verify scan silence.
            wait_past_end = TRAJECTORY_DURATION_SEC - READY_DELAY_SEC - 2.0 + 1.5
            self.assertGreater(wait_past_end, 0.0)
            self._spin_for(wait_past_end)
            scans_at_stop = scan_count['n']
            self._spin_for(3.0)
            self.assertEqual(
                scan_count['n'], scans_at_stop,
                'fake_lidar should stop publishing /drone_0/points after trajectory ends',
            )

            self.assertGreater(len(cloud_sizes), 0, 'expected cloud_map during flight')
            size_at_stop = cloud_sizes[-1]
            self._spin_for(2.0)
            for size in cloud_sizes[-3:]:
                self.assertEqual(
                    size, size_at_stop,
                    'cloud_map point count should not grow after fake_lidar stops',
                )
        finally:
            self.node.destroy_subscription(scan_sub)
            self.node.destroy_subscription(map_sub)


@launch_testing.post_shutdown_test()
class TestStopScanShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
