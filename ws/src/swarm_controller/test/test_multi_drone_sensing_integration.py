import time
import unittest

import launch
import launch.actions
import launch.launch_description_sources
import launch_ros.substitutions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from nav_msgs.msg import Odometry
from octomap_msgs.msg import Octomap
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


def generate_test_description():
    sensing_launch = launch.substitutions.PathJoinSubstitution([
        launch_ros.substitutions.FindPackageShare('swarm_controller'),
        'launch',
        'multi_drone_sensing.launch.py',
    ])
    included = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            sensing_launch
        ),
        launch_arguments={
            'num_drones': '3',
            'show_rviz': 'false',
            'show_cave_truth': 'false',
            'enable_scan_accumulator': 'false',
        }.items(),
    )
    return (
        launch.LaunchDescription([
            included,
            launch.actions.TimerAction(
                period=2.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {},
    )


class TestMultiDroneSensingIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_multi_drone_sensing')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.odoms = {i: [] for i in range(3)}
        self.scans = {i: [] for i in range(3)}
        self.maps = {i: [] for i in range(3)}
        for i in range(3):
            ns = f'/drone_{i}'
            self.node.create_subscription(
                Odometry, f'{ns}/odom', self.odoms[i].append, 10)
            self.node.create_subscription(
                PointCloud2,
                f'{ns}/scan_returns',
                self.scans[i].append,
                qos_profile_sensor_data,
            )
            self.node.create_subscription(
                Octomap, f'{ns}/octomap', self.maps[i].append, transient_qos)

    def tearDown(self):
        self.node.destroy_node()

    def _spin_until(self, predicate, timeout_sec):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not predicate():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def test_three_drones_publish_odom_scan_and_octomap(self):
        self._spin_until(
            lambda: all(
                self.odoms[i] and self.scans[i] and self.maps[i] for i in range(3)
            ),
            timeout_sec=25.0,
        )
        for i in range(3):
            self.assertTrue(self.odoms[i], f'drone_{i} odom missing')
            self.assertTrue(self.scans[i], f'drone_{i} scan_returns missing')
            self.assertTrue(self.maps[i], f'drone_{i} octomap missing')

        ys = [self.odoms[i][-1].pose.pose.position.y for i in range(3)]
        self.assertGreater(max(ys) - min(ys), 0.5, 'expected staggered start Y')

    def test_per_drone_tf_chains(self):
        self._spin_until(lambda: all(self.odoms[i] for i in range(3)), 15.0)
        deadline = time.time() + 10.0
        missing = set(range(3))
        while time.time() < deadline and missing:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            stamp = rclpy.time.Time()
            resolved = set()
            for i in missing:
                ns = f'drone_{i}'
                try:
                    self.tf_buffer.lookup_transform('map', f'{ns}/odom', stamp)
                    self.tf_buffer.lookup_transform(
                        f'{ns}/odom', f'{ns}/base_link', stamp)
                    self.tf_buffer.lookup_transform(
                        f'{ns}/base_link', f'{ns}/lidar_link', stamp)
                    resolved.add(i)
                except TransformException:
                    pass
            missing -= resolved
        self.assertFalse(missing, f'TF chain missing for drones {sorted(missing)}')


@launch_testing.post_shutdown_test()
class TestMultiDroneSensingShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
