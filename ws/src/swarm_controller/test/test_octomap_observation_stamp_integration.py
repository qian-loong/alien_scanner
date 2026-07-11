import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from builtin_interfaces.msg import Time
from octomap_msgs.msg import Octomap
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def generate_test_description():
    return (
        launch.LaunchDescription([
            launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_lidar',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'lidar_link'],
            ),
            launch_ros.actions.Node(
                package='swarm_controller',
                executable='octomap_builder',
                name='octomap_builder',
                namespace='stamp_test',
                output='screen',
                parameters=[{
                    'map_frame': 'map',
                    'input_topic': 'scan_returns',
                    'output_topic': 'octomap',
                    'resolution': 0.1,
                    'publish_rate': 10.0,
                    'max_range': 5.0,
                }],
            ),
            launch.actions.TimerAction(
                period=1.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {},
    )


class TestOctomapObservationStamp(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_octomap_observation_stamp')
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.node.create_publisher(
            PointCloud2,
            '/stamp_test/scan_returns',
            scan_qos,
        )
        self.maps = []
        self.subscription = self.node.create_subscription(
            Octomap,
            '/stamp_test/octomap',
            self.maps.append,
            map_qos,
        )

    def tearDown(self):
        self.node.destroy_node()

    @staticmethod
    def _scan(seconds):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='range', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='hit', offset=16, datatype=PointField.UINT8, count=1),
            PointField(name='intensity', offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        header = Header()
        # 本测试只验证 observation stamp 契约；使用 map frame 避免把 TF
        # discovery 时序引入无关断言。
        header.frame_id = 'map'
        header.stamp = Time(sec=seconds)
        return point_cloud2.create_cloud(
            header,
            fields,
            [(1.0, 0.0, 0.0, 1.0, 1, 1.0)],
        )

    def _spin_until(self, predicate, timeout=2.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not predicate():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_only_strictly_increasing_nonzero_scan_stamps_publish(self):
        self._spin_until(
            lambda: self.publisher.get_subscription_count() > 0,
            timeout=3.0,
        )
        self.assertGreater(self.publisher.get_subscription_count(), 0)

        for _ in range(20):
            self.publisher.publish(self._scan(10))
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.maps:
                break
        self._spin_until(lambda: len(self.maps) >= 1)
        self.assertEqual(len(self.maps), 1)
        self.assertEqual(self.maps[-1].header.stamp.sec, 10)

        for stamp in (0, 10, 9):
            self.publisher.publish(self._scan(stamp))
            self._spin_until(lambda: False, timeout=0.25)
        self.assertEqual(len(self.maps), 1)

        deadline = time.time() + 2.0
        while time.time() < deadline and len(self.maps) < 2:
            self.publisher.publish(self._scan(11))
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(len(self.maps), 2)
        self.assertEqual(self.maps[-1].header.stamp.sec, 11)


@launch_testing.post_shutdown_test()
class TestOctomapObservationStampShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
