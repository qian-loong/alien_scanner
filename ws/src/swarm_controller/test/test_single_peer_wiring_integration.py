import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from swarm_controller_interfaces.msg import ExplorationTask


def generate_test_description():
    explorer = launch_ros.actions.Node(
        package='swarm_controller',
        executable='single_drone_explorer',
        name='single_drone_explorer',
        namespace='subject',
        parameters=[{
            'map_frame': 'map',
            'control_rate': 20.0,
            'peer_namespaces': ['peer'],
            'motion.timeout': 0.4,
            'peer.position_timeout': 0.3,
            'peer.goal_timeout': 0.6,
            'peer.retry_interval': 0.1,
        }],
    )
    delayed_peer_tf = launch.actions.TimerAction(
        period=1.0,
        actions=[launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='peer_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'peer/odom'],
        )],
    )
    return (
        launch.LaunchDescription([
            explorer,
            delayed_peer_tf,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'explorer': explorer},
    )


class TestSinglePeerWiringIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_single_peer_wiring')
        self.self_odom_pub = self.node.create_publisher(Odometry, '/subject/odom', 10)
        self.peer_odom_pub = self.node.create_publisher(Odometry, '/peer/odom', 10)
        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.peer_goal_pub = self.node.create_publisher(
            PoseStamped, '/peer/motion_goal', transient_qos)
        self.task_pub = self.node.create_publisher(
            ExplorationTask, '/subject/exploration_task', transient_qos)
        self.diagnostics = []
        self.node.create_subscription(
            DiagnosticArray,
            '/subject/exploration_diagnostics',
            self.diagnostics.append,
            transient_qos,
        )

    def tearDown(self):
        self.node.destroy_node()

    def _odom(self, frame, x=0.0):
        message = Odometry()
        message.header.frame_id = frame
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.pose.position.x = x
        message.pose.pose.orientation.w = 1.0
        return message

    def _goal(self, x):
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.position.x = x
        message.pose.orientation.w = 1.0
        return message

    def _task(self, frame):
        message = ExplorationTask()
        message.header.frame_id = frame
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.allocator_epoch = 1
        message.revision = 1
        message.task_id = 1
        message.mode = ExplorationTask.MODE_ASSIGNED
        message.target.position.x = 2.0
        message.target.orientation.w = 1.0
        message.lease.sec = 2
        return message

    def _latest_values(self):
        if not self.diagnostics or not self.diagnostics[-1].status:
            return {}
        return {
            item.key: item.value
            for item in self.diagnostics[-1].status[0].values
        }

    def _drive_until(self, predicate, timeout_sec, publish_peer_odom=True):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            self.self_odom_pub.publish(self._odom('map'))
            if publish_peer_odom:
                self.peer_odom_pub.publish(self._odom('peer/odom'))
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate(self._latest_values()):
                return True
        return False

    def test_task_frame_contract(self):
        self.task_pub.publish(self._task('peer/odom'))
        self.assertTrue(
            self._drive_until(
                lambda values: values.get('task_update_status')
                == 'RejectedInvalidFrame',
                timeout_sec=1.0,
            ),
            f'non-map task must be rejected: {self._latest_values()}',
        )
        self.assertEqual(self._latest_values().get('task_valid'), '0')

    def test_tf_recovery_and_independent_peer_expiry(self):
        self.peer_goal_pub.publish(self._goal(2.0))
        self.assertTrue(
            self._drive_until(
                lambda values: int(values.get('tf_pending', '0')) > 0,
                timeout_sec=0.8,
            ),
            'peer odom should remain pending before delayed static TF arrives',
        )

        self.assertTrue(
            self._drive_until(
                lambda values: values.get('fresh_peer_positions') == '1',
                timeout_sec=4.0,
            ),
            f'peer position should recover after static TF arrives: '
            f'{self._latest_values()}',
        )
        self.peer_goal_pub.publish(self._goal(2.0))
        self.assertTrue(
            self._drive_until(
                lambda values: values.get('active_peer_goals') == '1',
                timeout_sec=0.5,
            ),
            f'fresh far goal should become active: {self._latest_values()}',
        )

        self.assertTrue(
            self._drive_until(
                lambda values: values.get('fresh_peer_positions') == '1'
                and values.get('stale_peer_goals') == '1'
                and values.get('active_peer_goals') == '0',
                timeout_sec=1.2,
            ),
            'stale goal should release hard separation while odom stays fresh',
        )

        self.peer_goal_pub.publish(self._goal(0.1))
        self.assertTrue(
            self._drive_until(
                lambda values: values.get('fresh_peer_goals') == '1'
                and values.get('active_peer_goals') == '0',
                timeout_sec=0.5,
            ),
            'hold goal should not be active',
        )

        self.peer_goal_pub.publish(self._goal(2.0))
        self.assertTrue(
            self._drive_until(
                lambda values: values.get('active_peer_goals') == '1',
                timeout_sec=0.5,
            ),
            'new far goal should reactivate hard separation',
        )
        self.assertTrue(
            self._drive_until(
                lambda values: values.get('stale_peer_positions') == '1'
                and values.get('active_peer_goals') == '0',
                timeout_sec=0.8,
                publish_peer_odom=False,
            ),
            'stale position should remove both soft and hard peer effects',
        )


@launch_testing.post_shutdown_test()
class TestSinglePeerWiringShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
