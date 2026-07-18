import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from swarm_controller_interfaces.msg import ExplorationTask


def generate_test_description():
    probe = launch_ros.actions.Node(
        package='swarm_controller',
        executable='global_task_allocator_integration_probe',
        name='stale_global_task_allocator_probe',
        output='screen',
        parameters=[{'stamp_offset_seconds': 10.0}],
    )
    allocator = launch_ros.actions.Node(
        package='swarm_controller',
        executable='global_task_allocator',
        name='stale_global_task_allocator',
        output='screen',
        parameters=[{
            'map_frame': 'map',
            'drone_namespaces': ['drone_0', 'drone_1', 'drone_2'],
            'task_allocation.rate': 4.0,
            'task.lease': 1.0,
            'global_map.stale_timeout': 1.0,
            'global_map.diagnostics_timeout': 1.0,
            'drone.odom_timeout': 1.0,
            'drone.local_map_timeout': 1.0,
            'resolution': 0.1,
            'frontier.column_stride_voxels': 1,
            'frontier.min_z_layers': 5,
            'frontier.min_z_span': 0.4,
            'frontier.support_depth': 0.2,
            'frontier.min_columns': 4,
            'frontier.min_area': 0.05,
            'frontier.min_span': 0.3,
            'frontier.min_direction_consistency': 0.3,
            'frontier.min_persistence_updates': 1,
            'frontier.min_persistence_time': 0.0,
            'allocation.activation_updates': 1,
            'allocation.activation_time': 0.0,
            'body.robot_radius': 0.05,
            'body.robot_half_height': 0.05,
            'body.safety_margin': 0.05,
            'body.vertical_margin': 0.05,
        }],
    )
    return (
        launch.LaunchDescription([
            probe,
            allocator,
            launch.actions.TimerAction(
                period=1.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {'probe': probe, 'allocator': allocator},
    )


class TestStaleGlobalTaskAllocatorIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_stale_global_task_allocator')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.tasks = {index: [] for index in range(3)}
        self.subscriptions = [
            self.node.create_subscription(
                ExplorationTask,
                '/drone_{}/exploration_task'.format(index),
                self.tasks[index].append,
                qos,
            )
            for index in range(3)
        ]
        self.diagnostics = []
        self.subscriptions.append(self.node.create_subscription(
            DiagnosticArray,
            '/global_task_diagnostics',
            self.diagnostics.append,
            qos,
        ))

    def tearDown(self):
        self.node.destroy_node()

    def _spin_until(self, predicate, timeout=8.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not predicate():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_old_latched_inputs_never_activate_coordination(self):
        self._spin_until(
            lambda: all(self.tasks[index] for index in range(3))
            and bool(self.diagnostics),
        )
        self.assertTrue(all(self.tasks[index] for index in range(3)))
        self.assertTrue(all(
            self.tasks[index][-1].mode == ExplorationTask.MODE_LOCAL_FALLBACK
            for index in range(3)
        ))

        statuses = [
            status for message in self.diagnostics
            for status in message.status
            if status.name == 'global_task_allocator'
        ]
        self.assertTrue(statuses)
        values = {item.key: item.value for item in statuses[-1].values}
        self.assertGreater(int(values['global_update_sequence']), 0)
        self.assertEqual(values['global_map_fresh'], '0')
        self.assertEqual(values['global_map_diagnostics_healthy'], '0')


@launch_testing.post_shutdown_test()
class TestStaleGlobalTaskAllocatorShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info, probe, allocator):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=probe, allowable_exit_codes=[0, -15])
        launch_testing.asserts.assertExitCodes(
            proc_info, process=allocator, allowable_exit_codes=[0, -15])
