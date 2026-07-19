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
from std_msgs.msg import Empty


def generate_test_description():
    probe = launch_ros.actions.Node(
        package='swarm_controller',
        executable='global_task_allocator_integration_probe',
        name='global_task_allocator_pipeline_probe',
        output='screen',
        parameters=[{
            'controlled_global_updates': True,
            'diagnostics_reject_after_seconds': -1.0,
        }],
    )
    allocator = launch_ros.actions.Node(
        package='swarm_controller',
        executable='global_task_allocator',
        name='global_task_allocator_pipeline',
        output='screen',
        parameters=[{
            'map_frame': 'map',
            'drone_namespaces': ['drone_0', 'drone_1', 'drone_2'],
            'task_allocation.rate': 4.0,
            'task.lease': 1.0,
            'global_map.stale_timeout': 5.0,
            'global_map.diagnostics_timeout': 2.0,
            'drone.odom_timeout': 2.0,
            'drone.local_map_timeout': 5.0,
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
            'frontier.missed_update_grace': 1,
            'allocation.first_hop_distance': 0.5,
            'allocation.activation_updates': 1,
            'allocation.activation_time': 0.0,
            'allocation.deactivation_grace': 0.0,
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


class TestGlobalTaskAllocatorPipeline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_global_task_allocator_pipeline')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.diagnostics = []
        self.subscription = self.node.create_subscription(
            DiagnosticArray,
            '/global_task_diagnostics',
            self.diagnostics.append,
            qos,
        )
        self.invalid_global = self.node.create_publisher(
            Empty,
            '/test/global_task_allocator/publish_invalid_global_map',
            1,
        )
        self.recover_global = self.node.create_publisher(
            Empty,
            '/test/global_task_allocator/recover_global_map',
            1,
        )
        self.invalid_local = self.node.create_publisher(
            Empty,
            '/test/global_task_allocator/publish_invalid_local_map',
            1,
        )
        self.recover_local = self.node.create_publisher(
            Empty,
            '/test/global_task_allocator/recover_local_map',
            1,
        )
        self.invalid_stamps = self.node.create_publisher(
            Empty,
            '/test/global_task_allocator/publish_invalid_stamp_inputs',
            1,
        )
        self.future_global = self.node.create_publisher(
            Empty,
            '/test/global_task_allocator/publish_future_global_map',
            1,
        )

    def tearDown(self):
        self.node.destroy_node()

    def _spin_until(self, predicate, timeout=10.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not predicate():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return predicate()

    def _values(self):
        if not self.diagnostics:
            return {}
        statuses = [
            status for status in self.diagnostics[-1].status
            if status.name == 'global_task_allocator'
        ]
        if not statuses:
            return {}
        return {item.key: item.value for item in statuses[-1].values}

    def _publish_when_connected(self, publisher, timeout=3.0):
        self._spin_until(lambda: publisher.get_subscription_count() > 0, timeout)
        self.assertGreater(publisher.get_subscription_count(), 0)
        publisher.publish(Empty())

    def test_invalid_results_preserve_state_and_recover(self):
        self.assertTrue(self._spin_until(
            lambda: self._values().get('global_update_sequence') == '1'
            and int(self._values().get('global_map_applied_revision', '0')) > 0
            and int(self._values().get('drone_0.local_map_applied_revision', '0')) > 0,
        ))
        initial = self._values()
        initial_sequence = int(initial['global_update_sequence'])
        initial_global_revision = int(initial['global_map_applied_revision'])
        initial_global_age = float(initial['global_map_valid_age_seconds'])

        self._publish_when_connected(self.invalid_stamps)
        self.assertTrue(self._spin_until(
            lambda: int(self._values().get('global_map_invalid_envelope', '0')) >= 1
            and int(self._values().get(
                'global_map_diagnostics_invalid_stamp', '0')) >= 1
            and int(self._values().get('drone_0.local_map_invalid_envelope', '0')) >= 1
            and int(self._values().get('drone_0.odom_invalid_stamp', '0')) >= 1,
        ))

        self._publish_when_connected(self.future_global)
        self.assertTrue(self._spin_until(
            lambda: int(self._values().get('global_map_invalid_envelope', '0')) >= 2
            and self._values().get('global_map_input_reason')
            == 'observation stamp is in the future',
        ))

        self._publish_when_connected(self.invalid_global)
        self.assertTrue(self._spin_until(
            lambda: int(self._values().get('global_map_processing_failures', '0')) >= 1
            and int(self._values().get('global_map_consumed_revision', '0'))
            > int(self._values().get('global_map_applied_revision', '0'))
            and bool(self._values().get('global_map_input_reason')),
        ))
        failed_global = self._values()
        self.assertEqual(int(failed_global['global_update_sequence']), initial_sequence)
        self.assertEqual(
            int(failed_global['global_map_applied_revision']),
            initial_global_revision,
        )
        self.assertGreater(
            float(failed_global['global_map_valid_age_seconds']),
            initial_global_age,
        )

        self._publish_when_connected(self.recover_global)
        self.assertTrue(self._spin_until(
            lambda: int(self._values().get('global_update_sequence', '0'))
            == initial_sequence + 1
            and not self._values().get('global_map_input_reason')
            and self._values().get('global_map_fresh') == '1',
        ))
        recovered_global = self._values()
        self.assertGreater(
            int(recovered_global['global_map_applied_revision']),
            initial_global_revision,
        )
        self.assertLess(
            float(recovered_global['global_map_valid_age_seconds']),
            float(failed_global['global_map_valid_age_seconds']),
        )

        self._publish_when_connected(self.invalid_local)
        self.assertTrue(self._spin_until(
            lambda: int(self._values().get('drone_0.local_map_failures', '0')) >= 1
            and int(self._values().get('drone_0.local_map_consumed_revision', '0'))
            > int(self._values().get('drone_0.local_map_applied_revision', '0'))
            and bool(self._values().get('drone_0.local_map_reason')),
        ))
        failed_local = self._values()
        failed_local_revision = int(failed_local['drone_0.local_map_applied_revision'])
        failed_local_age = float(failed_local['drone_0.local_map_valid_age_seconds'])
        self.assertEqual(failed_local['drone_0.local_map_fresh'], '1')
        self.assertTrue(self._spin_until(
            lambda: float(self._values().get(
                'drone_0.local_map_valid_age_seconds', '0'))
            > failed_local_age + 0.3,
            timeout=1.5,
        ))
        aged_local = self._values()
        self.assertEqual(
            int(aged_local['drone_0.local_map_applied_revision']),
            failed_local_revision,
        )

        self._publish_when_connected(self.recover_local)
        self.assertTrue(self._spin_until(
            lambda: int(self._values().get('drone_0.local_map_applied_revision', '0'))
            > failed_local_revision
            and not self._values().get('drone_0.local_map_reason')
            and self._values().get('drone_0.local_map_fresh') == '1',
        ))


@launch_testing.post_shutdown_test()
class TestGlobalTaskAllocatorPipelineShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info, probe, allocator):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=probe, allowable_exit_codes=[0, -15])
        launch_testing.asserts.assertExitCodes(proc_info, process=allocator)
