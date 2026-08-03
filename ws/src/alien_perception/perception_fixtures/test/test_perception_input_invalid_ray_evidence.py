import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.asserts
import pytest


@pytest.mark.launch_test
def generate_test_description():
    keep_alive = launch.actions.ExecuteProcess(
        cmd=["python3", "-c", "import time; time.sleep(30)"],
        name="invalid_ray_evidence_test_keep_alive",
    )
    invalid_contract = launch_ros.actions.Node(
        package="perception_input_node",
        executable="perception_input_node",
        name="invalid_contract_ray_evidence",
        parameters=[{"minimum_lidar_ray_evidence": "invalid"}],
        output="screen",
    )
    invalid_descriptor = launch_ros.actions.Node(
        package="perception_input_node",
        executable="perception_input_node",
        name="invalid_descriptor_ray_evidence",
        parameters=[
            {
                "sensor_ids": ["front"],
                "sensor.front.ray_evidence": "invalid",
            }
        ],
        output="screen",
    )
    invalid_degraded = launch_ros.actions.Node(
        package="perception_input_node",
        executable="perception_input_node",
        name="invalid_degraded_ray_evidence",
        parameters=[{"degraded_lidar_ray_evidence": "invalid"}],
        output="screen",
    )
    return (
        launch.LaunchDescription(
            [
                keep_alive,
                invalid_contract,
                invalid_descriptor,
                invalid_degraded,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "invalid_contract": invalid_contract,
            "invalid_descriptor": invalid_descriptor,
            "invalid_degraded": invalid_degraded,
        },
    )


class TestInvalidRayEvidence(unittest.TestCase):
    def test_invalid_values_fail_startup(
        self,
        proc_info,
        proc_output,
        invalid_contract,
        invalid_descriptor,
        invalid_degraded,
    ):
        proc_output.assertWaitFor(
            "Unsupported ray evidence 'invalid'",
            process=invalid_contract,
            timeout=10,
        )
        proc_output.assertWaitFor(
            "Unsupported ray evidence 'invalid'",
            process=invalid_descriptor,
            timeout=10,
        )
        proc_output.assertWaitFor(
            "Unsupported ray evidence 'invalid'",
            process=invalid_degraded,
            timeout=10,
        )
        proc_info.assertWaitForShutdown(process=invalid_contract, timeout=10)
        proc_info.assertWaitForShutdown(process=invalid_descriptor, timeout=10)
        proc_info.assertWaitForShutdown(process=invalid_degraded, timeout=10)


@launch_testing.post_shutdown_test()
class TestInvalidRayEvidenceExitCodes(unittest.TestCase):
    def test_processes_exit_with_failure(
        self,
        proc_info,
        invalid_contract,
        invalid_descriptor,
        invalid_degraded,
    ):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=invalid_contract,
            allowable_exit_codes=[1],
        )
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=invalid_descriptor,
            allowable_exit_codes=[1],
        )
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=invalid_degraded,
            allowable_exit_codes=[1],
        )
