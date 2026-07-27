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
        name="local_map_invalid_config_keep_alive",
    )
    invalid_minimum_count = launch_ros.actions.Node(
        package="perception_local_map",
        executable="perception_local_map_node",
        name="local_map_invalid_minimum_count",
        parameters=[{"minimum_lidar_count": 0}],
        output="screen",
    )
    return (
        launch.LaunchDescription(
            [
                keep_alive,
                invalid_minimum_count,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"invalid_minimum_count": invalid_minimum_count},
    )


class TestLocalMapInvalidConfig(unittest.TestCase):
    def test_zero_minimum_count_fails_startup(
        self,
        proc_info,
        proc_output,
        invalid_minimum_count,
    ):
        proc_output.assertWaitFor(
            "lidar requirement counts must be positive",
            process=invalid_minimum_count,
            timeout=10,
        )
        proc_info.assertWaitForShutdown(process=invalid_minimum_count, timeout=10)


@launch_testing.post_shutdown_test()
class TestLocalMapInvalidConfigExitCode(unittest.TestCase):
    def test_process_exits_with_failure(self, proc_info, invalid_minimum_count):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=invalid_minimum_count,
            allowable_exit_codes=[1],
        )
