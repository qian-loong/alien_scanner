import importlib.util
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).with_name("analyze-perception-profile.py")
RUNNER_PATH = Path(__file__).with_name("profile-perception.sh")
REPORT_PARSER_PATH = SCRIPT_PATH.parent / "lib" / "profile_report_parsers.py"
ROLE_MONITOR_PATH = SCRIPT_PATH.parent / "lib" / "profile_role_monitor.py"
SHELL_COMMON_PATH = SCRIPT_PATH.parent / "lib" / "profile-runner-common.sh"
if str(SCRIPT_PATH.parent) not in sys.path:
    sys.path.insert(0, str(SCRIPT_PATH.parent))

from lib import profile_analysis as COMMON_ANALYSIS
from lib import profile_report_parsers as REPORT_PARSERS
from lib import profile_role_monitor as ROLE_MONITOR

SPEC = importlib.util.spec_from_file_location("analyze_perception_profile", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
ANALYZER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYZER)


class AnalyzePerceptionProfileTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory(
            dir=SCRIPT_PATH.parent.parent
        )
        self.addCleanup(self.temporary_directory.cleanup)
        self.root = Path(self.temporary_directory.name)

    def make_run(self, name: str, *, pidstat_samples: int = 300) -> Path:
        run_dir = self.root / name
        run_dir.mkdir()
        tracee_pid = 4242
        t0_ns = 1_000_000_000_000
        duration_ns = 300_100_000_000
        t1_ns = t0_ns + duration_ns
        (run_dir / "run-manifest.txt").write_text(
            "\n".join(
                (
                    "mode=plain-sample",
                    "duration_requested_s=300",
                    f"tracee_pid={tracee_pid}",
                    f"t0_monotonic_ns={t0_ns}",
                    f"t1_monotonic_ns={t1_ns}",
                    "valid=true",
                    "normal_completion=true",
                    "",
                )
            ),
            encoding="utf-8",
        )
        duration_s = duration_ns / 1_000_000_000
        required_total = math.ceil(duration_s * 27.0)
        required_each = math.ceil(duration_s * 9.0)
        (run_dir / "workload-counts.txt").write_text(
            "\n".join(
                (
                    f"duration_actual_s={duration_s}",
                    "front=3001",
                    "rear=3001",
                    "top=3001",
                    "total=9003",
                    f"required_total={required_total}",
                    f"required_each={required_each}",
                    "unknown=0",
                    "",
                )
            ),
            encoding="utf-8",
        )
        memory_blocks = []
        for index in range(31):
            timestamp_ns = t0_ns + index * 10_000_000_000
            memory_blocks.append(
                "\n".join(
                    (
                        f"sample_realtime=2026-07-25T00:00:{index:02d}+00:00 "
                        f"sample_monotonic_ns={timestamp_ns}",
                        f"{tracee_pid} 11000 13000 32000 perception_input_node",
                        "Rss:               32000 kB",
                        "Pss:               13000 kB",
                        "Private_Clean:       100 kB",
                        "Private_Dirty:     10900 kB",
                    )
                )
            )
        (run_dir / "smem-smaps.txt").write_text(
            "\n\n".join(memory_blocks) + "\n", encoding="utf-8"
        )
        pidstat_line = (
            f"12:00:00 1000 {tracee_pid} 0.10 0.10 0.00 0.00 0.20 0 "
            "0.00 0.00 100000 32000 0.10 perception_input_node"
        )
        (run_dir / "pidstat.txt").write_text(
            "\n".join(pidstat_line for _ in range(pidstat_samples)) + "\n",
            encoding="utf-8",
        )
        return run_dir

    def run_report_parser(self, command: str, *arguments: Path | str):
        return subprocess.run(
            [
                sys.executable,
                str(REPORT_PARSER_PATH),
                command,
                *(str(argument) for argument in arguments),
            ],
            text=True,
            capture_output=True,
            check=False,
        )

    def test_valid_run_is_recomputed(self) -> None:
        result = ANALYZER.analyze_run(self.make_run("run-a"))
        self.assertEqual(240, result["pidstat_steady_samples"])
        self.assertGreaterEqual(result["steady_memory_samples"], 23)
        self.assertEqual(0.2, result["cpu_mean_percent"])

    def test_incomplete_pidstat_is_rejected(self) -> None:
        run_dir = self.make_run("run-a", pidstat_samples=298)
        with self.assertRaisesRegex(ValueError, "steady pidstat samples"):
            ANALYZER.analyze_run(run_dir)

    def test_wrong_mode_is_rejected(self) -> None:
        run_dir = self.make_run("run-a")
        manifest = run_dir / "run-manifest.txt"
        manifest.write_text(
            manifest.read_text(encoding="utf-8").replace(
                "mode=plain-sample", "mode=heaptrack"
            ),
            encoding="utf-8",
        )
        with self.assertRaisesRegex(ValueError, "mode is not plain-sample"):
            ANALYZER.analyze_run(run_dir)

    def test_low_workload_is_rejected(self) -> None:
        run_dir = self.make_run("run-a")
        workload = run_dir / "workload-counts.txt"
        workload.write_text(
            workload.read_text(encoding="utf-8")
            .replace("front=3001", "front=1")
            .replace("total=9003", "total=6003"),
            encoding="utf-8",
        )
        with self.assertRaisesRegex(ValueError, "workload count gate"):
            ANALYZER.analyze_run(run_dir)

    def test_run_directories_must_be_distinct(self) -> None:
        run_dir = self.make_run("run-a")
        with self.assertRaisesRegex(ValueError, "must be distinct"):
            ANALYZER.resolve_distinct_run_dirs([run_dir, run_dir, run_dir])

    def test_distinct_directories_require_independent_evidence(self) -> None:
        run_dirs = [self.make_run(name) for name in ("run-a", "run-b", "run-c")]
        with self.assertRaisesRegex(ValueError, "independent evidence identities"):
            ANALYZER.require_independent_evidence(run_dirs)

    def test_perf_control_window_must_be_nested(self) -> None:
        control = self.root / "perf-control.txt"
        quality = self.root / "perf-window-quality.txt"
        control.write_text(
            "enable_ack=ack\n"
            "enable_ack_monotonic_ns=100\n"
            "disable_ack=ack\n"
            "disable_ack_monotonic_ns=250\n"
            "stop_ack=ack\n"
            "stop_ack_monotonic_ns=300\n",
            encoding="utf-8",
        )
        result = self.run_report_parser(
            "perf-control",
            control,
            "150",
            "200",
            quality,
        )
        self.assertEqual(0, result.returncode, result.stderr)
        result = self.run_report_parser(
            "perf-control",
            control,
            "50",
            "200",
            quality,
        )
        self.assertNotEqual(0, result.returncode)

    def test_perf_control_rejects_duplicate_ack_evidence(self) -> None:
        control = self.root / "perf-control.txt"
        quality = self.root / "perf-window-quality.txt"
        control.write_text(
            "enable_ack=ack\n"
            "enable_ack=ack\n"
            "enable_ack_monotonic_ns=100\n"
            "disable_ack=ack\n"
            "disable_ack_monotonic_ns=250\n"
            "stop_ack=ack\n"
            "stop_ack_monotonic_ns=300\n",
            encoding="utf-8",
        )
        result = self.run_report_parser(
            "perf-control", control, "150", "200", quality
        )
        self.assertNotEqual(0, result.returncode)
        self.assertIn("duplicate perf control evidence", result.stderr)

    def test_c1_workload_parser_preserves_count_schema(self) -> None:
        measurement = self.root / "measurement.csv"
        output = self.root / "workload-counts.txt"
        rows = ["sensor_id"] + [
            sensor_id
            for _ in range(9)
            for sensor_id in ("front", "rear", "top")
        ]
        measurement.write_text("\n".join(rows) + "\n", encoding="utf-8")
        result = self.run_report_parser(
            "c1-workload", measurement, "1", "28", "0", "1000000000", output
        )
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual(
            "duration_actual_s=1.000000000\n"
            "line_start=1\n"
            "line_end=28\n"
            "front=9\n"
            "rear=9\n"
            "top=9\n"
            "total=27\n"
            "required_total=27\n"
            "required_each=9\n"
            "unknown=0\n",
            output.read_text(encoding="utf-8"),
        )

    def test_perf_stat_requires_every_declared_event(self) -> None:
        stat = self.root / "perf-stat.csv"
        quality = self.root / "perf-stat-quality.txt"
        events = (
            "task-clock",
            "context-switches",
            "cpu-migrations",
            "page-faults",
            "cycles:u",
            "instructions:u",
            "branches:u",
            "branch-misses:u",
            "cache-references:u",
            "cache-misses:u",
        )
        stat.write_text(
            "".join(f"1,,{event},1000000000,100.00\n" for event in events),
            encoding="utf-8",
        )
        result = self.run_report_parser("perf-stat", stat, quality)
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn("gate_pass=true", quality.read_text(encoding="utf-8"))
        stat.write_text(
            "".join(f"1,,{event},1000000000,100.00\n" for event in events[:-1]),
            encoding="utf-8",
        )
        result = self.run_report_parser("perf-stat", stat, quality)
        self.assertNotEqual(0, result.returncode)
        self.assertIn("missing perf stat events", result.stderr)

    def test_perf_record_rejects_lost_samples(self) -> None:
        symbols = self.root / "perf-symbols.txt"
        report = self.root / "perf-report.txt"
        quality = self.root / "perf-quality.txt"
        top = self.root / "perf-top10.txt"
        symbols.write_text(
            "# Samples: 1K of event 'cpu-clock:u'\n"
            "90.00%|900|perception_input_node|target|PerceptionInputNode::run\n"
            "10.00%|100|perception_input_node|[unknown]|[unknown]\n",
            encoding="utf-8",
        )
        report.write_text("# Total Lost Samples: 1\n", encoding="utf-8")
        result = self.run_report_parser(
            "perf-record",
            symbols,
            report,
            quality,
            top,
            "120",
        )
        self.assertNotEqual(0, result.returncode)
        report.write_text("# Total Lost Samples: 0\n", encoding="utf-8")
        result = self.run_report_parser(
            "perf-record",
            symbols,
            report,
            quality,
            top,
            "120",
        )
        self.assertEqual(0, result.returncode, result.stderr)

        symbols.write_text(
            symbols.read_text(encoding="utf-8").replace(
                "PerceptionInputNode::run", "third_party_symbol"
            ),
            encoding="utf-8",
        )
        result = self.run_report_parser(
            "perf-record", symbols, report, quality, top, "120"
        )
        self.assertNotEqual(0, result.returncode)

    def test_heaptrack_rejects_unparseable_peak(self) -> None:
        report = self.root / "heaptrack-report.txt"
        quality = self.root / "heaptrack-quality.txt"
        report.write_text(
            "MOST CALLS TO ALLOCATION FUNCTIONS\n"
            "PEAK MEMORY CONSUMERS\n"
            "MOST TEMPORARY ALLOCATIONS\n"
            "total runtime: 301.0s.\n"
            "calls to allocation functions: 100 (1/s)\n"
            "temporary memory allocations: 10 (1/s)\n"
            "peak heap memory consumption: invalid\n"
            "peak RSS (including heaptrack overhead): 40.62M\n"
            "total memory leaked: 426.69K\n",
            encoding="utf-8",
        )
        result = self.run_report_parser(
            "heaptrack",
            report,
            quality,
            "300",
        )
        self.assertNotEqual(0, result.returncode)
        report.write_text(
            report.read_text(encoding="utf-8").replace(
                "peak heap memory consumption: invalid",
                "peak heap memory consumption: 7.29M",
            ),
            encoding="utf-8",
        )
        result = self.run_report_parser(
            "heaptrack",
            report,
            quality,
            "300",
        )
        self.assertEqual(0, result.returncode, result.stderr)

    def test_massif_requires_stack_aware_peak(self) -> None:
        massif = self.root / "massif.out"
        quality = self.root / "massif-quality.txt"
        lines = ["cmd: /target --ros-args", "time_unit: ms"]
        for index in range(20):
            lines.extend(
                (
                    f"snapshot={index}",
                    f"time={index * 10000}",
                    "mem_heap_B=100",
                    "mem_heap_extra_B=10",
                    "mem_stacks_B=0",
                    "heap_tree=empty",
                )
            )
        massif.write_text("\n".join(lines) + "\n", encoding="utf-8")
        result = self.run_report_parser(
            "massif",
            massif,
            quality,
            "/target",
            "180",
        )
        self.assertNotEqual(0, result.returncode)
        massif.write_text(
            massif.read_text(encoding="utf-8")
            .replace("mem_stacks_B=0", "mem_stacks_B=100")
            .replace("heap_tree=empty", "heap_tree=peak", 1),
            encoding="utf-8",
        )
        result = self.run_report_parser(
            "massif",
            massif,
            quality,
            "/target",
            "180",
        )
        self.assertEqual(0, result.returncode, result.stderr)

    def test_memcheck_requires_target_command(self) -> None:
        log = self.root / "memcheck.log"
        summary = self.root / "memcheck-summary.txt"
        quality = self.root / "memcheck-quality.txt"
        log.write_text(
            "==42== Command: /wrong-target\n"
            "==42== HEAP SUMMARY:\n"
            "==42== in use at exit: 0 bytes in 0 blocks\n"
            "==42== total heap usage: 1 allocs, 1 frees, 1 bytes allocated\n"
            "==42== LEAK SUMMARY:\n"
            "==42== definitely lost: 0 bytes in 0 blocks\n"
            "==42== indirectly lost: 0 bytes in 0 blocks\n"
            "==42== possibly lost: 0 bytes in 0 blocks\n"
            "==42== still reachable: 0 bytes in 0 blocks\n"
            "==42== ERROR SUMMARY: 0 errors from 0 contexts\n",
            encoding="utf-8",
        )
        result = self.run_report_parser(
            "memcheck",
            log,
            summary,
            quality,
            "/target",
        )
        self.assertNotEqual(0, result.returncode)
        log.write_text(
            log.read_text(encoding="utf-8").replace(
                "Command: /wrong-target", "Command: /target"
            ),
            encoding="utf-8",
        )
        result = self.run_report_parser(
            "memcheck",
            log,
            summary,
            quality,
            "/target",
        )
        self.assertEqual(0, result.returncode, result.stderr)

        log.write_text(
            log.read_text(encoding="utf-8").replace(
                "==42== ERROR SUMMARY: 0 errors from 0 contexts\n", ""
            ),
            encoding="utf-8",
        )
        result = self.run_report_parser(
            "memcheck", log, summary, quality, "/target"
        )
        self.assertNotEqual(0, result.returncode)
        self.assertIn("missing Memcheck sections", result.stderr)

    def test_common_sampler_and_manifest_primitives_are_importable(self) -> None:
        run_dir = self.make_run("run-a")
        manifest_path = run_dir / "run-manifest.txt"
        manifest_path.write_text(
            manifest_path.read_text(encoding="utf-8") + "source_patch=a=b\n",
            encoding="utf-8",
        )
        manifest = COMMON_ANALYSIS.parse_manifest(manifest_path)
        self.assertEqual("a=b", manifest["source_patch"])
        memory = COMMON_ANALYSIS.parse_smem_smaps_samples(
            run_dir / "smem-smaps.txt", manifest["tracee_pid"]
        )
        cpu, rss = COMMON_ANALYSIS.parse_pidstat_samples(
            run_dir / "pidstat.txt", manifest["tracee_pid"]
        )
        self.assertEqual(31, len(memory))
        self.assertEqual(300, len(cpu))
        self.assertEqual(300, len(rss))
        self.assertEqual(0.2, cpu[0])

    def test_role_monitor_rejects_zombie_and_missing_child(self) -> None:
        proc_root = self.root / "proc"
        process_dir = proc_root / "4242"
        process_dir.mkdir(parents=True)
        stat_path = process_dir / "stat"

        def stat_line(state: str, starttime: str) -> str:
            suffix = [state, *("0" for _ in range(18)), starttime]
            return f"4242 (profile worker) {' '.join(suffix)}\n"

        stat_path.write_text(stat_line("S", "777"), encoding="ascii")
        role = ROLE_MONITOR.RoleSpec("tracee", 4242, "777")
        ROLE_MONITOR.validate_role(role, proc_root, None)
        stat_path.write_text(stat_line("Z", "777"), encoding="ascii")
        with self.assertRaisesRegex(ROLE_MONITOR.RoleMonitorError, "zombie"):
            ROLE_MONITOR.validate_role(role, proc_root, None)
        missing = ROLE_MONITOR.RoleSpec("fixture", 9001, "123")
        with self.assertRaisesRegex(
            ROLE_MONITOR.RoleMonitorError, "identity is unavailable"
        ):
            ROLE_MONITOR.validate_role(missing, proc_root, None)

    def test_shell_common_matches_only_exact_cmdline_arguments(self) -> None:
        process = subprocess.Popen(["sleep", "5"])
        try:
            exact = subprocess.run(
                [
                    "bash",
                    "-c",
                    f'source "{SHELL_COMMON_PATH}"; '
                    f'process_cmdline_has_exact_arguments {process.pid} sleep 5',
                ],
                text=True,
                capture_output=True,
                check=False,
            )
            substring = subprocess.run(
                [
                    "bash",
                    "-c",
                    f'source "{SHELL_COMMON_PATH}"; '
                    f'process_cmdline_has_exact_arguments {process.pid} lee 5',
                ],
                text=True,
                capture_output=True,
                check=False,
            )
        finally:
            process.terminate()
            process.wait(timeout=5)
        self.assertEqual(0, exact.returncode, exact.stderr)
        self.assertNotEqual(0, substring.returncode)

    def test_heaptrack_massif_timeline_parser_checks_seconds_and_target(self) -> None:
        timeline = self.root / "heaptrack-massif.out"
        quality = self.root / "heaptrack-massif-quality.txt"
        rows = []
        for index in range(20):
            rows.extend(
                (
                    f"snapshot={index}",
                    f"time={index + 0.125}",
                    f"mem_heap_B={1024 + index}",
                    "mem_heap_extra_B=0",
                    "mem_stacks_B=0",
                    "heap_tree=detailed",
                )
            )
        rows.extend(("desc: heaptrack", "cmd: /tmp/target --flag", "time_unit: s", ""))
        timeline.write_text("\n".join(rows), encoding="utf-8")
        parsed = REPORT_PARSERS.parse_heaptrack_massif_timeline(
            timeline, quality, "/tmp/target", 10
        )
        self.assertEqual(20, parsed["snapshot_count"])
        self.assertEqual("true", parsed["gate_pass"])
        with self.assertRaisesRegex(ValueError, "does not identify"):
            REPORT_PARSERS.parse_heaptrack_massif_timeline(
                timeline, quality, "/tmp/other-target", 10
            )

    def test_c1_runner_keeps_cli_common_provenance_and_completion_gate(self) -> None:
        result = subprocess.run(
            ["bash", str(RUNNER_PATH)],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(2, result.returncode)
        self.assertIn(
            "<mode> <install-prefix> <new-output-dir> <duration-seconds>",
            result.stderr,
        )
        runner = RUNNER_PATH.read_text(encoding="utf-8")
        self.assertIn('source "${PROFILE_RUNNER_COMMON}"', runner)
        self.assertIn("profile_runner_common_sha256=", runner)
        self.assertIn("profile_report_parsers_sha256=", runner)
        self.assertIn("profile_role_monitor_sha256=", runner)
        self.assertIn("profile_analysis_common_sha256=", runner)
        self.assertNotIn("ros2 trace --list", runner)
        self.assertIn("list_ros_trace_events", runner)
        self.assertIn("DEFAULT_EVENTS_ROS", SHELL_COMMON_PATH.read_text(encoding="utf-8"))
        self.assertNotIn(
            'wait_for_process_identity "${LAUNCHER_PID}" heaptrack', runner
        )
        self.assertIn(
            'TRACEE_PID="$(find_matching_descendant "${LAUNCHER_PID}" || true)"',
            runner,
        )
        self.assertIn("process_cmdline_has_exact_arguments", runner)
        self.assertIn("heaptrack_process_model=", runner)
        self.assertIn(
            'heaptrack_print -f "${heap_file}" -M '
            '"${OUTPUT_DIR}/heaptrack-massif.out"',
            runner,
        )
        self.assertNotIn('heaptrack_print -M "${heap_file}"', runner)
        self.assertIn('"${PROFILE_REPORT_PARSER}" heaptrack-massif', runner)
        self.assertIn("TARGET_IDENTITY_RECORDED=false", runner)
        self.assertIn("stop_partial_target_startup", runner)
        self.assertIn('if [[ "${TARGET_IDENTITY_RECORDED}" != true', runner)
        self.assertIn(
            'if [[ "${FORCED_STOP}" == false && "${ROLE_EXIT_FAILURE}" == false ]]',
            runner,
        )
        stop_start = runner.index("stop_perf()\n{")
        stop_end = runner.index("\nstop_trace()", stop_start)
        stop_perf = runner[stop_start:stop_end]
        disable_index = stop_perf.index("perf_control disable")
        stop_index = stop_perf.index("perf_control stop")
        close_index = stop_perf.index("exec 8>&- 9>&-")
        signal_index = stop_perf.index(
            'signal_process INT "${TOOL_PID}" "${TOOL_PGID}" "${TOOL_STARTTIME}"'
        )
        timeout_index = stop_perf.index('if ! wait_for_dead "${TOOL_PID}" 20; then')
        forced_index = stop_perf.index("FORCED_STOP=true")
        reap_index = stop_perf.index('wait_child "${TOOL_PID}" perf || true')
        self.assertLess(disable_index, stop_index)
        self.assertLess(stop_index, close_index)
        self.assertLess(close_index, signal_index)
        self.assertLess(signal_index, timeout_index)
        self.assertLess(timeout_index, forced_index)
        self.assertLess(forced_index, reap_index)
        self.assertIn("if (( rc != 0 && rc != 130 )); then", stop_perf)
        self.assertIn("ROLE_EXIT_FAILURE=true", stop_perf)
        self.assertIn(
            'invalidate "perf returned unexpected exit code ${rc}"', stop_perf
        )
        self.assertNotIn("tool_exit_code=", stop_perf)
        common = SHELL_COMMON_PATH.read_text(encoding="utf-8")
        self.assertIn(
            'process_identity_matches "${pid}" "${starttime}" "${pgid}"', common
        )
        self.assertTrue(SHELL_COMMON_PATH.is_file())
        self.assertTrue(ROLE_MONITOR_PATH.is_file())


if __name__ == "__main__":
    unittest.main()
