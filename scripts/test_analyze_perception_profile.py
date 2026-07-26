import importlib.util
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).with_name("analyze-perception-profile.py")
RUNNER_PATH = Path(__file__).with_name("profile-perception.sh")
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

    def run_embedded_parser(self, anchor: str, *arguments: Path | str):
        runner = RUNNER_PATH.read_text(encoding="utf-8")
        anchor_offset = runner.index(anchor)
        marker = "<<'PY'\n"
        start = runner.rfind(marker, 0, anchor_offset) + len(marker)
        end = runner.index("\nPY", anchor_offset)
        self.assertGreaterEqual(start, len(marker))
        return subprocess.run(
            [sys.executable, "-", *(str(argument) for argument in arguments)],
            input=runner[start:end],
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
        result = self.run_embedded_parser(
            "control_path, t0, t1, quality_path = sys.argv[1:]",
            control,
            "150",
            "200",
            quality,
        )
        self.assertEqual(0, result.returncode, result.stderr)
        result = self.run_embedded_parser(
            "control_path, t0, t1, quality_path = sys.argv[1:]",
            control,
            "50",
            "200",
            quality,
        )
        self.assertNotEqual(0, result.returncode)

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
        result = self.run_embedded_parser(
            "symbols_path, report_path, quality_path, top_path, requested_duration = sys.argv[1:]",
            symbols,
            report,
            quality,
            top,
            "120",
        )
        self.assertNotEqual(0, result.returncode)
        report.write_text("# Total Lost Samples: 0\n", encoding="utf-8")
        result = self.run_embedded_parser(
            "symbols_path, report_path, quality_path, top_path, requested_duration = sys.argv[1:]",
            symbols,
            report,
            quality,
            top,
            "120",
        )
        self.assertEqual(0, result.returncode, result.stderr)

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
        result = self.run_embedded_parser(
            "report_path, quality_path, requested_duration = sys.argv[1:]",
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
        result = self.run_embedded_parser(
            "report_path, quality_path, requested_duration = sys.argv[1:]",
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
        result = self.run_embedded_parser(
            "massif_path, quality_path, expected_target, requested_duration = sys.argv[1:]",
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
        result = self.run_embedded_parser(
            "massif_path, quality_path, expected_target, requested_duration = sys.argv[1:]",
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
        result = self.run_embedded_parser(
            "log_path, summary_path, quality_path, expected_target = sys.argv[1:]",
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
        result = self.run_embedded_parser(
            "log_path, summary_path, quality_path, expected_target = sys.argv[1:]",
            log,
            summary,
            quality,
            "/target",
        )
        self.assertEqual(0, result.returncode, result.stderr)


if __name__ == "__main__":
    unittest.main()
