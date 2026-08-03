#!/usr/bin/env python3
"""Positive and negative fixtures for graph-wide PID resolution, sampling, and analysis."""

from __future__ import annotations

import csv
import importlib.util
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ANALYZER_PATH = SCRIPT_DIR / "analyze-graph-profile.py"
RESOLVER_PATH = SCRIPT_DIR / "resolve-graph-pids.py"
RUNNER_PATH = SCRIPT_DIR / "profile-graph.sh"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from lib import graph_replay_equivalence as EQUIVALENCE  # noqa: E402
from lib import graph_run_gates as GATES  # noqa: E402
from lib import graph_sampler as GRAPH  # noqa: E402
from lib import profile_analysis as COMMON_ANALYSIS  # noqa: E402
from lib import profile_local_map_sampler as SINGLE_PID_SAMPLER  # noqa: E402
from lib import profile_role_monitor as ROLE_MONITOR  # noqa: E402

ANALYZER_SPEC = importlib.util.spec_from_file_location(
    "analyze_graph_profile", ANALYZER_PATH
)
assert ANALYZER_SPEC is not None and ANALYZER_SPEC.loader is not None
ANALYZER = importlib.util.module_from_spec(ANALYZER_SPEC)
ANALYZER_SPEC.loader.exec_module(ANALYZER)


def _symlinks_supported() -> bool:
    with tempfile.TemporaryDirectory(dir=SCRIPT_DIR) as directory:
        try:
            Path(directory, "probe").symlink_to("/usr/bin/probe")
        except OSError:
            return False
    return True


SYMLINKS_SUPPORTED = _symlinks_supported()
NEEDS_PROC = unittest.skipUnless(
    SYMLINKS_SUPPORTED, "a fabricated /proc tree needs symlink support for exe"
)

PRODUCT_NODES = ("cave_full_ray_perception_input", "cave_full_ray_local_map")
FIXTURE_NODE = "cave_full_ray_truth"
SUPPORT_ROLE_NAME = "bag_play"
NODE_PIDS = {
    PRODUCT_NODES[0]: 1001,
    PRODUCT_NODES[1]: 1002,
    FIXTURE_NODE: 1003,
    SUPPORT_ROLE_NAME: 1004,
}
NODE_TIERS = {
    PRODUCT_NODES[0]: GRAPH.PRODUCT_TIER,
    PRODUCT_NODES[1]: GRAPH.PRODUCT_TIER,
    FIXTURE_NODE: GRAPH.FIXTURE_TIER,
    SUPPORT_ROLE_NAME: GRAPH.SUPPORT_TIER,
}
NODE_ROLES = {
    PRODUCT_NODES[0]: GRAPH.MEASURED_ROLE,
    PRODUCT_NODES[1]: GRAPH.MEASURED_ROLE,
    FIXTURE_NODE: GRAPH.MEASURED_ROLE,
    SUPPORT_ROLE_NAME: GRAPH.SUPPORT_ROLE,
}
MEASURED_NODES = (PRODUCT_NODES[0], PRODUCT_NODES[1], FIXTURE_NODE)
BASE_PSS_KIB = {
    PRODUCT_NODES[0]: 40_000,
    PRODUCT_NODES[1]: 90_000,
    FIXTURE_NODE: 30_000,
    SUPPORT_ROLE_NAME: 20_000,
}
CPU_PERCENT = {
    PRODUCT_NODES[0]: 12.0,
    PRODUCT_NODES[1]: 41.0,
    FIXTURE_NODE: 7.0,
    SUPPORT_ROLE_NAME: 3.0,
}


def process_stat(pid: int, comm: str, state: str, starttime: str) -> str:
    fields = [state] + [str(index) for index in range(1, 19)] + [starttime]
    return f"{pid} ({comm}) " + " ".join(fields) + "\n"


def smaps_rollup(rss_kib: int, pss_kib: int, private_kib: int) -> str:
    return (
        "55a0d0000000-7ffd00000000 ---p 00000000 00:00 0                  [rollup]\n"
        f"Rss:              {rss_kib} kB\n"
        f"Pss:              {pss_kib} kB\n"
        "Shared_Clean:         0 kB\n"
        f"Private_Clean:    {private_kib // 2} kB\n"
        f"Private_Dirty:    {private_kib - private_kib // 2} kB\n"
    )


def pidstat_line(pid: int, cpu_percent: float, command: str) -> str:
    return (
        f"12:00:01      0      {pid}    1.00    0.50    0.00    0.00"
        f"    {cpu_percent:.2f}     3      0.00      0.00  123456   45678   0.55  {command}"
    )


class ProcFixture:
    """A fabricated /proc tree, enough for identity and smaps_rollup reads."""

    def __init__(self, root: Path) -> None:
        self.root = root
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root / "meminfo").write_text(
            "MemTotal:       16000000 kB\nMemAvailable:    8000000 kB\n",
            encoding="utf-8",
        )

    def add(
        self,
        pid: int,
        node_name: str,
        *,
        starttime: str = "9000",
        exe: str = "/opt/ros/node",
        argv: list[str] | None = None,
        state: str = "S",
        rss_kib: int = 100_000,
        pss_kib: int = 40_000,
    ) -> None:
        directory = self.root / str(pid)
        directory.mkdir(exist_ok=True)
        (directory / "stat").write_text(
            process_stat(pid, node_name[:15], state, starttime), encoding="ascii"
        )
        (directory / "cgroup").write_text("0::/measure\n", encoding="ascii")
        (directory / "smaps_rollup").write_text(
            smaps_rollup(rss_kib, pss_kib, pss_kib // 2), encoding="ascii"
        )
        tokens = argv if argv is not None else [exe, "--ros-args", "-r", f"__node:={node_name}"]
        (directory / "cmdline").write_bytes(
            b"\0".join(token.encode("utf-8") for token in tokens) + b"\0"
        )
        link = directory / "exe"
        if link.is_symlink() or link.exists():
            link.unlink()
        link.symlink_to(exe)

    def set_starttime(self, pid: int, node_name: str, starttime: str) -> None:
        (self.root / str(pid) / "stat").write_text(
            process_stat(pid, node_name[:15], "S", starttime), encoding="ascii"
        )

    def set_state(self, pid: int, node_name: str, state: str, starttime: str = "9000") -> None:
        (self.root / str(pid) / "stat").write_text(
            process_stat(pid, node_name[:15], state, starttime), encoding="ascii"
        )

    def set_exe(self, pid: int, exe: str) -> None:
        link = self.root / str(pid) / "exe"
        link.unlink()
        link.symlink_to(exe)


class TemporaryRootTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory(dir=SCRIPT_DIR)
        self.addCleanup(self.temporary_directory.cleanup)
        self.root = Path(self.temporary_directory.name)

    def make_cgroup_root(self) -> Path:
        cgroup_root = self.root / "cgroup"
        (cgroup_root / "measure").mkdir(parents=True, exist_ok=True)
        (cgroup_root / "measure" / "memory.current").write_text("100\n", encoding="ascii")
        (cgroup_root / "measure" / "memory.max").write_text("max\n", encoding="ascii")
        (cgroup_root / "measure" / "memory.events").write_text(
            "oom 0\noom_kill 0\n", encoding="ascii"
        )
        return cgroup_root


@NEEDS_PROC
class ResolveGraphPidsTest(TemporaryRootTest):
    def make_graph(self) -> ProcFixture:
        proc = ProcFixture(self.root / "proc")
        for node_name in MEASURED_NODES:
            proc.add(
                NODE_PIDS[node_name],
                node_name,
                starttime=str(9000 + NODE_PIDS[node_name]),
                exe=f"/opt/ros/{node_name}",
            )
        proc.add(
            2000,
            "ros2",
            argv=["/usr/bin/python3", "/opt/ros/jazzy/bin/ros2", "launch", "pkg", "a.py"],
        )
        return proc

    def test_resolves_every_expected_node_and_freezes_identity(self) -> None:
        proc = self.make_graph()
        identities = GRAPH.resolve_node_identities(
            [(name, GRAPH.MEASURED_ROLE, GRAPH.PRODUCT_TIER) for name in MEASURED_NODES],
            proc.root,
        )
        self.assertEqual([name for name in MEASURED_NODES], [i.node_name for i in identities])
        self.assertEqual([NODE_PIDS[name] for name in MEASURED_NODES], [i.pid for i in identities])
        self.assertEqual(
            [str(9000 + NODE_PIDS[name]) for name in MEASURED_NODES],
            [i.starttime for i in identities],
        )
        self.assertEqual(
            [f"/opt/ros/{name}" for name in MEASURED_NODES], [i.exe for i in identities]
        )

    def test_missing_expected_node_is_rejected(self) -> None:
        proc = self.make_graph()
        with self.assertRaises(GRAPH.GraphIdentityError) as raised:
            GRAPH.resolve_node_identities(
                [
                    (MEASURED_NODES[0], GRAPH.MEASURED_ROLE, GRAPH.PRODUCT_TIER),
                    ("cave_full_ray_pose_gate", GRAPH.MEASURED_ROLE, GRAPH.FIXTURE_TIER),
                ],
                proc.root,
            )
        self.assertIn("cave_full_ray_pose_gate", str(raised.exception))
        self.assertIn("not running", str(raised.exception))

    def test_duplicate_node_name_match_is_rejected(self) -> None:
        proc = self.make_graph()
        proc.add(3100, PRODUCT_NODES[0], starttime="7777", exe="/opt/ros/duplicate")
        with self.assertRaises(GRAPH.GraphIdentityError) as raised:
            GRAPH.resolve_node_identities(
                [(PRODUCT_NODES[0], GRAPH.MEASURED_ROLE, GRAPH.PRODUCT_TIER)], proc.root
            )
        self.assertIn("matched several processes", str(raised.exception))

    def test_executable_basename_matches_only_without_a_remap(self) -> None:
        proc = ProcFixture(self.root / "proc")
        proc.add(4100, "fake_odom", argv=["/opt/ros/lib/fake_odom"], exe="/opt/ros/lib/fake_odom")
        proc.add(
            4101,
            "renamed",
            argv=["/opt/ros/lib/fake_odom", "--ros-args", "-r", "__node:=renamed_odom"],
            exe="/opt/ros/lib/fake_odom",
        )
        identities = GRAPH.resolve_node_identities(
            [
                ("fake_odom", GRAPH.MEASURED_ROLE, GRAPH.FIXTURE_TIER),
                ("renamed_odom", GRAPH.MEASURED_ROLE, GRAPH.FIXTURE_TIER),
            ],
            proc.root,
        )
        self.assertEqual([4100, 4101], [identity.pid for identity in identities])

    def test_support_role_is_frozen_separately_from_measured_nodes(self) -> None:
        proc = self.make_graph()
        proc.add(NODE_PIDS[SUPPORT_ROLE_NAME], "bag", exe="/opt/ros/ros2", starttime="5555")
        output = self.root / "graph-identities.json"
        result = subprocess.run(
            [
                sys.executable,
                str(RESOLVER_PATH),
                "--product",
                PRODUCT_NODES[0],
                "--product",
                PRODUCT_NODES[1],
                "--fixture",
                FIXTURE_NODE,
                "--support",
                f"{SUPPORT_ROLE_NAME}={NODE_PIDS[SUPPORT_ROLE_NAME]}",
                "--proc-root",
                str(proc.root),
                "--output",
                str(output),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(0, result.returncode, result.stderr)
        payload = json.loads(output.read_text(encoding="utf-8"))
        roles = {record["node_name"]: record["role"] for record in payload["nodes"]}
        tiers = {record["node_name"]: record["tier"] for record in payload["nodes"]}
        self.assertEqual(GRAPH.SUPPORT_ROLE, roles[SUPPORT_ROLE_NAME])
        self.assertEqual(GRAPH.SUPPORT_TIER, tiers[SUPPORT_ROLE_NAME])
        self.assertEqual({GRAPH.MEASURED_ROLE}, {roles[name] for name in MEASURED_NODES})
        self.assertEqual(GRAPH.FIXTURE_TIER, tiers[FIXTURE_NODE])

    def test_resolver_cli_reports_a_missing_node_with_a_nonzero_exit(self) -> None:
        proc = self.make_graph()
        result = subprocess.run(
            [
                sys.executable,
                str(RESOLVER_PATH),
                "--product",
                "cave_full_ray_scanner",
                "--proc-root",
                str(proc.root),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(1, result.returncode)
        self.assertIn("cave_full_ray_scanner", result.stderr)


@NEEDS_PROC
class GraphSamplerTest(TemporaryRootTest):
    def make_graph(self) -> tuple[ProcFixture, list[GRAPH.NodeIdentity]]:
        proc = ProcFixture(self.root / "proc")
        for node_name in MEASURED_NODES:
            proc.add(
                NODE_PIDS[node_name],
                node_name,
                starttime=str(9000 + NODE_PIDS[node_name]),
                exe=f"/opt/ros/{node_name}",
                pss_kib=BASE_PSS_KIB[node_name],
            )
        identities = GRAPH.resolve_node_identities(
            [(name, GRAPH.MEASURED_ROLE, NODE_TIERS[name]) for name in MEASURED_NODES],
            proc.root,
        )
        return proc, identities

    def run_loop(self, proc: ProcFixture, identities, rounds: int) -> int:
        remaining = [rounds]

        def stop_requested() -> bool:
            if remaining[0] <= 0:
                return True
            remaining[0] -= 1
            return False

        return GRAPH.sample_loop(
            identities,
            self.root / GRAPH.SAMPLES_FILENAME,
            self.root / GRAPH.HEADROOM_FILENAME,
            0.001,
            stop_requested,
            0.0,
            proc.root,
            self.make_cgroup_root(),
            proc.root / "meminfo",
        )

    def read_samples(self) -> list[dict[str, str]]:
        with (self.root / GRAPH.SAMPLES_FILENAME).open(newline="", encoding="utf-8") as stream:
            return list(csv.DictReader(stream))

    def test_every_sample_index_holds_one_row_per_node(self) -> None:
        proc, identities = self.make_graph()
        written = self.run_loop(proc, identities, 4)
        self.assertEqual(4, written)
        rows = self.read_samples()
        self.assertEqual(4 * len(MEASURED_NODES), len(rows))
        by_index: dict[str, set[str]] = {}
        for row in rows:
            by_index.setdefault(row["sample_index"], set()).add(row["node_name"])
        self.assertEqual({"0", "1", "2", "3"}, set(by_index))
        for names in by_index.values():
            self.assertEqual(set(MEASURED_NODES), names)
        self.assertEqual(
            str(BASE_PSS_KIB[PRODUCT_NODES[1]]),
            next(row["pss_kib"] for row in rows if row["node_name"] == PRODUCT_NODES[1]),
        )

    def test_headroom_is_captured_once_per_round(self) -> None:
        proc, identities = self.make_graph()
        self.run_loop(proc, identities, 3)
        with (self.root / GRAPH.HEADROOM_FILENAME).open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(["0", "1", "2"], [row["sample_index"] for row in rows])
        self.assertEqual({str(len(MEASURED_NODES))}, {row["node_count"] for row in rows})
        self.assertEqual({"8000000"}, {row["mem_available_kib"] for row in rows})
        self.assertEqual({"100"}, {row["cgroup_current_bytes"] for row in rows})
        self.assertEqual({""}, {row["cgroup_max_bytes"] for row in rows})

    def test_pid_reuse_stops_sampling_and_writes_no_partial_round(self) -> None:
        proc, identities = self.make_graph()
        rounds = [0]

        def stop_requested() -> bool:
            rounds[0] += 1
            if rounds[0] == 3:
                proc.set_starttime(NODE_PIDS[FIXTURE_NODE], FIXTURE_NODE, "424242")
            return False

        with self.assertRaises(GRAPH.GraphIdentityError) as raised:
            GRAPH.sample_loop(
                identities,
                self.root / GRAPH.SAMPLES_FILENAME,
                self.root / GRAPH.HEADROOM_FILENAME,
                0.001,
                stop_requested,
                0.0,
                proc.root,
                self.make_cgroup_root(),
                proc.root / "meminfo",
            )
        self.assertIn("starttime changed", str(raised.exception))
        self.assertIn("reused", str(raised.exception))
        rows = self.read_samples()
        by_index: dict[str, set[str]] = {}
        for row in rows:
            by_index.setdefault(row["sample_index"], set()).add(row["node_name"])
        self.assertTrue(by_index)
        for names in by_index.values():
            self.assertEqual(set(MEASURED_NODES), names)

    def test_executable_change_is_rejected(self) -> None:
        proc, identities = self.make_graph()
        proc.set_exe(NODE_PIDS[PRODUCT_NODES[0]], "/opt/ros/other_node")
        with self.assertRaises(GRAPH.GraphIdentityError) as raised:
            GRAPH.sample_round(identities, 0, self.make_cgroup_root(), proc.root, proc.root / "meminfo")
        self.assertIn("executable changed", str(raised.exception))

    def test_zombie_node_is_rejected(self) -> None:
        proc, identities = self.make_graph()
        proc.set_state(
            NODE_PIDS[PRODUCT_NODES[1]],
            PRODUCT_NODES[1],
            "Z",
            str(9000 + NODE_PIDS[PRODUCT_NODES[1]]),
        )
        with self.assertRaises(GRAPH.GraphIdentityError) as raised:
            GRAPH.sample_round(identities, 0, self.make_cgroup_root(), proc.root, proc.root / "meminfo")
        self.assertIn("zombie", str(raised.exception))

    def test_cli_records_the_violation_reason_and_exits_nonzero(self) -> None:
        proc, identities = self.make_graph()
        identities_path = self.root / GRAPH.IDENTITIES_FILENAME
        identities_path.write_text(GRAPH.dump_frozen_identities(identities, proc.root), encoding="utf-8")
        proc.set_starttime(NODE_PIDS[FIXTURE_NODE], FIXTURE_NODE, "1")
        violation = self.root / "identity-violation.txt"
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPT_DIR / "lib" / "graph_sampler.py"),
                str(identities_path),
                str(self.root / GRAPH.SAMPLES_FILENAME),
                str(self.root / GRAPH.HEADROOM_FILENAME),
                str(violation),
                "--interval-s",
                "0.01",
                "--duration-s",
                "1",
                "--proc-root",
                str(proc.root),
                "--cgroup-root",
                str(self.make_cgroup_root()),
                "--meminfo",
                str(proc.root / "meminfo"),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(1, result.returncode, result.stdout)
        self.assertIn("starttime changed", violation.read_text(encoding="utf-8"))


class FrozenIdentityTest(TemporaryRootTest):
    def write_identities(self, records: list[dict]) -> Path:
        path = self.root / GRAPH.IDENTITIES_FILENAME
        path.write_text(json.dumps({"nodes": records}), encoding="utf-8")
        return path

    def record(self, node_name: str, pid: int, **overrides) -> dict:
        record = {
            "node_name": node_name,
            "role": NODE_ROLES[node_name],
            "tier": NODE_TIERS[node_name],
            "pid": pid,
            "starttime": str(9000 + pid),
            "exe": f"/opt/ros/{node_name}",
        }
        record.update(overrides)
        return record

    def test_repeated_node_name_is_rejected(self) -> None:
        path = self.write_identities(
            [self.record(PRODUCT_NODES[0], 11), self.record(PRODUCT_NODES[0], 12)]
        )
        with self.assertRaises(GRAPH.GraphIdentityError) as raised:
            GRAPH.load_frozen_identities(path)
        self.assertIn("repeat a node name", str(raised.exception))

    def test_repeated_pid_is_rejected(self) -> None:
        path = self.write_identities(
            [self.record(PRODUCT_NODES[0], 11), self.record(PRODUCT_NODES[1], 11)]
        )
        with self.assertRaises(GRAPH.GraphIdentityError) as raised:
            GRAPH.load_frozen_identities(path)
        self.assertIn("repeat a PID", str(raised.exception))

    def test_unknown_role_is_rejected(self) -> None:
        path = self.write_identities([self.record(PRODUCT_NODES[0], 11, role="tracee")])
        with self.assertRaises(GRAPH.GraphIdentityError) as raised:
            GRAPH.load_frozen_identities(path)
        self.assertIn("unknown role", str(raised.exception))

    def test_identities_without_a_measured_node_are_rejected(self) -> None:
        path = self.write_identities([self.record(SUPPORT_ROLE_NAME, 14)])
        with self.assertRaises(GRAPH.GraphIdentityError) as raised:
            GRAPH.load_frozen_identities(path)
        self.assertIn("no measured node", str(raised.exception))

    def test_node_name_matching_prefers_an_explicit_remap(self) -> None:
        self.assertEqual(
            {"cave_full_ray_local_map"},
            GRAPH.node_names_in_cmdline(
                [
                    "/opt/ros/perception_local_map_node",
                    "--ros-args",
                    "-r",
                    "__node:=cave_full_ray_local_map",
                ]
            ),
        )
        self.assertEqual(
            {"perception_local_map_node"},
            GRAPH.node_names_in_cmdline(["/opt/ros/perception_local_map_node"]),
        )
        self.assertEqual(
            {"cave_publisher"},
            GRAPH.node_names_in_cmdline(
                ["/usr/bin/python3", "/opt/ros/lib/cave_world/cave_publisher"]
            ),
        )
        self.assertEqual(set(), GRAPH.node_names_in_cmdline([]))


class GraphRunBuilder:
    """Builds an on-disk graph profiling run directory."""

    def __init__(self, run_dir: Path) -> None:
        self.run_dir = run_dir
        run_dir.mkdir(parents=True)

    def write(
        self,
        *,
        rounds: int = 20,
        t0_ns: int = 1_000_000_000_000,
        interval_ns: int = 1_000_000_000,
        growth_kib_per_round: dict[str, int] | None = None,
        pid_offset: int = 0,
        drop_sample: tuple[int, str] | None = None,
        drop_round: int | None = None,
        drop_headroom_index: int | None = None,
        headroom_node_count: int | None = None,
        pidstat_short_node: str | None = None,
        node_names: tuple[str, ...] = tuple(NODE_PIDS),
        pss_for=None,
        cpu_for=None,
        segment_rounds: list[tuple[str, int, int]] | None = None,
        segment_lines: list[str] | None = None,
        mode: str = "graph-sample",
        replay_equivalence: str | None = None,
    ) -> Path:
        growth = growth_kib_per_round or {}
        pids = {name: NODE_PIDS[name] + pid_offset for name in node_names}
        identities = {
            "frozen_monotonic_ns": t0_ns,
            "proc_root": "/proc",
            "nodes": [
                {
                    "node_name": name,
                    "role": NODE_ROLES[name],
                    "tier": NODE_TIERS[name],
                    "pid": pids[name],
                    "starttime": str(9000 + pids[name]),
                    "exe": f"/opt/ros/{name}",
                }
                for name in node_names
            ],
        }
        (self.run_dir / GRAPH.IDENTITIES_FILENAME).write_text(
            json.dumps(identities, indent=2), encoding="utf-8"
        )

        sample_rows = []
        headroom_rows = []
        for index in range(rounds):
            if drop_round is not None and index == drop_round:
                continue
            receipt_ns = t0_ns + index * interval_ns
            for offset, name in enumerate(node_names):
                if drop_sample == (index, name):
                    continue
                if pss_for is not None:
                    pss = pss_for(name, index)
                else:
                    pss = BASE_PSS_KIB[name] + growth.get(name, 0) * index
                sample_rows.append(
                    {
                        "sample_index": index,
                        "receipt_monotonic_ns": receipt_ns + offset,
                        "node_name": name,
                        "role": NODE_ROLES[name],
                        "tier": NODE_TIERS[name],
                        "pid": pids[name],
                        "rss_kib": pss * 2,
                        "pss_kib": pss,
                        "uss_kib": pss // 2,
                    }
                )
            if drop_headroom_index is not None and index == drop_headroom_index:
                continue
            headroom_rows.append(
                {
                    "sample_index": index,
                    "receipt_monotonic_ns": receipt_ns + len(node_names),
                    "node_count": headroom_node_count
                    if headroom_node_count is not None
                    else len(node_names),
                    "cgroup_current_bytes": 100,
                    "cgroup_max_bytes": "",
                    "mem_available_kib": 8_000_000,
                    "oom": 0,
                    "oom_kill": 0,
                }
            )
        self._write_csv(GRAPH.SAMPLES_FILENAME, GRAPH.SAMPLE_COLUMNS, sample_rows)
        self._write_csv(GRAPH.HEADROOM_FILENAME, GRAPH.HEADROOM_COLUMNS, headroom_rows)

        lines = [
            "Linux 6.10.14-linuxkit (dev) \t08/01/26 \t_x86_64_\t(8 CPU)",
            "",
            "#      Time   UID       PID    %usr %system  %guest   %wait    %CPU"
            "   CPU  minflt/s  majflt/s     VSZ     RSS   %MEM  Command",
        ]
        for index in range(rounds):
            for name in node_names:
                if pidstat_short_node == name and index >= rounds - 3:
                    continue
                percent = (
                    cpu_for(name, index) if cpu_for is not None else CPU_PERCENT[name]
                )
                lines.append(pidstat_line(pids[name], percent, name))
        (self.run_dir / "pidstat.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")

        last_receipt = t0_ns + (rounds - 1) * interval_ns + len(node_names)
        manifest = [
            f"mode={mode}",
            f"duration_requested_s={rounds}",
            f"t0_monotonic_ns={t0_ns}",
            f"t1_monotonic_ns={last_receipt}",
            # pidstat sample i averages [t0 + i * interval, t0 + (i + 1) * interval].
            f"pidstat_start_monotonic_ns={t0_ns}",
            f"pidstat_interval_s={interval_ns / 1_000_000_000}",
        ]
        if segment_rounds is not None:
            manifest.append(f"segment_count={len(segment_rounds)}")
            for index, (name, first, last) in enumerate(segment_rounds):
                manifest.append(f"segment_{index}_name={name}")
                manifest.append(
                    f"segment_{index}_t0_monotonic_ns={t0_ns + first * interval_ns}"
                )
                manifest.append(
                    f"segment_{index}_t1_monotonic_ns="
                    f"{t0_ns + last * interval_ns + len(node_names)}"
                )
                manifest.append(f"segment_{index}_boundary_source=observed_scanner_latch")
        if segment_lines is not None:
            manifest.extend(segment_lines)
        manifest.extend(("valid=true", "normal_completion=true", ""))
        (self.run_dir / "run-manifest.txt").write_text(
            "\n".join(manifest), encoding="utf-8"
        )
        if replay_equivalence is not None:
            (self.run_dir / ANALYZER.REPLAY_EQUIVALENCE_FILENAME).write_text(
                replay_equivalence, encoding="utf-8"
            )
        return self.run_dir

    def _write_csv(self, name: str, fieldnames: tuple[str, ...], rows: list[dict]) -> None:
        with (self.run_dir / name).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)


class AnalyzeGraphProfileTest(TemporaryRootTest):
    def make_run(self, name: str, **kwargs) -> Path:
        return GraphRunBuilder(self.root / name).write(**kwargs)

    def test_single_run_reports_every_node_and_ranks_the_bottleneck(self) -> None:
        result = ANALYZER.analyze_run(self.make_run("run-a"))
        self.assertEqual(set(NODE_PIDS), set(result["nodes"]))
        self.assertEqual(20, result["round_count"])
        local_map = result["nodes"][PRODUCT_NODES[1]]
        self.assertEqual(41.0, local_map["cpu"]["mean_percent"])
        self.assertEqual(BASE_PSS_KIB[PRODUCT_NODES[1]], local_map["memory"]["pss_kib"]["peak"])
        self.assertEqual(
            [PRODUCT_NODES[1], PRODUCT_NODES[0], FIXTURE_NODE],
            [entry["node_name"] for entry in result["cpu_ranking"]],
        )
        self.assertNotIn(
            SUPPORT_ROLE_NAME, [entry["node_name"] for entry in result["cpu_ranking"]]
        )
        self.assertEqual(
            {GRAPH.PRODUCT_TIER: 53.0, GRAPH.FIXTURE_TIER: 7.0},
            result["cpu_mean_percent_by_tier"],
        )
        self.assertEqual(0.30, result["cpu_percent_uncertainty_ratio"])
        self.assertEqual(8_000_000, result["headroom"]["mem_available_kib_min"])
        self.assertEqual(0, result["headroom"]["oom_events_at_window_end"])

    def test_system_total_sums_pss_of_measured_nodes_only(self) -> None:
        result = ANALYZER.analyze_run(self.make_run("run-a"))
        expected = sum(BASE_PSS_KIB[name] for name in MEASURED_NODES)
        self.assertEqual(expected, result["system_total_pss_measured"]["peak_kib"])
        self.assertEqual(expected, result["system_total_pss_measured"]["mean_kib"])
        self.assertTrue(result["system_total_pss_measured"]["summed_within_one_sampling_round"])
        self.assertEqual(
            BASE_PSS_KIB[SUPPORT_ROLE_NAME], result["support_role_total_pss"]["peak_kib"]
        )

    def test_rss_is_never_summed(self) -> None:
        result = ANALYZER.analyze_run(self.make_run("run-a"))
        self.assertIn("rss_is_never_summed", result["rss_sum_policy"])
        serialized = json.dumps(result)
        for forbidden in ("rss_total", "total_rss", "system_total_rss"):
            self.assertNotIn(f'"{forbidden}', serialized)

    def test_incomplete_round_blocks_the_pss_total(self) -> None:
        run_dir = self.make_run("run-a", drop_sample=(7, FIXTURE_NODE))
        with self.assertRaises(ValueError) as raised:
            ANALYZER.analyze_run(run_dir)
        message = str(raised.exception)
        self.assertIn("sample_index 7 is missing", message)
        self.assertIn(FIXTURE_NODE, message)
        self.assertIn("incomplete round", message)

    def test_interrupted_sampling_leaves_a_sample_index_gap(self) -> None:
        run_dir = self.make_run("run-a", drop_round=9)
        with self.assertRaises(ValueError) as raised:
            ANALYZER.analyze_run(run_dir)
        self.assertIn("sampling was interrupted", str(raised.exception))

    def test_unequal_pidstat_row_counts_are_rejected(self) -> None:
        run_dir = self.make_run("run-a", pidstat_short_node=FIXTURE_NODE)
        with self.assertRaises(ValueError) as raised:
            ANALYZER.analyze_run(run_dir)
        self.assertIn("unequal sample counts", str(raised.exception))

    def test_missing_headroom_row_is_rejected(self) -> None:
        run_dir = self.make_run("run-a", drop_headroom_index=4)
        with self.assertRaises(ValueError) as raised:
            ANALYZER.analyze_run(run_dir)
        self.assertIn("no headroom row", str(raised.exception))

    def test_headroom_node_count_must_match_the_sampled_round(self) -> None:
        run_dir = self.make_run("run-a", headroom_node_count=2)
        with self.assertRaises(ValueError) as raised:
            ANALYZER.analyze_run(run_dir)
        self.assertIn("incomplete round", str(raised.exception))

    def test_sample_from_a_foreign_pid_is_rejected(self) -> None:
        run_dir = self.make_run("run-a")
        samples_path = run_dir / GRAPH.SAMPLES_FILENAME
        text = samples_path.read_text(encoding="utf-8")
        samples_path.write_text(
            text.replace(f",{NODE_PIDS[FIXTURE_NODE]},", ",999999,", 1), encoding="utf-8"
        )
        with self.assertRaises(ValueError) as raised:
            ANALYZER.analyze_run(run_dir)
        self.assertIn("foreign PID", str(raised.exception))

    def test_invalid_run_is_rejected(self) -> None:
        run_dir = self.make_run("run-a")
        manifest = run_dir / "run-manifest.txt"
        manifest.write_text(
            manifest.read_text(encoding="utf-8").replace("valid=true", "valid=false"),
            encoding="utf-8",
        )
        with self.assertRaises(ValueError) as raised:
            ANALYZER.analyze_run(run_dir)
        self.assertIn("not valid", str(raised.exception))

    def make_three_runs(self, growth: dict[str, int] | None = None) -> list[dict]:
        results = []
        for index, name in enumerate(("run-a", "run-b", "run-c")):
            run_dir = self.make_run(
                name,
                t0_ns=1_000_000_000_000 + index * 10_000_000_000_000,
                pid_offset=index * 10,
                growth_kib_per_round=growth,
            )
            results.append(ANALYZER.analyze_run(run_dir))
        return results

    def test_three_flat_runs_do_not_suspect_growth(self) -> None:
        aggregate = ANALYZER.aggregate_runs(self.make_three_runs())
        self.assertEqual(3, aggregate["run_count"])
        self.assertFalse(aggregate["any_node_suspected_sustained_growth"])
        self.assertFalse(aggregate["system_total_pss_suspected_sustained_growth"])
        for node_name in NODE_PIDS:
            self.assertFalse(aggregate["nodes"][node_name]["suspected_sustained_growth"])
            self.assertEqual(
                [0.0, 0.0, 0.0], aggregate["nodes"][node_name]["slopes_kib_per_min"]["pss_kib"]
            )

    def test_three_growing_runs_above_threshold_suspect_growth(self) -> None:
        aggregate = ANALYZER.aggregate_runs(
            self.make_three_runs({PRODUCT_NODES[1]: 100})
        )
        self.assertTrue(aggregate["any_node_suspected_sustained_growth"])
        leaking = aggregate["nodes"][PRODUCT_NODES[1]]
        self.assertTrue(leaking["suspected_sustained_growth"])
        for slope in leaking["slopes_kib_per_min"]["pss_kib"]:
            self.assertAlmostEqual(6000.0, slope, places=3)
        self.assertFalse(
            aggregate["nodes"][PRODUCT_NODES[0]]["suspected_sustained_growth"]
        )
        self.assertTrue(aggregate["system_total_pss_suspected_sustained_growth"])

    def test_growth_below_threshold_does_not_suspect_growth(self) -> None:
        aggregate = ANALYZER.aggregate_runs(self.make_three_runs({PRODUCT_NODES[1]: 5}))
        self.assertFalse(aggregate["any_node_suspected_sustained_growth"])
        leaking = aggregate["nodes"][PRODUCT_NODES[1]]
        for slope in leaking["slopes_kib_per_min"]["pss_kib"]:
            self.assertAlmostEqual(300.0, slope, places=3)
        for slope in leaking["slopes_kib_per_min"]["rss_kib"]:
            self.assertAlmostEqual(600.0, slope, places=3)
        self.assertEqual(
            {"rss_kib": False, "pss_kib": False}, leaking["all_runs_above_threshold"]
        )

    def test_duplicate_evidence_identity_is_rejected(self) -> None:
        results = [ANALYZER.analyze_run(self.make_run("run-a"))] * 3
        with self.assertRaises(ValueError) as raised:
            ANALYZER.aggregate_runs(results)
        self.assertIn("duplicate evidence identities", str(raised.exception))

    def test_mismatched_node_sets_are_rejected(self) -> None:
        results = self.make_three_runs()
        results[2] = ANALYZER.analyze_run(
            self.make_run(
                "run-d",
                t0_ns=9_000_000_000_000,
                pid_offset=40,
                node_names=(PRODUCT_NODES[0], PRODUCT_NODES[1], FIXTURE_NODE),
            )
        )
        with self.assertRaises(ValueError) as raised:
            ANALYZER.aggregate_runs(results)
        self.assertIn("same node set", str(raised.exception))

    def test_cli_writes_a_machine_readable_summary(self) -> None:
        run_dir = self.make_run("run-a")
        output = self.root / "summary.json"
        result = subprocess.run(
            [sys.executable, str(ANALYZER_PATH), str(run_dir), "--output", str(output)],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(0, result.returncode, result.stderr)
        payload = json.loads(output.read_text(encoding="utf-8"))
        self.assertIsNone(payload["aggregate"])
        self.assertEqual(
            PRODUCT_NODES[1], payload["runs"][0]["cpu_ranking"][0]["node_name"]
        )

    def test_cli_rejects_two_run_directories(self) -> None:
        first = self.make_run("run-a")
        second = self.make_run("run-b", t0_ns=5_000_000_000_000, pid_offset=10)
        result = subprocess.run(
            [sys.executable, str(ANALYZER_PATH), str(first), str(second)],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(2, result.returncode)
        self.assertIn("exactly three independent runs", result.stderr)


MOTION_ROUNDS = 10
SETTLED_CPU_PERCENT = 5.0
MOTION_CPU_PERCENT = 60.0
MOTION_PSS_GROWTH_KIB = 1_000
SETTLED_NOISE_KIB = (0, 4, -4, 2, -2, 6, -6, 1, -1, 0)


def segmented_pss(name: str, index: int) -> int:
    """A phase whose map grows, then a phase where it is saturated and noisy."""
    base = BASE_PSS_KIB[name]
    if index < MOTION_ROUNDS:
        return base + MOTION_PSS_GROWTH_KIB * index
    settled = base + MOTION_PSS_GROWTH_KIB * (MOTION_ROUNDS - 1)
    return settled + SETTLED_NOISE_KIB[(index - MOTION_ROUNDS) % len(SETTLED_NOISE_KIB)]


def segmented_cpu(_name: str, index: int) -> float:
    return MOTION_CPU_PERCENT if index < MOTION_ROUNDS else SETTLED_CPU_PERCENT


class SegmentedAnalysisTest(TemporaryRootTest):
    """A run with two phases must never be summarized as one average."""

    def make_segmented_run(self, name: str = "run-a", **overrides) -> Path:
        arguments = {
            "rounds": 20,
            "pss_for": segmented_pss,
            "cpu_for": segmented_cpu,
            "segment_rounds": [("motion", 0, 9), ("settled", 10, 19)],
        }
        arguments.update(overrides)
        return GraphRunBuilder(self.root / name).write(**arguments)

    def test_each_segment_reports_its_own_cpu_and_memory(self) -> None:
        result = ANALYZER.analyze_run(self.make_segmented_run())
        self.assertEqual(["motion", "settled"], result["segment_names"])
        motion, settled = result["segments"]

        self.assertEqual(
            MOTION_CPU_PERCENT, motion["nodes"][PRODUCT_NODES[1]]["cpu"]["mean_percent"]
        )
        self.assertEqual(
            SETTLED_CPU_PERCENT,
            settled["nodes"][PRODUCT_NODES[1]]["cpu"]["mean_percent"],
        )
        # The whole-window mean sits between the two and describes neither phase.
        self.assertEqual(
            32.5, result["nodes"][PRODUCT_NODES[1]]["cpu"]["mean_percent"]
        )

        motion_memory = motion["nodes"][PRODUCT_NODES[1]]["memory"]["pss_kib"]
        settled_memory = settled["nodes"][PRODUCT_NODES[1]]["memory"]["pss_kib"]
        self.assertAlmostEqual(
            MOTION_PSS_GROWTH_KIB * 60.0, motion_memory["slope_kib_per_min"], places=1
        )
        self.assertLess(abs(settled_memory["slope_kib_per_min"]), 60.0)
        self.assertEqual(
            BASE_PSS_KIB[PRODUCT_NODES[1]], motion_memory["mean"] - 4_500.0
        )
        self.assertGreater(settled_memory["mean"], motion_memory["mean"])

    def test_a_segment_never_shares_a_sample_with_its_neighbour(self) -> None:
        result = ANALYZER.analyze_run(self.make_segmented_run())
        motion, settled = result["segments"]
        self.assertEqual(10, motion["round_count"])
        self.assertEqual(10, settled["round_count"])
        self.assertEqual(20, result["round_count"])
        # The pidstat sample that averages across the boundary belongs to neither.
        self.assertEqual(9, motion["cpu_sample_count"])
        self.assertEqual(9, settled["cpu_sample_count"])
        self.assertEqual(11, motion["cpu_samples_excluded_at_boundaries"])
        self.assertLessEqual(motion["t1_monotonic_ns"], settled["t0_monotonic_ns"])
        self.assertIn("never_merged", result["segment_policy"])

    def test_each_segment_sums_pss_only_across_measured_nodes(self) -> None:
        result = ANALYZER.analyze_run(self.make_segmented_run())
        motion = result["segments"][0]
        self.assertEqual(len(MEASURED_NODES), motion["system_total_pss_measured"]["node_count"])
        self.assertTrue(
            motion["system_total_pss_measured"]["summed_within_one_sampling_round"]
        )
        self.assertNotIn(SUPPORT_ROLE_NAME, str(motion["system_total_pss_measured"]))

    def test_segment_boundary_source_is_carried_into_the_report(self) -> None:
        result = ANALYZER.analyze_run(self.make_segmented_run())
        self.assertEqual(
            {"observed_scanner_latch"},
            {segment["boundary_source"] for segment in result["segments"]},
        )

    def test_overlapping_segments_are_rejected(self) -> None:
        run_dir = self.make_segmented_run(segment_rounds=[("motion", 0, 12), ("settled", 10, 19)])
        with self.assertRaises(ValueError) as raised:
            ANALYZER.analyze_run(run_dir)
        self.assertIn("overlaps", str(raised.exception))

    def test_a_segment_escaping_the_formal_window_is_rejected(self) -> None:
        run_dir = self.make_segmented_run()
        manifest = run_dir / "run-manifest.txt"
        manifest.write_text(
            manifest.read_text(encoding="utf-8").replace(
                "segment_0_t0_monotonic_ns=1000000000000",
                "segment_0_t0_monotonic_ns=999000000000",
            ),
            encoding="utf-8",
        )
        with self.assertRaises(ValueError) as raised:
            ANALYZER.analyze_run(run_dir)
        self.assertIn("escapes the formal window", str(raised.exception))

    def test_a_segment_too_short_for_statistics_is_rejected(self) -> None:
        run_dir = self.make_segmented_run(
            segment_rounds=[("motion", 0, 9), ("blink", 12, 12)]
        )
        with self.assertRaises(ValueError) as raised:
            ANALYZER.analyze_run(run_dir)
        self.assertIn("fewer than two complete", str(raised.exception))

    def test_segment_cpu_requires_a_pidstat_time_base(self) -> None:
        run_dir = self.make_segmented_run()
        manifest = run_dir / "run-manifest.txt"
        manifest.write_text(
            "\n".join(
                line
                for line in manifest.read_text(encoding="utf-8").splitlines()
                if not line.startswith("pidstat_start_monotonic_ns")
            ),
            encoding="utf-8",
        )
        with self.assertRaises(ValueError) as raised:
            ANALYZER.analyze_run(run_dir)
        self.assertIn("pidstat_start_monotonic_ns", str(raised.exception))

    def test_a_run_without_segments_still_analyzes(self) -> None:
        result = ANALYZER.analyze_run(GraphRunBuilder(self.root / "run-b").write())
        self.assertEqual([], result["segments"])
        self.assertEqual([], result["segment_names"])


class LeakSensitivityTest(TemporaryRootTest):
    """No growth observed is only meaningful next to what could have been seen."""

    @staticmethod
    def series(values: list[int], interval_ns: int = 1_000_000_000):
        return [(index * interval_ns, value) for index, value in enumerate(values)]

    def test_a_noisy_flat_series_reports_a_positive_sensitivity_floor(self) -> None:
        estimate = COMMON_ANALYSIS.estimate_slope(
            self.series([100, 104, 96, 102, 98, 106, 94, 101, 99, 100])
        )
        self.assertAlmostEqual(0.0, estimate.slope_kib_per_min, delta=30.0)
        self.assertGreater(estimate.detectable_kib_per_min, 0.0)
        self.assertTrue(estimate.resolvable)
        self.assertAlmostEqual(
            estimate.detectable_kib_per_min,
            estimate.sensitivity_multiplier * estimate.stderr_kib_per_min,
            places=6,
        )

    def test_a_shorter_window_resolves_less(self) -> None:
        noise = [100, 104, 96, 102, 98, 106, 94, 101, 99, 100]
        long_window = COMMON_ANALYSIS.estimate_slope(self.series(noise))
        short_window = COMMON_ANALYSIS.estimate_slope(self.series(noise[:5]))
        self.assertGreater(
            short_window.detectable_kib_per_min, long_window.detectable_kib_per_min
        )

    def test_noisier_samples_resolve_less(self) -> None:
        quiet = COMMON_ANALYSIS.estimate_slope(
            self.series([100, 101, 99, 100, 101, 99, 100, 101])
        )
        noisy = COMMON_ANALYSIS.estimate_slope(
            self.series([100, 140, 60, 100, 140, 60, 100, 140])
        )
        self.assertGreater(noisy.detectable_kib_per_min, quiet.detectable_kib_per_min)

    def test_two_samples_resolve_nothing(self) -> None:
        estimate = COMMON_ANALYSIS.estimate_slope(self.series([100, 200]))
        self.assertFalse(estimate.resolvable)
        self.assertEqual(math.inf, estimate.detectable_kib_per_min)

    def test_a_real_slope_is_recovered_and_exceeds_the_floor(self) -> None:
        estimate = COMMON_ANALYSIS.estimate_slope(
            self.series([100, 610, 1_190, 1_810, 2_390, 3_010, 3_590, 4_210])
        )
        self.assertAlmostEqual(36_000.0, estimate.slope_kib_per_min, delta=600.0)
        self.assertGreater(estimate.slope_kib_per_min, estimate.detectable_kib_per_min)

    def test_the_report_publishes_a_floor_beside_every_slope(self) -> None:
        run_dir = GraphRunBuilder(self.root / "run-a").write(
            rounds=20,
            pss_for=segmented_pss,
            cpu_for=segmented_cpu,
            segment_rounds=[("motion", 0, 9), ("settled", 10, 19)],
        )
        result = ANALYZER.analyze_run(run_dir)
        growth = result["nodes"][PRODUCT_NODES[1]]["memory"]["pss_kib"]["growth"]
        self.assertIn("detectable_slope_kib_per_min", growth)
        self.assertIn("residual_stddev_kib", growth)
        self.assertIn("does not establish that there is none", growth["sensitivity_note"])
        settled = result["leak_sensitivity_by_segment"]["settled"]
        self.assertTrue(settled["all_nodes_resolvable"])
        self.assertGreater(settled["pss_detectable_slope_kib_per_min"], 0.0)
        self.assertIn("leak_sensitivity", result)

    def test_a_flat_run_still_states_what_it_could_not_have_seen(self) -> None:
        results = []
        for index, name in enumerate(("run-a", "run-b", "run-c")):
            run_dir = GraphRunBuilder(self.root / name).write(
                t0_ns=1_000_000_000_000 + index * 10_000_000_000_000,
                pid_offset=index * 10,
                pss_for=lambda node, round_index: BASE_PSS_KIB[node]
                + SETTLED_NOISE_KIB[round_index % len(SETTLED_NOISE_KIB)],
            )
            results.append(ANALYZER.analyze_run(run_dir))
        aggregate = ANALYZER.aggregate_runs(results)
        self.assertFalse(aggregate["any_node_suspected_sustained_growth"])
        self.assertTrue(aggregate["leak_sensitivity"]["all_nodes_resolvable"])
        self.assertGreater(
            aggregate["leak_sensitivity"]["pss_detectable_slope_kib_per_min"], 0.0
        )
        node = aggregate["nodes"][PRODUCT_NODES[1]]
        self.assertEqual(3, len(node["pss_detectable_slope_kib_per_min_by_run"]))
        self.assertEqual(
            max(node["pss_detectable_slope_kib_per_min_by_run"]),
            node["pss_detectable_slope_kib_per_min"],
        )


class SegmentDerivationTest(unittest.TestCase):
    """The runner and the analyzer must agree on what a segment boundary is."""

    T0 = 1_000_000_000_000
    T1 = T0 + 50_000_000_000
    LATCH = T0 + 20_000_000_000

    def test_an_observed_latch_splits_the_window_and_drops_the_drain(self) -> None:
        segments = COMMON_ANALYSIS.derive_segments(
            self.T0, self.T1, self.LATCH, drain_s=0.5, minimum_duration_s=3.0
        )
        self.assertEqual(["motion", "settled"], [segment.name for segment in segments])
        self.assertEqual((self.T0, self.LATCH), (segments[0].t0_ns, segments[0].t1_ns))
        self.assertEqual(self.LATCH + 500_000_000, segments[1].t0_ns)
        self.assertEqual(self.T1, segments[1].t1_ns)
        self.assertLess(segments[0].t1_ns, segments[1].t0_ns)

    def test_without_an_observed_latch_the_window_stays_whole(self) -> None:
        self.assertEqual(
            [], COMMON_ANALYSIS.derive_segments(self.T0, self.T1, None, drain_s=0.5)
        )

    def test_a_latch_outside_the_window_splits_nothing(self) -> None:
        self.assertEqual(
            [],
            COMMON_ANALYSIS.derive_segments(self.T0, self.T1, self.T1 + 1, drain_s=0.5),
        )

    def test_a_segment_below_the_statistics_minimum_is_dropped(self) -> None:
        segments = COMMON_ANALYSIS.derive_segments(
            self.T0,
            self.LATCH + 1_500_000_000,
            self.LATCH,
            drain_s=0.5,
            minimum_duration_s=3.0,
        )
        self.assertEqual(["motion"], [segment.name for segment in segments])

    def test_rendered_boundaries_survive_a_round_trip(self) -> None:
        segments = COMMON_ANALYSIS.derive_segments(
            self.T0, self.T1, self.LATCH, drain_s=0.5, boundary_source="observed_latch"
        )
        rendered = COMMON_ANALYSIS.render_segment_manifest(segments)
        parsed = COMMON_ANALYSIS.parse_segments(
            COMMON_ANALYSIS.parse_manifest_text(rendered), self.T0, self.T1
        )
        self.assertEqual(segments, parsed)

    def test_the_derivation_is_reachable_from_the_runner_cli(self) -> None:
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPT_DIR / "lib" / "graph_run_gates.py"),
                "segments",
                str(self.T0),
                str(self.T1),
                "--latch-monotonic-ns",
                str(self.LATCH),
                "--drain-s",
                "0.5",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn("segment_count=2", result.stdout)
        self.assertIn("segment_0_name=motion", result.stdout)


class ReadinessGateTest(TemporaryRootTest):
    """A recorder stops discovery once it has subscribed to whatever it found."""

    ALL_TOPICS = ["/cave_scene/raw_scan", "/cave_scene/odom", "/tf", "/tf_static"]

    def record_log(self, topics: list[str]) -> str:
        return "\n".join(
            f"[INFO] [17.1] [rosbag2_recorder]: Subscribed to topic '{topic}'"
            for topic in topics
        )

    def test_a_fully_subscribed_recorder_passes(self) -> None:
        self.assertEqual(
            [],
            GATES.missing_subscriptions(self.record_log(self.ALL_TOPICS), self.ALL_TOPICS),
        )

    def test_a_partially_subscribed_recorder_is_rejected(self) -> None:
        missing = GATES.missing_subscriptions(
            self.record_log(self.ALL_TOPICS[:2]), self.ALL_TOPICS
        )
        self.assertEqual(["/tf", "/tf_static"], missing)

    def test_a_recorder_that_subscribed_to_nothing_is_rejected(self) -> None:
        self.assertEqual(
            sorted(self.ALL_TOPICS),
            GATES.missing_subscriptions("waiting for topics", self.ALL_TOPICS),
        )

    def test_the_gate_cli_fails_and_names_every_missing_topic(self) -> None:
        log = self.root / "record-s1.log"
        log.write_text(self.record_log(self.ALL_TOPICS[:1]), encoding="utf-8")
        report = self.root / "subscription-gate.txt"
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPT_DIR / "lib" / "graph_run_gates.py"),
                "--report",
                str(report),
                "subscriptions",
                str(log),
                *self.ALL_TOPICS,
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(1, result.returncode)
        rendered = report.read_text(encoding="utf-8")
        self.assertIn("gate_pass=false", rendered)
        self.assertIn("missing_topic=/cave_scene/odom", rendered)
        self.assertIn("missing_topic=/tf_static", rendered)

    def test_a_missing_node_stops_the_run_before_the_window_opens(self) -> None:
        proc = ProcFixture(self.root / "proc")
        if not SYMLINKS_SUPPORTED:
            self.skipTest("a fabricated /proc tree needs symlink support for exe")
        proc.add(NODE_PIDS[PRODUCT_NODES[0]], PRODUCT_NODES[0])
        result = subprocess.run(
            [
                sys.executable,
                str(RESOLVER_PATH),
                "--product",
                PRODUCT_NODES[0],
                "--product",
                PRODUCT_NODES[1],
                "--proc-root",
                str(proc.root),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(1, result.returncode)
        self.assertIn("not running", result.stderr)
        self.assertIn(PRODUCT_NODES[1], result.stderr)


class TeardownGateTest(TemporaryRootTest):
    """A recorder cleaned up before it exits leaves an unreadable bag."""

    TOPICS = ["/cave_scene/perception/observations", "/cave_scene/perception/pose"]

    def bag_info(self, counts: dict[str, int]) -> str:
        lines = [
            "Files:             bag_0.mcap",
            "Storage id:        mcap",
            "Duration:          49.0s",
            f"Messages:          {sum(counts.values())}",
            "Topic information: "
            + "\n                   ".join(
                f"Topic: {topic} | Type: perception_interfaces/msg/LidarObservation "
                f"| Count: {count} | Serialization Format: cdr"
                for topic, count in counts.items()
            ),
        ]
        return "\n".join(lines) + "\n"

    def make_bag(self, *, metadata: bool = True) -> Path:
        bag_dir = self.root / "bag-s3"
        bag_dir.mkdir(exist_ok=True)
        (bag_dir / "bag_0.mcap").write_bytes(b"payload")
        if metadata:
            (bag_dir / "metadata.yaml").write_text("version: 9\n", encoding="utf-8")
        return bag_dir

    def test_a_finalized_and_readable_bag_passes(self) -> None:
        reasons = GATES.bag_completion_reasons(
            self.make_bag(), self.bag_info({topic: 200 for topic in self.TOPICS}), self.TOPICS
        )
        self.assertEqual([], reasons)

    def test_a_bag_without_metadata_is_rejected(self) -> None:
        reasons = GATES.bag_completion_reasons(
            self.make_bag(metadata=False),
            self.bag_info({topic: 200 for topic in self.TOPICS}),
            self.TOPICS,
        )
        self.assertTrue(any("metadata.yaml is missing" in reason for reason in reasons))
        self.assertTrue(any("unreadable" in reason for reason in reasons))

    def test_a_bag_info_that_did_not_parse_is_rejected(self) -> None:
        reasons = GATES.bag_completion_reasons(
            self.make_bag(), "Could not open bag: no metadata.yaml", self.TOPICS
        )
        self.assertTrue(any("unreadable" in reason for reason in reasons))

    def test_a_subscribed_but_empty_topic_is_rejected(self) -> None:
        counts = {self.TOPICS[0]: 200, self.TOPICS[1]: 0}
        reasons = GATES.bag_completion_reasons(
            self.make_bag(), self.bag_info(counts), self.TOPICS
        )
        self.assertTrue(any("recorded zero messages" in reason for reason in reasons))

    def test_a_topic_absent_from_the_bag_is_rejected(self) -> None:
        reasons = GATES.bag_completion_reasons(
            self.make_bag(), self.bag_info({self.TOPICS[0]: 200}), self.TOPICS
        )
        self.assertTrue(
            any(reason.startswith(self.TOPICS[1]) for reason in reasons), reasons
        )

    def test_the_gate_cli_reports_the_missing_metadata_and_exits_nonzero(self) -> None:
        bag_dir = self.make_bag(metadata=False)
        info = self.root / "bag-s3-info.txt"
        info.write_text("Could not open bag\n", encoding="utf-8")
        report = self.root / "bag-gate.txt"
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPT_DIR / "lib" / "graph_run_gates.py"),
                "--report",
                str(report),
                "bag",
                str(bag_dir),
                str(info),
                *self.TOPICS,
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(1, result.returncode)
        self.assertIn("gate_pass=false", report.read_text(encoding="utf-8"))
        self.assertIn("metadata.yaml", report.read_text(encoding="utf-8"))

    def test_a_process_group_that_is_not_empty_is_rejected(self) -> None:
        self.assertEqual([], GATES.residual_pids("", []))
        self.assertEqual([4242, 4243], GATES.residual_pids(" 4242\n 4243\n", []))
        self.assertEqual([4243], GATES.residual_pids(" 4242\n 4243\n", [4242]))


class ReplayEquivalenceTest(TemporaryRootTest):
    """Replay stands in for the live graph only if it did the same work."""

    BASELINE = {
        "observation_count": "201",
        "final_revision": "202",
        "final_octomap_data_bytes": "48210",
        "final_octomap_data_sha256": "a" * 64,
        "final_map_epoch": "17",
        "mapper_contract_fingerprint": "fp-1",
        "final_known_bounds_min": "-1.000000,-2.000000,0.000000",
        "final_known_bounds_max": "11.000000,2.000000,3.000000",
    }

    def observed(self, **overrides) -> dict[str, str]:
        values = dict(self.BASELINE)
        values.update(overrides)
        return values

    def test_an_identical_replay_is_equivalent(self) -> None:
        report = EQUIVALENCE.compare_counts(self.BASELINE, self.observed())
        self.assertTrue(report.equivalent)
        self.assertEqual([], report.mismatches)
        self.assertEqual([], report.missing_keys)

    def test_a_different_final_map_digest_is_rejected(self) -> None:
        report = EQUIVALENCE.compare_counts(
            self.BASELINE, self.observed(final_octomap_data_sha256="b" * 64)
        )
        self.assertFalse(report.equivalent)
        self.assertTrue(
            any("final_octomap_data_sha256" in text for text in report.mismatches)
        )

    def test_a_lost_observation_is_rejected_at_zero_tolerance(self) -> None:
        report = EQUIVALENCE.compare_counts(
            self.BASELINE, self.observed(observation_count="200")
        )
        self.assertFalse(report.equivalent)
        self.assertTrue(any("observation_count" in text for text in report.mismatches))

    def test_a_declared_tolerance_is_recorded_in_the_evidence(self) -> None:
        report = EQUIVALENCE.compare_counts(
            self.BASELINE, self.observed(observation_count="200"), count_tolerance=1
        )
        self.assertTrue(report.equivalent)
        self.assertIn("count_tolerance=1", report.render())

    def test_a_missing_key_is_rejected_rather_than_ignored(self) -> None:
        observed = self.observed()
        del observed["final_map_epoch"]
        report = EQUIVALENCE.compare_counts(self.BASELINE, observed)
        self.assertFalse(report.equivalent)
        self.assertIn("final_map_epoch", report.missing_keys)

    def test_a_shrunken_final_map_is_rejected(self) -> None:
        report = EQUIVALENCE.compare_counts(
            self.BASELINE,
            self.observed(
                final_known_bounds_max="8.000000,2.000000,3.000000",
                final_octomap_data_bytes="31000",
                final_octomap_data_sha256="c" * 64,
            ),
        )
        self.assertFalse(report.equivalent)
        self.assertGreaterEqual(len(report.mismatches), 3)

    def write_counts(self, name: str, values: dict[str, str]) -> Path:
        path = self.root / name
        path.write_text(
            "".join(f"{key}={value}\n" for key, value in values.items()), encoding="utf-8"
        )
        return path

    def test_the_cli_exits_nonzero_on_a_mismatch(self) -> None:
        baseline = self.write_counts("baseline.txt", self.BASELINE)
        observed = self.write_counts("observed.txt", self.observed(final_revision="150"))
        report = self.root / "replay-equivalence.txt"
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPT_DIR / "lib" / "graph_replay_equivalence.py"),
                str(baseline),
                str(observed),
                str(report),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(1, result.returncode)
        rendered = report.read_text(encoding="utf-8")
        self.assertIn("replay_equivalent=false", rendered)
        self.assertIn("final_revision", rendered)

    def test_the_analyzer_rejects_a_replay_run_that_was_not_equivalent(self) -> None:
        run_dir = GraphRunBuilder(self.root / "run-replay").write(
            mode="graph-replay",
            replay_equivalence=(
                "replay_equivalent=false\ncount_tolerance=0\n"
                "mismatch=observation_count: direct run 201 vs replay 150\n"
            ),
        )
        with self.assertRaises(ValueError) as raised:
            ANALYZER.analyze_run(run_dir)
        message = str(raised.exception)
        self.assertIn("did not reproduce the direct run", message)
        self.assertIn("observation_count", message)

    def test_the_analyzer_requires_equivalence_evidence_from_a_replay_run(self) -> None:
        run_dir = GraphRunBuilder(self.root / "run-replay").write(mode="graph-replay")
        with self.assertRaises(ValueError) as raised:
            ANALYZER.analyze_run(run_dir)
        self.assertIn("must carry replay-equivalence.txt", str(raised.exception))

    def test_an_equivalent_replay_run_is_accepted(self) -> None:
        run_dir = GraphRunBuilder(self.root / "run-replay").write(
            mode="graph-replay",
            replay_equivalence=(
                "replay_equivalent=true\ncount_tolerance=0\ncompared_key_count=8\n"
            ),
        )
        result = ANALYZER.analyze_run(run_dir)
        self.assertTrue(result["replay_equivalence"]["equivalent"])
        self.assertEqual("0", result["replay_equivalence"]["count_tolerance"])

    def test_looped_replay_states_that_equivalence_was_not_established(self) -> None:
        run_dir = GraphRunBuilder(self.root / "run-loop").write(mode="graph-replay-loop")
        result = ANALYZER.analyze_run(run_dir)
        self.assertIsNone(result["replay_equivalence"]["equivalent"])
        self.assertIn(
            "growth", result["replay_equivalence"]["not_established_reason"]
        )


class ReuseAndRegressionTest(unittest.TestCase):
    def test_graph_sampler_reuses_single_pid_proc_readers(self) -> None:
        self.assertIs(SINGLE_PID_SAMPLER.parse_smaps_rollup, GRAPH.parse_smaps_rollup)
        self.assertIs(SINGLE_PID_SAMPLER.read_headroom, GRAPH.read_headroom)
        self.assertIs(SINGLE_PID_SAMPLER.read_cgroup_v2_path, GRAPH.read_cgroup_v2_path)
        self.assertIs(ROLE_MONITOR.read_process_state, GRAPH.read_process_state)
        source = Path(GRAPH.__file__).read_text(encoding="utf-8")
        for name in (
            "parse_smaps_rollup",
            "read_headroom",
            "read_cgroup_v2_path",
            "parse_mem_available_kib",
            "read_process_state",
        ):
            self.assertNotIn(f"def {name}(", source)

    def test_analyzer_reuses_shared_pidstat_and_slope_helpers(self) -> None:
        self.assertIs(COMMON_ANALYSIS.parse_pidstat_samples, ANALYZER.parse_pidstat_samples)
        self.assertIs(COMMON_ANALYSIS.slope_kib_per_minute, ANALYZER.slope_kib_per_minute)
        self.assertIs(COMMON_ANALYSIS.estimate_slope, ANALYZER.estimate_slope)
        self.assertIs(COMMON_ANALYSIS.parse_segments, ANALYZER.parse_segments)
        self.assertIs(
            COMMON_ANALYSIS.percentile_nearest_rank, ANALYZER.percentile_nearest_rank
        )
        source = ANALYZER_PATH.read_text(encoding="utf-8")
        self.assertNotIn("%CPU", source)
        self.assertNotIn("def slope", source)

    def test_the_runner_and_the_analyzer_share_one_segment_definition(self) -> None:
        self.assertIs(COMMON_ANALYSIS.derive_segments, GATES.derive_segments)
        self.assertIs(COMMON_ANALYSIS.render_segment_manifest, GATES.render_segment_manifest)
        self.assertIs(COMMON_ANALYSIS.parse_manifest, EQUIVALENCE.parse_manifest)
        for module_path in (GATES.__file__, EQUIVALENCE.__file__):
            source = Path(module_path).read_text(encoding="utf-8")
            self.assertNotIn("def parse_manifest(", source)

    def test_the_runner_never_reaches_for_pkill(self) -> None:
        source = RUNNER_PATH.read_text(encoding="utf-8")
        # pkill -f matches the container's own exec wrapper and kills the caller;
        # teardown is by process group only.
        self.assertNotIn("pkill", source)
        self.assertIn("setsid", source)

    def test_single_pid_path_keeps_its_public_entry_points(self) -> None:
        for name in ("sample_loop", "parse_smaps_rollup", "read_headroom", "safety_reason"):
            self.assertTrue(hasattr(SINGLE_PID_SAMPLER, name), name)
        signature = SINGLE_PID_SAMPLER.sample_loop.__code__.co_varnames[
            : SINGLE_PID_SAMPLER.sample_loop.__code__.co_argcount
        ]
        self.assertEqual(("args",), signature)
        self.assertEqual(
            (
                "receipt_monotonic_ns",
                "state_receipt_monotonic_ns",
                "map_epoch",
                "revision",
                "stamp_ns",
                "rss_kib",
                "pss_kib",
                "uss_kib",
                "cgroup_current_bytes",
                "cgroup_max_bytes",
                "mem_available_kib",
                "oom",
                "oom_kill",
            ),
            SINGLE_PID_SAMPLER.RESOURCE_COLUMNS,
        )


if __name__ == "__main__":
    unittest.main()
