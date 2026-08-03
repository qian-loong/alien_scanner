import gzip
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).with_name("summarize-perception-coverage.py")
SPEC = importlib.util.spec_from_file_location(
    "summarize_perception_coverage", SCRIPT_PATH
)
assert SPEC is not None and SPEC.loader is not None
SUMMARIZER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SUMMARIZER)


class SummarizePerceptionCoverageTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.root = Path(self.temporary_directory.name)
        self.package_root = self.root / "source" / "sample_package"
        self.gcov_root = self.root / "gcov"
        (self.package_root / "include" / "sample_package").mkdir(parents=True)
        (self.package_root / "include" / "generated").mkdir()
        (self.package_root / "src").mkdir()
        (self.package_root / "src" / "external").mkdir()
        (self.package_root / "test").mkdir()
        self.gcov_root.mkdir()

    def write_gcov(self, name: str, files: list[dict]) -> None:
        document = {
            "format_version": "1",
            "gcc_version": "test",
            "current_working_directory": str(self.root),
            "data_file": name,
            "files": files,
        }
        with gzip.open(
            self.gcov_root / f"{name}.gcov.json.gz", "wt", encoding="utf-8"
        ) as stream:
            json.dump(document, stream)

    @staticmethod
    def source(path: Path, lines: list[tuple[int, int]]) -> dict:
        return {
            "file": str(path),
            "functions": [],
            "lines": [
                {
                    "line_number": line_number,
                    "count": count,
                    "unexecuted_block": count == 0,
                    "branches": [],
                }
                for line_number, count in lines
            ],
        }

    def test_realpath_line_union_uses_any_executed_instance(self) -> None:
        header = self.package_root / "include" / "sample_package" / "api.hpp"
        source = self.package_root / "src" / "algorithm.cpp"
        test_source = self.package_root / "test" / "test_algorithm.cpp"
        node_source = self.package_root / "src" / "SampleNode.cpp"
        generated_source = self.package_root / "include" / "generated" / "schema.hpp"
        embedded_external = self.package_root / "src" / "external" / "vendor.cpp"
        external = self.root / "external.hpp"
        self.write_gcov(
            "first",
            [
                self.source(header, [(10, 0), (11, 2)]),
                self.source(source, [(20, 0)]),
                self.source(test_source, [(1, 1)]),
                self.source(node_source, [(1, 1)]),
                self.source(generated_source, [(1, 1)]),
                self.source(embedded_external, [(1, 1)]),
                self.source(external, [(1, 1)]),
            ],
        )
        self.write_gcov(
            "second",
            [
                self.source(header, [(10, 3), (11, 0)]),
                self.source(source, [(20, 0)]),
            ],
        )

        summary = SUMMARIZER.summarize_package(
            self.package_root, self.gcov_root
        )

        self.assertEqual(2, summary.covered)
        self.assertEqual(3, summary.total)

    def test_empty_gcov_directory_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "no gcov JSON files"):
            SUMMARIZER.summarize_package(self.package_root, self.gcov_root)

    def test_package_spec_validates_threshold(self) -> None:
        package = SUMMARIZER.parse_package_spec("core:core-json:90")
        self.assertEqual("core", package.name)
        self.assertEqual("core-json", package.gcov_subdir)
        self.assertEqual(90.0, package.minimum_percent)
        with self.assertRaisesRegex(Exception, "within"):
            SUMMARIZER.parse_package_spec("core:core-json:101")


if __name__ == "__main__":
    unittest.main()
