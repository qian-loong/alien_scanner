"""Validate C2 target builds and capture the fixed calibration closure."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Any


PERFORMANCE_FLAGS = {
    "-O2",
    "-g",
    "-DNDEBUG",
    "-fno-omit-frame-pointer",
}
SANITIZER_FLAGS = {
    "-O1",
    "-g",
    "-DNDEBUG",
    "-fno-omit-frame-pointer",
    "-fsanitize=address",
}
OPTIMIZATION_FLAGS = {
    "-O0",
    "-O1",
    "-O2",
    "-O3",
    "-Os",
    "-Oz",
    "-Ofast",
}
TRUE_VALUES = {"1", "ON", "TRUE", "YES", "Y"}
FALSE_VALUES = {"0", "OFF", "FALSE", "NO", "N"}

WORKSPACE_DEPENDENCY_PACKAGES = ("perception_core", "perception_interfaces")
NON_TARGET_WORKSPACE_PACKAGES = (
    "cave_world",
    "drone_scanner",
    "perception_adapters",
    "perception_fixtures",
    "perception_input_node",
    "perception_profiling",
)
PROFILING_HELPERS = {
    "fixture": Path("lib/perception_profiling/perception_profile_fixture"),
    "oracle": Path("lib/perception_profiling/perception_profile_oracle"),
    "sink": Path("lib/perception_profiling/perception_profile_sink"),
}
PROFILING_WORKLOAD = Path("share/perception_profiling/config/profile_local_map.yaml")
FORBIDDEN_MAIN_WORKSPACE_INSTALL = "/workspaces/alien-scanner/ws/install"
WORKSPACE_SOURCE_ROOT = "/workspaces/alien-scanner/ws/src"
SOURCE_IDENTITY_KEYS = (
    "schema_version",
    "source_revision",
    "source_diff_sha256",
    "source_untracked_sha256",
    "source_untracked_archive_sha256",
    "paired_source_identity_sha256",
)
CLOSURE_KEYS = {
    "schema_version",
    "paired_source_identity_sha256",
    "closure_install_base",
    "closure_build_base",
    "dependencies",
    "helpers",
    "workload",
    "dependency_comparison_sha256",
    "helper_set_sha256",
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_json_bytes(value: Any) -> bytes:
    return (
        json.dumps(value, ensure_ascii=True, separators=(",", ":"), sort_keys=True)
        + "\n"
    ).encode("utf-8")


def _canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def _parse_cmake_cache(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith(("#", "//")) or "=" not in line:
            continue
        key_and_type, value = line.split("=", 1)
        key = key_and_type.split(":", 1)[0]
        values[key] = value
    return values


def _load_compile_commands(path: Path) -> list[dict[str, Any]]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid compile_commands.json: {error}") from error
    if not isinstance(payload, list) or not all(
        isinstance(entry, dict) for entry in payload
    ):
        raise ValueError("compile_commands.json root must be a list of objects")
    return payload


def _compile_arguments(entry: dict[str, Any]) -> list[str]:
    arguments = entry.get("arguments")
    if isinstance(arguments, list) and all(isinstance(value, str) for value in arguments):
        return arguments
    command = entry.get("command")
    if not isinstance(command, str):
        raise ValueError("target compile entry lacks command or arguments")
    return shlex.split(command)


def _target_compile_entry(path: Path) -> dict[str, Any]:
    matches = []
    for entry in _load_compile_commands(path):
        source = entry.get("file")
        output = entry.get("output", "")
        command = entry.get("command", "")
        if (
            isinstance(source, str)
            and Path(source).name == "PerceptionLocalMapNode.cpp"
            and "perception_local_map_node.dir" in f"{output} {command}"
        ):
            matches.append(entry)
    if len(matches) != 1:
        raise ValueError(
            "compile_commands.json must contain exactly one local-map node compile entry"
        )
    return matches[0]


def _validate_compile_flags(
    arguments: list[str], required_flags: set[str], expected_optimization: str, label: str
) -> None:
    argument_set = set(arguments)
    missing_flags = sorted(required_flags - argument_set)
    if missing_flags:
        raise ValueError(f"{label} compile command lacks required flags: " + ", ".join(missing_flags))
    conflicting_optimizations = sorted(
        (argument_set & OPTIMIZATION_FLAGS) - {expected_optimization}
    )
    if conflicting_optimizations:
        raise ValueError(
            f"{label} compile command contains conflicting optimization flags: "
            + ", ".join(conflicting_optimizations)
        )
    if "-fomit-frame-pointer" in argument_set:
        raise ValueError(f"{label} compile command enables frame-pointer omission")


def validate_build(
    build_directory: Path, build_profile: str, expected_stage_option: str
) -> dict[str, str]:
    build_directory = build_directory.resolve(strict=True)
    cache_path = build_directory / "CMakeCache.txt"
    commands_path = build_directory / "compile_commands.json"
    if not cache_path.is_file() or not commands_path.is_file():
        raise ValueError("target build evidence lacks CMakeCache.txt or compile_commands.json")
    if build_profile not in {"performance", "sanitizer"}:
        raise ValueError(f"unknown build profile: {build_profile}")
    if expected_stage_option not in {"ON", "OFF"}:
        raise ValueError(f"invalid expected stage option: {expected_stage_option}")

    cache = _parse_cmake_cache(cache_path)
    build_type = cache.get("CMAKE_BUILD_TYPE")
    if build_type != "RelWithDebInfo":
        raise ValueError(f"target build type is {build_type!r}, expected RelWithDebInfo")

    raw_stage_option = cache.get("PERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS")
    if raw_stage_option is None:
        raise ValueError("target build cache lacks the stage latency option")
    normalized_stage_option = raw_stage_option.upper()
    if normalized_stage_option in TRUE_VALUES:
        stage_option = "ON"
    elif normalized_stage_option in FALSE_VALUES:
        stage_option = "OFF"
    else:
        raise ValueError(f"target stage latency option is invalid: {raw_stage_option!r}")
    if stage_option != expected_stage_option:
        raise ValueError(
            f"target stage latency option is {stage_option}, expected {expected_stage_option}"
        )

    arguments = _compile_arguments(_target_compile_entry(commands_path))
    required_flags = PERFORMANCE_FLAGS if build_profile == "performance" else SANITIZER_FLAGS
    expected_optimization = "-O2" if build_profile == "performance" else "-O1"
    _validate_compile_flags(arguments, required_flags, expected_optimization, "target")
    if build_profile == "performance" and "-fsanitize=address" in set(arguments):
        raise ValueError("performance target compile command contains address sanitizer")

    return {
        "target_build_directory": str(build_directory),
        "target_build_type": build_type,
        "target_compile_flags_verified": "true",
        "target_stage_latency_option": stage_option,
    }


def source_identity_from_files(
    source_revision: str,
    source_diff: Path,
    source_untracked: Path,
    source_untracked_archive: Path,
) -> dict[str, str]:
    if re.fullmatch(r"[0-9a-f]{40}", source_revision) is None:
        raise ValueError("source revision must be a 40-character lowercase Git SHA")
    values = {
        "schema_version": "1",
        "source_revision": source_revision,
        "source_diff_sha256": sha256_file(source_diff.resolve(strict=True)),
        "source_untracked_sha256": sha256_file(source_untracked.resolve(strict=True)),
        "source_untracked_archive_sha256": sha256_file(
            source_untracked_archive.resolve(strict=True)
        ),
    }
    values["paired_source_identity_sha256"] = _canonical_sha256(values)
    return values


def _parse_values(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or "=" not in line:
            raise ValueError(f"invalid key/value record in {path}")
        key, value = line.split("=", 1)
        if key in values:
            raise ValueError(f"duplicate key {key!r} in {path}")
        values[key] = value
    return values


def load_source_identity(path: Path) -> dict[str, str]:
    path = path.resolve(strict=True)
    values = _parse_values(path)
    if tuple(values) != SOURCE_IDENTITY_KEYS:
        raise ValueError("paired source identity has missing, extra, or reordered fields")
    if values["schema_version"] != "1":
        raise ValueError("paired source identity schema must be 1")
    if re.fullmatch(r"[0-9a-f]{40}", values["source_revision"]) is None:
        raise ValueError("paired source identity revision is not a lowercase Git SHA")
    for key in SOURCE_IDENTITY_KEYS[2:]:
        if re.fullmatch(r"[0-9a-f]{64}", values[key]) is None:
            raise ValueError(f"paired source identity field {key} is not a SHA-256")
    expected = _canonical_sha256(
        {key: values[key] for key in SOURCE_IDENTITY_KEYS[:-1]}
    )
    if values["paired_source_identity_sha256"] != expected:
        raise ValueError("paired source identity digest does not match its fields")
    return values


def _require_within(
    path: Path,
    base: Path,
    label: str,
    base_description: str = "closure install base",
) -> Path:
    resolved = path.resolve(strict=True)
    try:
        resolved.relative_to(base)
    except ValueError as error:
        raise ValueError(f"{label} resolves outside {base_description}: {path}") from error
    return resolved


def _gnu_build_id(path: Path) -> str | None:
    with path.open("rb") as stream:
        if stream.read(4) != b"\x7fELF":
            return None
    result = subprocess.run(
        ["readelf", "-n", str(path)],
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        raise ValueError(f"readelf failed for closure ELF: {path}")
    match = re.search(r"Build ID:\s*([0-9a-f]+)", result.stdout)
    if match is None:
        raise ValueError(f"closure ELF lacks a GNU build ID: {path}")
    return match.group(1)


def _artifact_record(
    path: Path, install_base: Path, kind: str, *, require_build_id: bool = False
) -> dict[str, Any]:
    if not path.exists() and not path.is_symlink():
        raise ValueError(f"closure artifact is missing: {path}")
    if path.is_symlink() and not path.exists():
        raise ValueError(f"closure artifact is a broken symlink: {path}")
    realpath = _require_within(path, install_base, "closure artifact")
    if not realpath.is_file():
        raise ValueError(f"closure artifact is not a regular file: {path}")
    build_id = _gnu_build_id(realpath)
    if require_build_id and build_id is None:
        raise ValueError(f"closure helper lacks an ELF build ID: {path}")
    return {
        "relative_path": path.relative_to(install_base).as_posix(),
        "realpath": str(realpath),
        "kind": kind,
        "size": realpath.stat().st_size,
        "sha256": sha256_file(realpath),
        "build_id": build_id,
    }


def _files_below(root: Path) -> list[Path]:
    if not root.is_dir():
        raise ValueError(f"closure header root is missing: {root}")
    files: list[Path] = []
    for path in root.rglob("*"):
        if path.is_dir() and not path.is_symlink():
            continue
        if not path.is_file() and not path.is_symlink():
            raise ValueError(f"closure artifact is not a file: {path}")
        files.append(path)
    if not files:
        raise ValueError(f"closure header root is empty: {root}")
    return files


def _normalize_compile_token(token: str, build_dir: Path, install_base: Path) -> str:
    return (
        token.replace(str(build_dir), "$BUILD")
        .replace(str(install_base), "$INSTALL")
        .replace(WORKSPACE_SOURCE_ROOT, "$SOURCE")
    )


def _dependency_compile_profile(
    package: str, build_dir: Path, install_base: Path
) -> tuple[int, str]:
    commands_path = build_dir / "compile_commands.json"
    entries = _load_compile_commands(commands_path)
    selected: list[dict[str, Any]] = []
    for entry in entries:
        source = entry.get("file")
        output = entry.get("output", "")
        if not isinstance(source, str) or not isinstance(output, str):
            continue
        normalized = f"/{output.lower().replace(chr(92), '/')}"
        if any(marker in normalized for marker in ("/test", "/gtest")):
            continue
        if package == "perception_core":
            if "CMakeFiles/perception_core.dir/" not in output:
                continue
        elif "CMakeFiles/perception_interfaces" not in output:
            continue
        arguments = _compile_arguments(entry)
        _validate_compile_flags(arguments, PERFORMANCE_FLAGS, "-O2", package)
        if any(argument.startswith("-fsanitize=") for argument in arguments):
            raise ValueError(f"{package} compile command contains a sanitizer")
        selected.append(
            {
                "source": _normalize_compile_token(source, build_dir, install_base),
                "output": _normalize_compile_token(output, build_dir, install_base),
                "arguments": [
                    _normalize_compile_token(argument, build_dir, install_base)
                    for argument in arguments
                ],
            }
        )
    if not selected:
        raise ValueError(f"{package} has no non-test compile entries")
    selected.sort(key=lambda value: (value["source"], value["output"]))
    return len(selected), _canonical_sha256(selected)


def _reject_forbidden_text(text: str, label: str) -> None:
    if FORBIDDEN_MAIN_WORKSPACE_INSTALL in text:
        raise ValueError(f"{label} references forbidden main-workspace install")


def _dependency_record(
    package: str, install_base: Path, build_base: Path
) -> dict[str, Any]:
    prefix = _require_within(install_base / package, install_base, package)
    build_dir = _require_within(
        build_base / package,
        build_base,
        f"{package} build directory",
        "closure build base",
    )
    cache_path = _require_within(
        build_dir / "CMakeCache.txt",
        build_base,
        f"{package} CMake cache",
        "closure build base",
    )
    commands_path = _require_within(
        build_dir / "compile_commands.json",
        build_base,
        f"{package} compile database",
        "closure build base",
    )
    if not cache_path.is_file() or not commands_path.is_file():
        raise ValueError(f"{package} build evidence is incomplete")
    cache_text = cache_path.read_text(encoding="utf-8")
    commands_text = commands_path.read_text(encoding="utf-8")
    _reject_forbidden_text(cache_text, f"{package} CMake cache")
    _reject_forbidden_text(commands_text, f"{package} compile database")
    cache = _parse_cmake_cache(cache_path)
    if cache.get("CMAKE_BUILD_TYPE") != "RelWithDebInfo":
        raise ValueError(f"{package} build type is not RelWithDebInfo")
    expected_prefix = str(prefix)
    if cache.get("CMAKE_INSTALL_PREFIX") != expected_prefix:
        raise ValueError(f"{package} install prefix does not match closure")
    entry_count, compile_profile = _dependency_compile_profile(
        package, build_dir, install_base
    )

    if package == "perception_core":
        paths = [
            (path, "header", False)
            for path in _files_below(prefix / "include/perception_core")
        ]
        paths.append((prefix / "lib/libperception_core.a", "static_library", False))
    else:
        paths = [
            (path, "generated_header", False)
            for path in _files_below(prefix / "include/perception_interfaces")
        ]
        libraries = sorted((prefix / "lib").glob("libperception_interfaces*"))
        if not libraries:
            raise ValueError("perception_interfaces installed libraries are missing")
        paths.extend(
            (
                path,
                "static_library" if path.name.endswith(".a") else "shared_library",
                not path.name.endswith(".a"),
            )
            for path in libraries
        )
    artifacts = [
        _artifact_record(path, install_base, kind, require_build_id=require_build_id)
        for path, kind, require_build_id in paths
    ]
    artifacts.sort(key=lambda value: value["relative_path"])
    relative_paths = [artifact["relative_path"] for artifact in artifacts]
    realpaths = [artifact["realpath"] for artifact in artifacts]
    if len(relative_paths) != len(set(relative_paths)) or len(realpaths) != len(set(realpaths)):
        raise ValueError(f"{package} closure contains duplicate artifacts")
    return {
        "package": package,
        "prefix": str(prefix),
        "build_directory": str(build_dir),
        "build_type": "RelWithDebInfo",
        "cmake_cache_sha256": sha256_file(cache_path),
        "compile_commands_sha256": sha256_file(commands_path),
        "compile_entry_count": entry_count,
        "compile_profile_sha256": compile_profile,
        "artifacts": artifacts,
    }


PRODUCTION_LOCAL_MAP_COMPILE_MARKERS = (
    "CMakeFiles/perception_local_map_core.dir/",
    "CMakeFiles/perception_local_map_octomap.dir/",
    "CMakeFiles/perception_local_map_node.dir/",
)
PRODUCTION_LOCAL_MAP_LINK_PATHS = (
    "CMakeFiles/perception_local_map_core.dir/link.txt",
    "CMakeFiles/perception_local_map_octomap.dir/link.txt",
    "CMakeFiles/perception_local_map_node.dir/link.txt",
)


def _is_production_local_map_compile_entry(entry: dict[str, Any]) -> bool:
    output = entry.get("output")
    source = entry.get("file")
    if isinstance(output, str) and output.strip():
        normalized = output.replace("\\", "/")
        return any(marker in normalized for marker in PRODUCTION_LOCAL_MAP_COMPILE_MARKERS)
    if isinstance(source, str) and source.strip():
        normalized = f"/{source.replace(chr(92), '/').lower()}"
        if any(
            marker in normalized
            for marker in ("/test/", "/gtest/", "testlocalmap", "testcave")
        ):
            return False
        return "/perception_local_map/src/" in normalized
    return False


def _production_target_evidence_text(
    target_build_directory: Path, compile_commands_path: Path
) -> str:
    entries = _load_compile_commands(compile_commands_path)
    fragments: list[str] = []
    for entry in entries:
        if not _is_production_local_map_compile_entry(entry):
            continue
        command = entry.get("command")
        if isinstance(command, str) and command.strip():
            fragments.append(command)
        arguments = entry.get("arguments")
        if isinstance(arguments, list):
            fragments.append(
                " ".join(str(argument) for argument in arguments if argument is not None)
            )
        for key in ("file", "output"):
            value = entry.get(key)
            if isinstance(value, str) and value.strip():
                fragments.append(value)
    for relative in PRODUCTION_LOCAL_MAP_LINK_PATHS:
        path = target_build_directory / relative
        if not path.exists():
            continue
        path = _require_within(
            path,
            target_build_directory,
            "production target link evidence",
            "target build directory",
        )
        if not path.is_file():
            raise ValueError(f"production target link evidence is not a file: {path}")
        fragments.append(path.read_text(encoding="utf-8"))
    if not fragments:
        raise ValueError("production target compile/link evidence is missing")
    return "\n".join(fragments)


def _validate_target_closure_resolution(
    target_build_directory: Path,
    target_executable: Path,
    closure_install_base: Path,
) -> None:
    target_build_directory = target_build_directory.resolve(strict=True)
    target_executable = target_executable.resolve(strict=True)
    evidence_paths = (
        target_build_directory / "CMakeCache.txt",
        target_build_directory / "compile_commands.json",
        target_build_directory / "CMakeFiles/perception_local_map_node.dir/link.txt",
    )
    texts = []
    for path in evidence_paths:
        path = _require_within(
            path,
            target_build_directory,
            "target closure resolution evidence",
            "target build directory",
        )
        if not path.is_file():
            raise ValueError(f"target closure resolution evidence is missing: {path}")
        text = path.read_text(encoding="utf-8")
        _reject_forbidden_text(text, f"target evidence {path.name}")
        texts.append(text)
    link_text = texts[-1]
    expected_core = str(
        closure_install_base / "perception_core/lib/libperception_core.a"
    )
    expected_interfaces = str(closure_install_base / "perception_interfaces/lib/")
    if expected_core not in link_text or expected_interfaces not in link_text:
        raise ValueError("target link command does not consume the declared closure")
    # Only production local-map targets are in the fixed calibration domain.
    # Package tests may link cave_world/drone_scanner/fixtures and must not
    # poison the formal target dependency gate.
    compile_and_link_text = _production_target_evidence_text(
        target_build_directory, evidence_paths[1]
    )
    _reject_forbidden_text(
        compile_and_link_text, "production target compile/link evidence"
    )
    unexpected_packages = [
        package
        for package in NON_TARGET_WORKSPACE_PACKAGES
        if f"{closure_install_base / package}/" in compile_and_link_text
    ]
    if unexpected_packages:
        raise ValueError(
            "target compile/link evidence consumes unexpected workspace packages: "
            + ", ".join(unexpected_packages)
        )

    for command in (["readelf", "-d", str(target_executable)], ["ldd", str(target_executable)]):
        result = subprocess.run(command, text=True, capture_output=True, check=False)
        if result.returncode != 0:
            raise ValueError(f"target runtime closure command failed: {' '.join(command)}")
        _reject_forbidden_text(result.stdout + result.stderr, "target runtime resolution")
        if command[0] == "ldd":
            for line in result.stdout.splitlines():
                if "libperception_interfaces" in line and expected_interfaces not in line:
                    raise ValueError(
                        "target runtime perception_interfaces resolves outside closure"
                    )

def _capture_workspace_closure(
    install_base: Path, build_base: Path, paired_source_identity_sha256: str
) -> dict[str, Any]:
    install_base = install_base.resolve(strict=True)
    build_base = build_base.resolve(strict=True)
    _reject_forbidden_text(str(install_base), "closure install base")
    _reject_forbidden_text(str(build_base), "closure build base")
    if re.fullmatch(r"[0-9a-f]{64}", paired_source_identity_sha256) is None:
        raise ValueError("paired source identity digest is invalid")

    dependencies = [
        _dependency_record(package, install_base, build_base)
        for package in WORKSPACE_DEPENDENCY_PACKAGES
    ]
    dependencies.sort(key=lambda value: value["package"])
    profiling_prefix = _require_within(
        install_base / "perception_profiling", install_base, "profiling prefix"
    )
    helpers = [
        {
            "name": name,
            **_artifact_record(
                profiling_prefix / relative_path,
                install_base,
                "helper_elf",
                require_build_id=True,
            ),
        }
        for name, relative_path in PROFILING_HELPERS.items()
    ]
    helpers.sort(key=lambda value: value["relative_path"])
    workload = _artifact_record(
        profiling_prefix / PROFILING_WORKLOAD, install_base, "workload"
    )
    dependency_digest = _canonical_sha256(
        {
            "paired_source_identity_sha256": paired_source_identity_sha256,
            "closure_install_base": str(install_base),
            "closure_build_base": str(build_base),
            "dependencies": dependencies,
        }
    )
    helper_digest = _canonical_sha256({"helpers": helpers, "workload": workload})
    return {
        "schema_version": 1,
        "paired_source_identity_sha256": paired_source_identity_sha256,
        "closure_install_base": str(install_base),
        "closure_build_base": str(build_base),
        "dependencies": dependencies,
        "helpers": helpers,
        "workload": workload,
        "dependency_comparison_sha256": dependency_digest,
        "helper_set_sha256": helper_digest,
    }


def capture_workspace_closure(
    install_base: Path,
    build_base: Path,
    source_identity_path: Path,
    *,
    current_source_identity: dict[str, str] | None = None,
    target_build_directory: Path | None = None,
    target_executable: Path | None = None,
) -> dict[str, Any]:
    source_identity = load_source_identity(source_identity_path)
    if current_source_identity is not None and source_identity != current_source_identity:
        raise ValueError("current source identity differs from paired build identity")
    resolved_install = install_base.resolve(strict=True)
    if (target_build_directory is None) != (target_executable is None):
        raise ValueError("target build directory and executable must be provided together")
    if target_build_directory is not None and target_executable is not None:
        _validate_target_closure_resolution(
            target_build_directory, target_executable, resolved_install
        )
    return _capture_workspace_closure(
        resolved_install,
        build_base,
        source_identity["paired_source_identity_sha256"],
    )


def write_workspace_closure(path: Path, payload: dict[str, Any]) -> None:
    path.write_bytes(canonical_json_bytes(payload))


def load_and_validate_workspace_closure(path: Path) -> dict[str, Any]:
    path = path.resolve(strict=True)
    raw = path.read_bytes()
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid closure manifest JSON: {error}") from error
    if not isinstance(payload, dict) or set(payload) != CLOSURE_KEYS:
        raise ValueError("closure manifest has missing or unknown top-level fields")
    if payload.get("schema_version") != 1:
        raise ValueError("closure manifest schema must be 1")
    if raw != canonical_json_bytes(payload):
        raise ValueError("closure manifest is not canonical JSON")
    source_identity = load_source_identity(path.parent / "paired-source-identity.txt")
    if source_identity["paired_source_identity_sha256"] != payload.get(
        "paired_source_identity_sha256"
    ):
        raise ValueError("closure/source identity digest mismatch")
    rebuilt = _capture_workspace_closure(
        Path(str(payload.get("closure_install_base"))),
        Path(str(payload.get("closure_build_base"))),
        source_identity["paired_source_identity_sha256"],
    )
    if rebuilt != payload:
        raise ValueError("closure artifact changed since the manifest was captured")
    return payload


def write_values(path: Path, values: dict[str, str]) -> None:
    path.write_text(
        "".join(f"{key}={value}\n" for key, value in values.items()),
        encoding="utf-8",
    )


def _closure_values(path: Path, payload: dict[str, Any]) -> dict[str, str]:
    return {
        "paired_source_identity_sha256": payload["paired_source_identity_sha256"],
        "workspace_closure_install_base": payload["closure_install_base"],
        "workspace_closure_build_base": payload["closure_build_base"],
        "workspace_closure_manifest_sha256": sha256_file(path),
        "workspace_dependency_comparison_sha256": payload[
            "dependency_comparison_sha256"
        ],
        "profiling_prefix": str(
            Path(payload["closure_install_base"]) / "perception_profiling"
        ),
        "profiling_helper_set_sha256": payload["helper_set_sha256"],
    }


def _legacy_main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("build_directory", type=Path)
    parser.add_argument("build_profile", choices=("performance", "sanitizer"))
    parser.add_argument("expected_stage_option", choices=("ON", "OFF"))
    parser.add_argument("output", type=Path)
    args = parser.parse_args(argv)
    try:
        values = validate_build(
            args.build_directory, args.build_profile, args.expected_stage_option
        )
        write_values(args.output, values)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    return 0


def _command_main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    source_parser = subparsers.add_parser("source-identity")
    source_parser.add_argument("source_revision")
    source_parser.add_argument("source_diff", type=Path)
    source_parser.add_argument("source_untracked", type=Path)
    source_parser.add_argument("source_untracked_archive", type=Path)
    source_parser.add_argument("output", type=Path)

    capture_parser = subparsers.add_parser("capture-closure")
    capture_parser.add_argument("install_base", type=Path)
    capture_parser.add_argument("build_base", type=Path)
    capture_parser.add_argument("source_identity", type=Path)
    capture_parser.add_argument("output", type=Path)
    capture_parser.add_argument("values_output", type=Path)
    capture_parser.add_argument("--source-revision")
    capture_parser.add_argument("--source-diff", type=Path)
    capture_parser.add_argument("--source-untracked", type=Path)
    capture_parser.add_argument("--source-untracked-archive", type=Path)
    capture_parser.add_argument("--target-build-directory", type=Path)
    capture_parser.add_argument("--target-executable", type=Path)

    verify_parser = subparsers.add_parser("verify-closure")
    verify_parser.add_argument("manifest", type=Path)
    verify_parser.add_argument("values_output", type=Path)

    args = parser.parse_args(argv)
    try:
        if args.command == "source-identity":
            values = source_identity_from_files(
                args.source_revision,
                args.source_diff,
                args.source_untracked,
                args.source_untracked_archive,
            )
            write_values(args.output, values)
        elif args.command == "capture-closure":
            current_fields = (
                args.source_revision,
                args.source_diff,
                args.source_untracked,
                args.source_untracked_archive,
            )
            if any(value is not None for value in current_fields) and not all(
                value is not None for value in current_fields
            ):
                raise ValueError("all current source identity inputs are required together")
            current_identity = None
            if all(value is not None for value in current_fields):
                current_identity = source_identity_from_files(*current_fields)
            payload = capture_workspace_closure(
                args.install_base,
                args.build_base,
                args.source_identity,
                current_source_identity=current_identity,
                target_build_directory=args.target_build_directory,
                target_executable=args.target_executable,
            )
            write_workspace_closure(args.output, payload)
            write_values(args.values_output, _closure_values(args.output, payload))
        else:
            payload = load_and_validate_workspace_closure(args.manifest)
            write_values(args.values_output, _closure_values(args.manifest, payload))
    except (OSError, ValueError) as error:
        parser.error(str(error))
    return 0


def main() -> int:
    commands = {"source-identity", "capture-closure", "verify-closure"}
    if len(sys.argv) > 1 and sys.argv[1] in commands:
        return _command_main(sys.argv[1:])
    return _legacy_main(sys.argv[1:])


if __name__ == "__main__":
    raise SystemExit(main())
