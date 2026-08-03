#!/usr/bin/env bash
set -uo pipefail

usage()
{
    echo "usage: $0 <mode> <install-prefix> <new-output-dir> <duration-seconds> [bounded|expanding] [callback|full]" >&2
    echo "modes: plain-sample capacity-ramp perf-stat perf-record ros-trace stage-latency heaptrack asan-smoke lsan-smoke valgrind-memcheck valgrind-massif" >&2
}

if [[ $# -lt 4 || $# -gt 6 ]]; then
    usage
    exit 2
fi

MODE="$1"
INSTALL_PREFIX="$(readlink -f "$2")"
OUTPUT_DIR="$3"
DURATION="$4"
WORKLOAD="${5:-bounded}"
STAGE_EVENT_SET="${6:-full}"
case "${MODE}" in
    plain-sample|capacity-ramp|perf-stat|perf-record|ros-trace|stage-latency|heaptrack|asan-smoke|lsan-smoke|valgrind-memcheck|valgrind-massif)
        ;;
    *)
        usage
        exit 2
        ;;
esac
if [[ "${MODE}" == capacity-ramp ]]; then
    WORKLOAD=expanding
fi
if [[ "${WORKLOAD}" != bounded && "${WORKLOAD}" != expanding ]]; then
    usage
    exit 2
fi
if [[ "${MODE}" == stage-latency ]]; then
    if [[ "${STAGE_EVENT_SET}" != callback && "${STAGE_EVENT_SET}" != full ]]; then
        echo "stage-latency event set must be callback or full" >&2
        exit 2
    fi
elif (( $# == 6 )); then
    echo "stage event set is valid only for stage-latency mode" >&2
    exit 2
else
    STAGE_EVENT_SET=none
fi
if [[ ! "${DURATION}" =~ ^[1-9][0-9]*$ ]]; then
    echo "duration must be a positive integer" >&2
    exit 2
fi
if [[ "${MODE}" == capacity-ramp && "${DURATION}" -gt 600 ]]; then
    echo "capacity-ramp duration must not exceed 600 seconds" >&2
    exit 2
fi
if [[ ! -f "${INSTALL_PREFIX}/setup.bash" ]]; then
    echo "missing install setup: ${INSTALL_PREFIX}/setup.bash" >&2
    exit 2
fi
if [[ ! -f "${INSTALL_PREFIX}/local_setup.bash" ]]; then
    echo "missing install local setup: ${INSTALL_PREFIX}/local_setup.bash" >&2
    exit 2
fi
if [[ -e "${OUTPUT_DIR}" ]]; then
    echo "output directory already exists: ${OUTPUT_DIR}" >&2
    exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROFILE_RUNNER_COMMON="${SCRIPT_DIR}/lib/profile-runner-common.sh"
PROFILE_REPORT_PARSER="${SCRIPT_DIR}/lib/profile_report_parsers.py"
PROFILE_ROLE_MONITOR="${SCRIPT_DIR}/lib/profile_role_monitor.py"
PROFILE_SAMPLER="${SCRIPT_DIR}/lib/profile_local_map_sampler.py"
PROFILE_STAGE_ANALYZER="${SCRIPT_DIR}/lib/stage_latency_analysis.py"
PROFILE_BUILD_PROVENANCE="${SCRIPT_DIR}/lib/local_map_build_provenance.py"
PROFILE_ANALYZER="${SCRIPT_DIR}/analyze-local-map-profile.py"
for required_file in "${PROFILE_RUNNER_COMMON}" "${PROFILE_REPORT_PARSER}" \
    "${PROFILE_ROLE_MONITOR}" "${PROFILE_SAMPLER}" "${PROFILE_STAGE_ANALYZER}" \
    "${PROFILE_BUILD_PROVENANCE}" "${PROFILE_ANALYZER}"; do
    if [[ ! -r "${required_file}" ]]; then
        echo "missing profiling asset: ${required_file}" >&2
        exit 2
    fi
done
source "${PROFILE_RUNNER_COMMON}"

OUTPUT_DIR="$(readlink -m "${OUTPUT_DIR}")"

CLOSURE_PROVENANCE_ENABLED=false
CLOSURE_INSTALL_BASE="${ALIEN_PROFILE_CLOSURE_INSTALL_BASE:-${INSTALL_PREFIX}}"
CLOSURE_BUILD_BASE="${ALIEN_PROFILE_CLOSURE_BUILD_BASE:-}"
PAIRED_SOURCE_IDENTITY="${ALIEN_PROFILE_PAIRED_SOURCE_IDENTITY:-}"
if [[ "${MODE}" == stage-latency ]]; then
    if [[ -z "${ALIEN_PROFILE_CLOSURE_INSTALL_BASE:-}" \
        || -z "${ALIEN_PROFILE_CLOSURE_BUILD_BASE:-}" \
        || -z "${PAIRED_SOURCE_IDENTITY}" ]]; then
        echo "stage-latency requires closure install/build bases and paired source identity" >&2
        exit 2
    fi
    CLOSURE_PROVENANCE_ENABLED=true
elif [[ -n "${PAIRED_SOURCE_IDENTITY}" ]]; then
    CLOSURE_PROVENANCE_ENABLED=true
fi
CLOSURE_INSTALL_BASE="$(readlink -f "${CLOSURE_INSTALL_BASE}" 2>/dev/null || true)"
if [[ -n "${CLOSURE_BUILD_BASE}" ]]; then
    CLOSURE_BUILD_BASE="$(readlink -f "${CLOSURE_BUILD_BASE}" 2>/dev/null || true)"
fi
if [[ "${MODE}" == stage-latency ]]; then
    if [[ -z "${CLOSURE_INSTALL_BASE}" || "${CLOSURE_INSTALL_BASE}" == "${INSTALL_PREFIX}" ]]; then
        echo "stage-latency target and production closure prefixes must be distinct" >&2
        exit 2
    fi
    if [[ ! -f "${CLOSURE_INSTALL_BASE}/setup.bash" ]]; then
        echo "missing production closure setup: ${CLOSURE_INSTALL_BASE}/setup.bash" >&2
        exit 2
    fi
elif [[ "${CLOSURE_INSTALL_BASE}" != "${INSTALL_PREFIX}" ]]; then
    echo "non-stage modes require target and helper packages from one install prefix" >&2
    exit 2
fi
if [[ "${CLOSURE_PROVENANCE_ENABLED}" == true && ! -r "${PAIRED_SOURCE_IDENTITY}" ]]; then
    echo "paired source identity is missing or unreadable: ${PAIRED_SOURCE_IDENTITY}" >&2
    exit 2
fi
mkdir -p "${OUTPUT_DIR}"

VALID=true
NORMAL_COMPLETION=false
FORCED_STOP=false
ROLE_EXIT_FAILURE=false
TARGET_STOPPED=false
FIXTURE_PID=""
FIXTURE_PGID=""
FIXTURE_STARTTIME=""
SINK_PID=""
SINK_PGID=""
SINK_STARTTIME=""
LAUNCHER_PID=""
LAUNCHER_PGID=""
LAUNCHER_STARTTIME=""
LAUNCHER_REAPED=false
TARGET_IDENTITY_RECORDED=false
TOOL_PID=""
TOOL_PGID=""
TOOL_STARTTIME=""
TOOL_REAPED=false
TRACEE_PID=""
TRACEE_PGID=""
TRACEE_STARTTIME=""
SAMPLER_PID=""
SAMPLER_PGID=""
SAMPLER_STARTTIME=""
PIDSTAT_PID=""
PIDSTAT_PGID=""
PIDSTAT_STARTTIME=""
PIDSTAT_START_MONOTONIC_NS=""
PIDSTAT_STOP_MONOTONIC_NS=""
PIDSTAT_INTERVAL_S=1
PIDSTAT_BRACKET_MARGIN_NS=2000000000
STAGE_TRACE_SESSION=""
STAGE_TRACE_CREATED=false
STAGE_TRACE_STARTED=false
STAGE_TRACE_NORMAL_COMPLETION=false
STAGE_TRACE_SUBBUF_SIZE_BYTES=262144
STAGE_TRACE_NUM_SUBBUF=4
STAGE_TRACE_SHM_MARGIN_BYTES=8388608
STAGE_PROVIDER_EVENTS=(
    callback_begin callback_end
    mapper_apply_begin mapper_apply_end
    state_publication_begin state_publication_end
    read_transaction_begin read_transaction_end
    snapshot_serialization_begin snapshot_serialization_end
    snapshot_total_begin snapshot_total_end
)
if [[ "${STAGE_EVENT_SET}" == callback ]]; then
    STAGE_SELECTED_EVENTS=(callback_begin callback_end)
else
    STAGE_SELECTED_EVENTS=("${STAGE_PROVIDER_EVENTS[@]}")
fi
PERF_CTL_FD_OPEN=false
PERF_PATH=""
TRACEE_TIDS=""
TRACE_SESSION=""
TRACE_STARTED=false
MEMCHECK_EXIT_CODE=""
SCRIPT_PGID="$(ps -o pgid= -p "$$" | tr -d ' ')"

invalidate()
{
    local reason="$1"
    VALID=false
    echo "INVALID: ${reason}" | tee -a "${OUTPUT_DIR}/invalid-reasons.txt" >&2
}

validate_profile_environment()
{
    local phase="$1" variable_name value
    for variable_name in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH \
        LD_LIBRARY_PATH PYTHONPATH; do
        value="${!variable_name-}"
        case ":${value}:" in
            *:/workspaces/alien-scanner/ws/install:* \
            | *:/workspaces/alien-scanner/ws/install/*:* )
                invalidate "${phase}: ${variable_name} contains forbidden main-workspace install"
                return 1
                ;;
        esac
    done
    return 0
}

stop_partial_target_startup()
{
    local rc=0
    [[ -n "${LAUNCHER_PID}" ]] || return 0
    if process_is_alive "${LAUNCHER_PID}"; then
        if [[ -z "${LAUNCHER_STARTTIME}" || -z "${LAUNCHER_PGID}" ]] \
            || ! process_identity_matches \
                "${LAUNCHER_PID}" "${LAUNCHER_STARTTIME}" "${LAUNCHER_PGID}"; then
            ROLE_EXIT_FAILURE=true
            invalidate "unable to safely stop unresolved target launcher: identity unavailable"
            return 1
        fi
        signal_process TERM \
            "${LAUNCHER_PID}" "${LAUNCHER_PGID}" "${LAUNCHER_STARTTIME}" || return 1
        if ! wait_for_dead "${LAUNCHER_PID}" 10; then
            FORCED_STOP=true
            invalidate "unresolved target launcher did not exit after SIGTERM"
            signal_process KILL \
                "${LAUNCHER_PID}" "${LAUNCHER_PGID}" "${LAUNCHER_STARTTIME}" || true
            if ! wait_for_dead "${LAUNCHER_PID}" 5; then
                ROLE_EXIT_FAILURE=true
                invalidate "unresolved target launcher could not be reaped"
                return 1
            fi
        fi
    fi
    wait_child "${LAUNCHER_PID}" startup_launcher || rc=$?
    LAUNCHER_REAPED=true
    [[ "${TOOL_PID}" == "${LAUNCHER_PID}" ]] && TOOL_REAPED=true
    TARGET_STOPPED=true
    return 0
}

cleanup()
{
    local original_rc=$?
    trap - EXIT INT TERM
    if [[ "${TRACE_STARTED}" == true && -n "${TRACE_SESSION}" ]]; then
        ros2 trace stop "${TRACE_SESSION}" >> "${OUTPUT_DIR}/trace-stop.log" 2>&1 || true
    fi
    if [[ "${STAGE_TRACE_CREATED}" == true && -n "${STAGE_TRACE_SESSION}" ]]; then
        if [[ "${STAGE_TRACE_STARTED}" == true ]]; then
            lttng stop "${STAGE_TRACE_SESSION}" >> "${OUTPUT_DIR}/stage-trace-stop.log" 2>&1 || true
        fi
        lttng destroy "${STAGE_TRACE_SESSION}" >> "${OUTPUT_DIR}/stage-trace-destroy.log" 2>&1 || true
    fi
    if [[ -n "${SAMPLER_PID}" ]]; then
        stop_group "${SAMPLER_PID}" "${SAMPLER_PGID}" checkpoint_sampler "${SAMPLER_STARTTIME}" TERM || true
    fi
    if [[ -n "${PIDSTAT_PID}" ]]; then
        stop_group "${PIDSTAT_PID}" "${PIDSTAT_PGID}" pidstat "${PIDSTAT_STARTTIME}" TERM || true
    fi
    if [[ "${PERF_CTL_FD_OPEN}" == true ]]; then
        exec 8>&- 9>&-
    fi
    if [[ "${TARGET_STOPPED}" != true && -n "${LAUNCHER_PID}" ]]; then
        if [[ "${TARGET_IDENTITY_RECORDED}" == true ]]; then
            stop_target || true
        else
            stop_partial_target_startup || true
        fi
    fi
    if [[ -n "${SINK_PID}" ]]; then
        stop_group "${SINK_PID}" "${SINK_PGID}" sink "${SINK_STARTTIME}" || true
    fi
    if [[ -n "${FIXTURE_PID}" ]]; then
        stop_group "${FIXTURE_PID}" "${FIXTURE_PGID}" fixture "${FIXTURE_STARTTIME}" || true
    fi
    if [[ -d "${OUTPUT_DIR}" ]]; then
        {
            [[ -f "${OUTPUT_DIR}/exit-codes.txt" ]] && sed 's/^/role_/' "${OUTPUT_DIR}/exit-codes.txt"
            echo "valid=${VALID}"
            echo "normal_completion=${NORMAL_COMPLETION}"
            echo "forced_stop=${FORCED_STOP}"
            echo "role_exit_failure=${ROLE_EXIT_FAILURE}"
            echo "script_exit_code=${original_rc}"
            echo "finished_at=$(date --iso-8601=ns)"
        } >> "${OUTPUT_DIR}/run-manifest.txt"
        find "${OUTPUT_DIR}" -type f ! -name sha256sum.txt -print0 \
            | sort -z | xargs -0 -r sha256sum > "${OUTPUT_DIR}/sha256sum.txt"
    fi
    if [[ "${VALID}" != true || "${NORMAL_COMPLETION}" != true ]]; then
        exit 1
    fi
    exit "${original_rc}"
}
trap cleanup EXIT INT TERM

set +u
SETUP_OK=true
validate_profile_environment "before ROS setup" || SETUP_OK=false
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" || SETUP_OK=false
validate_profile_environment "after ROS setup" || SETUP_OK=false
if [[ "${MODE}" == stage-latency ]]; then
    source "${CLOSURE_INSTALL_BASE}/setup.bash" || SETUP_OK=false
    validate_profile_environment "after production closure setup" || SETUP_OK=false
    source "${INSTALL_PREFIX}/local_setup.bash" || SETUP_OK=false
else
    source "${INSTALL_PREFIX}/setup.bash" || SETUP_OK=false
fi
validate_profile_environment "after profiling setup" || SETUP_OK=false
set -u
if [[ "${SETUP_OK}" != true ]]; then
    invalidate "ROS or profiling install setup failed"
    exit 1
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export RCUTILS_COLORIZED_OUTPUT=0
export LC_ALL=C
if [[ -n "${ALIEN_PROFILE_ROS_DOMAIN_ID:-}" ]]; then
    export ROS_DOMAIN_ID="${ALIEN_PROFILE_ROS_DOMAIN_ID}"
else
    for candidate in $(seq 100 229 | shuf); do
        if eval "exec 7>/tmp/alien-local-map-profile-domain-${candidate}.lock" \
            && flock -n 7; then
            export ROS_DOMAIN_ID="${candidate}"
            break
        fi
    done
fi
if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
    invalidate "could not reserve a ROS domain ID"
    exit 1
fi

PROFILE_IMAGE_ID="${ALIEN_PROFILE_IMAGE_ID:-}"
if [[ ! "${PROFILE_IMAGE_ID}" =~ ^sha256:[0-9a-f]{64}$ ]]; then
    invalidate "ALIEN_PROFILE_IMAGE_ID must be the host-reported sha256 image ID"
    exit 1
fi
if (( $(nproc) < 2 )) || ! taskset -c 0 true || ! taskset -c 1 true; then
    invalidate "profiling requires available CPU 0 and CPU 1"
    exit 1
fi

LOCAL_MAP_PREFIX="$(ros2 pkg prefix perception_local_map 2>/dev/null || true)"
PROFILING_PREFIX="$(ros2 pkg prefix perception_profiling 2>/dev/null || true)"
CORE_PREFIX="$(ros2 pkg prefix perception_core 2>/dev/null || true)"
INTERFACES_PREFIX="$(ros2 pkg prefix perception_interfaces 2>/dev/null || true)"
LOCAL_MAP_PREFIX="$(readlink -f "${LOCAL_MAP_PREFIX}" 2>/dev/null || true)"
PROFILING_PREFIX="$(readlink -f "${PROFILING_PREFIX}" 2>/dev/null || true)"
CORE_PREFIX="$(readlink -f "${CORE_PREFIX}" 2>/dev/null || true)"
INTERFACES_PREFIX="$(readlink -f "${INTERFACES_PREFIX}" 2>/dev/null || true)"
if [[ -z "${LOCAL_MAP_PREFIX}" || ( "${LOCAL_MAP_PREFIX}" != "${INSTALL_PREFIX}" \
    && "${LOCAL_MAP_PREFIX}" != "${INSTALL_PREFIX}/"* ) ]]; then
    invalidate "perception_local_map does not resolve from the target install prefix"
    exit 1
fi
for resolved_prefix in "${PROFILING_PREFIX}" "${CORE_PREFIX}" "${INTERFACES_PREFIX}"; do
    if [[ -z "${resolved_prefix}" || ( "${resolved_prefix}" != "${CLOSURE_INSTALL_BASE}" \
        && "${resolved_prefix}" != "${CLOSURE_INSTALL_BASE}/"* ) ]]; then
        invalidate "profiling helper or dependency package does not resolve from production closure"
        exit 1
    fi
done

EXPECTED_TARGET_EXE="${LOCAL_MAP_PREFIX}/lib/perception_local_map/perception_local_map_node"
FIXTURE_EXE="${PROFILING_PREFIX}/lib/perception_profiling/perception_profile_fixture"
SINK_EXE="${PROFILING_PREFIX}/lib/perception_profiling/perception_profile_sink"
ORACLE_EXE="${PROFILING_PREFIX}/lib/perception_profiling/perception_profile_oracle"
WORKLOAD_YAML="${PROFILING_PREFIX}/share/perception_profiling/config/profile_local_map.yaml"
for executable in "${EXPECTED_TARGET_EXE}" "${FIXTURE_EXE}" "${SINK_EXE}" "${ORACLE_EXE}"; do
    if [[ ! -x "${executable}" ]]; then
        invalidate "installed profiling executable is missing: ${executable}"
        exit 1
    fi
done
if [[ ! -r "${WORKLOAD_YAML}" ]]; then
    invalidate "installed C2 workload config is missing"
    exit 1
fi

TARGET_SHA256="$(sha256sum "${EXPECTED_TARGET_EXE}" | awk '{print $1}')"
TARGET_BUILD_ID="$(readelf -n "${EXPECTED_TARGET_EXE}" 2>/dev/null | awk '/Build ID/ {print $3; exit}')"
if [[ ! "${TARGET_SHA256}" =~ ^[0-9a-f]{64}$ || -z "${TARGET_BUILD_ID}" ]]; then
    invalidate "target SHA-256 or build ID is unavailable"
    exit 1
fi
PROFILE_BUILD_BASE="${ALIEN_PROFILE_BUILD_BASE:-$(dirname "${INSTALL_PREFIX}")/build}"
PROFILE_BUILD_BASE="$(readlink -f "${PROFILE_BUILD_BASE}" 2>/dev/null || true)"
if [[ -z "${CLOSURE_BUILD_BASE}" ]]; then
    CLOSURE_BUILD_BASE="${PROFILE_BUILD_BASE}"
fi
TARGET_BUILD_DIR="${PROFILE_BUILD_BASE}/perception_local_map"
TARGET_BUILD_EXE="${TARGET_BUILD_DIR}/perception_local_map_node"
if [[ -z "${PROFILE_BUILD_BASE}" || ! -x "${TARGET_BUILD_EXE}" ]]; then
    invalidate "target build tree is unavailable for the requested install prefix"
    exit 1
fi
TARGET_BUILD_ARTIFACT_BUILD_ID="$(
    readelf -n "${TARGET_BUILD_EXE}" 2>/dev/null | awk '/Build ID/ {print $3; exit}'
)"
if [[ -z "${TARGET_BUILD_ARTIFACT_BUILD_ID}" \
    || "${TARGET_BUILD_ARTIFACT_BUILD_ID}" != "${TARGET_BUILD_ID}" ]]; then
    invalidate "installed target does not match the corresponding build-tree ELF"
    exit 1
fi
TARGET_BUILD_PROFILE=performance
[[ "${MODE}" == asan-smoke || "${MODE}" == lsan-smoke ]] \
    && TARGET_BUILD_PROFILE=sanitizer
TARGET_EXPECTED_STAGE_OPTION=OFF
[[ "${MODE}" == stage-latency ]] && TARGET_EXPECTED_STAGE_OPTION=ON
python3 "${PROFILE_BUILD_PROVENANCE}" "${TARGET_BUILD_DIR}" \
    "${TARGET_BUILD_PROFILE}" "${TARGET_EXPECTED_STAGE_OPTION}" \
    "${OUTPUT_DIR}/target-build-provenance.txt" \
    || { invalidate "target build provenance validation failed"; exit 1; }
cp "${TARGET_BUILD_DIR}/CMakeCache.txt" "${OUTPUT_DIR}/target-cmake-cache.txt"
cp "${TARGET_BUILD_DIR}/compile_commands.json" \
    "${OUTPUT_DIR}/target-compile-commands.json"
TARGET_BUILD_ARTIFACT_SHA256="$(sha256sum "${TARGET_BUILD_EXE}" | awk '{print $1}')"
PERF_PATH="$(find_perf || true)"

git -C "${REPO_ROOT}" status --short > "${OUTPUT_DIR}/git-status.txt" 2>&1 || true
SOURCE_INPUT_PATHS=(.devcontainer scripts ws/src CMakeLists.txt)
git -C "${REPO_ROOT}" diff --binary HEAD -- "${SOURCE_INPUT_PATHS[@]}" \
    > "${OUTPUT_DIR}/source-diff.patch" 2>&1 || true
git -C "${REPO_ROOT}" ls-files --others --exclude-standard -- "${SOURCE_INPUT_PATHS[@]}" \
    > "${OUTPUT_DIR}/source-untracked.txt" 2>&1 || true
create_deterministic_source_archive "${REPO_ROOT}" \
    "${OUTPUT_DIR}/source-untracked.txt" "${OUTPUT_DIR}/source-untracked.tar.gz" \
    || { invalidate "deterministic source archive creation failed"; exit 1; }
ldd "${EXPECTED_TARGET_EXE}" > "${OUTPUT_DIR}/target-ldd.txt" 2>&1 || true
readelf -n "${EXPECTED_TARGET_EXE}" > "${OUTPUT_DIR}/target-notes.txt" 2>&1 || true
readelf -d "${EXPECTED_TARGET_EXE}" > "${OUTPUT_DIR}/target-dynamic.txt" 2>&1 || true
nm -C "${EXPECTED_TARGET_EXE}" > "${OUTPUT_DIR}/target-symbols.txt" 2>&1 || true
SOURCE_REVISION="$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || echo unavailable)"
if [[ "${CLOSURE_PROVENANCE_ENABLED}" == true ]]; then
    cp "${PAIRED_SOURCE_IDENTITY}" "${OUTPUT_DIR}/paired-source-identity.txt"
    python3 "${PROFILE_BUILD_PROVENANCE}" capture-closure \
        "${CLOSURE_INSTALL_BASE}" "${CLOSURE_BUILD_BASE}" \
        "${OUTPUT_DIR}/paired-source-identity.txt" \
        "${OUTPUT_DIR}/target-workspace-closure.json" \
        "${OUTPUT_DIR}/target-workspace-closure-values.txt" \
        --source-revision "${SOURCE_REVISION}" \
        --source-diff "${OUTPUT_DIR}/source-diff.patch" \
        --source-untracked "${OUTPUT_DIR}/source-untracked.txt" \
        --source-untracked-archive "${OUTPUT_DIR}/source-untracked.tar.gz" \
        --target-build-directory "${TARGET_BUILD_DIR}" \
        --target-executable "${EXPECTED_TARGET_EXE}" \
        || { invalidate "workspace closure provenance validation failed"; exit 1; }
fi
if [[ "${MODE}" == stage-latency ]] \
    && ! grep -q 'libperception_local_map_stage_latency\.so' "${OUTPUT_DIR}/target-ldd.txt"; then
    invalidate "stage-latency requires a target from the dedicated UST latency prefix"
    exit 1
fi
if [[ "${MODE}" == asan-smoke || "${MODE}" == lsan-smoke ]]; then
    if ! grep -Eq 'Shared library: \[libasan\.so' "${OUTPUT_DIR}/target-dynamic.txt"; then
        invalidate "sanitizer mode requires a target linked with libasan"
        exit 1
    fi
fi

{
    echo "mode=${MODE}"
    echo "workload=${WORKLOAD}"
    echo "duration_requested_s=${DURATION}"
    [[ "${MODE}" == stage-latency ]] && echo "stage_event_set=${STAGE_EVENT_SET}"
    echo "started_at=$(date --iso-8601=ns)"
    echo "source_revision=${SOURCE_REVISION}"
    echo "source_dirty_count=$(wc -l < "${OUTPUT_DIR}/git-status.txt")"
    echo "source_diff_sha256=$(sha256sum "${OUTPUT_DIR}/source-diff.patch" | awk '{print $1}')"
    echo "source_untracked_sha256=$(sha256sum "${OUTPUT_DIR}/source-untracked.txt" | awk '{print $1}')"
    echo "source_untracked_archive_sha256=$(sha256sum "${OUTPUT_DIR}/source-untracked.tar.gz" | awk '{print $1}')"
    echo "profile_script_sha256=$(sha256sum "${BASH_SOURCE[0]}" | awk '{print $1}')"
    echo "profile_runner_common_sha256=$(sha256sum "${PROFILE_RUNNER_COMMON}" | awk '{print $1}')"
    echo "profile_report_parsers_sha256=$(sha256sum "${PROFILE_REPORT_PARSER}" | awk '{print $1}')"
    echo "profile_role_monitor_sha256=$(sha256sum "${PROFILE_ROLE_MONITOR}" | awk '{print $1}')"
    echo "profile_sampler_sha256=$(sha256sum "${PROFILE_SAMPLER}" | awk '{print $1}')"
    echo "stage_latency_analysis_sha256=$(sha256sum "${PROFILE_STAGE_ANALYZER}" | awk '{print $1}')"
    echo "profile_build_provenance_sha256=$(sha256sum "${PROFILE_BUILD_PROVENANCE}" | awk '{print $1}')"
    echo "analysis_script_sha256=$(sha256sum "${PROFILE_ANALYZER}" | awk '{print $1}')"
    echo "workload_yaml_sha256=$(sha256sum "${WORKLOAD_YAML}" | awk '{print $1}')"
    echo "container_id=$(hostname)"
    echo "image_id=${PROFILE_IMAGE_ID}"
    echo "kernel=$(uname -srvm)"
    echo "rmw=${RMW_IMPLEMENTATION}"
    echo "ros_domain_id=${ROS_DOMAIN_ID}"
    echo "cpu_target=0"
    echo "cpu_helpers=1"
    echo "cpu_allowed=$(awk '/Cpus_allowed_list/ {print $2}' /proc/self/status)"
    echo "cpu_max=$(cat /sys/fs/cgroup/cpu.max 2>/dev/null || echo unavailable)"
    echo "memory_max=$(cat /sys/fs/cgroup/memory.max 2>/dev/null || echo unavailable)"
    echo "perf_event_paranoid=$(cat /proc/sys/kernel/perf_event_paranoid 2>/dev/null || echo unavailable)"
    echo "install_prefix=${INSTALL_PREFIX}"
    echo "target_exe=${EXPECTED_TARGET_EXE}"
    echo "target_sha256=${TARGET_SHA256}"
    echo "target_build_id=${TARGET_BUILD_ID}"
    echo "target_build_artifact=${TARGET_BUILD_EXE}"
    echo "target_build_artifact_sha256=${TARGET_BUILD_ARTIFACT_SHA256}"
    echo "target_build_artifact_build_id=${TARGET_BUILD_ARTIFACT_BUILD_ID}"
    cat "${OUTPUT_DIR}/target-build-provenance.txt"
    if [[ "${CLOSURE_PROVENANCE_ENABLED}" == true ]]; then
        cat "${OUTPUT_DIR}/target-workspace-closure-values.txt"
    fi
    awk '/^(CapInh|CapPrm|CapEff|CapBnd|CapAmb|NoNewPrivs|Seccomp|Seccomp_filters):/ {
        key = tolower($1); sub(/:$/, "", key); print "container_" key "=" $2
    }' /proc/self/status
} > "${OUTPUT_DIR}/run-manifest.txt"
{
    echo "perf=$(${PERF_PATH:-false} --version 2>&1 | head -1)"
    echo "heaptrack=$(heaptrack --version 2>&1 | head -1)"
    echo "heaptrack_print=$(heaptrack_print --version 2>&1 | head -1)"
    echo "valgrind=$(valgrind --version 2>&1 | head -1)"
    echo "pidstat=$(pidstat -V 2>&1 | head -1)"
    echo "lttng=$(lttng --version 2>&1 | head -1)"
    echo "babeltrace2=$(babeltrace2 --version 2>&1 | head -1)"
} > "${OUTPUT_DIR}/tool-versions.txt"

if [[ -n "${ALIEN_PROFILE_ORACLE_COUNT:-}" ]]; then
    ORACLE_COUNT="${ALIEN_PROFILE_ORACLE_COUNT}"
elif [[ "${MODE}" == capacity-ramp ]]; then
    ORACLE_COUNT=6000
elif [[ "${WORKLOAD}" == bounded ]]; then
    ORACLE_COUNT=$(( (DURATION + 180) * 10 ))
else
    ORACLE_COUNT=$(( (DURATION + 30) * 10 ))
fi
if [[ ! "${ORACLE_COUNT}" =~ ^[1-9][0-9]*$ ]]; then
    invalidate "oracle sequence count is invalid"
    exit 1
fi
if ! taskset -c 1 "${ORACLE_EXE}" "${WORKLOAD}" "${ORACLE_COUNT}" "${OUTPUT_DIR}" \
    > "${OUTPUT_DIR}/oracle.log" 2>&1; then
    invalidate "deterministic oracle failed"
    exit 1
fi
ORACLE_STATUS="$(PYTHONPATH="${SCRIPT_DIR}" python3 - "${OUTPUT_DIR}/oracle_manifest.yaml" <<'PY'
import sys
from pathlib import Path
from lib.local_map_profile_analysis import parse_flat_yaml
value = parse_flat_yaml(Path(sys.argv[1]))
print(value.get("capacity.status", ""))
PY
)"
PLATEAU_START="$(PYTHONPATH="${SCRIPT_DIR}" python3 - "${OUTPUT_DIR}/oracle_manifest.yaml" <<'PY'
import sys
from pathlib import Path
from lib.local_map_profile_analysis import parse_flat_yaml
value = parse_flat_yaml(Path(sys.argv[1])).get("plateau_start_revision")
print("" if value is None else value)
PY
)"
REQUIRED_END_REVISION="$(PYTHONPATH="${SCRIPT_DIR}" python3 - "${OUTPUT_DIR}/oracle_manifest.yaml" <<'PY'
import sys
from pathlib import Path
from lib.local_map_profile_analysis import parse_flat_yaml
value = parse_flat_yaml(Path(sys.argv[1])).get("capacity.required_end_revision")
print("" if value is None else value)
PY
)"
{
    echo "oracle_sequence_count=${ORACLE_COUNT}"
    echo "capacity_status=${ORACLE_STATUS}"
    echo "capacity_required_end_revision=${REQUIRED_END_REVISION:-none}"
} >> "${OUTPUT_DIR}/run-manifest.txt"
if [[ "${WORKLOAD}" == bounded ]]; then
    if [[ ! "${PLATEAU_START}" =~ ^[0-9]+$ ]]; then
        invalidate "bounded oracle did not prove a 600-revision plateau"
        exit 1
    fi
    WARMUP_REVISION=$((PLATEAU_START + 600))
else
    WARMUP_REVISION=100
fi

record_role()
{
    local role="$1" pid="$2" pgid="$3" starttime="$4"
    {
        echo "${role}_pid=${pid}"
        echo "${role}_pgid=${pgid}"
        echo "${role}_starttime=${starttime}"
        echo "${role}_affinity=$(taskset -pc "${pid}" 2>&1 | tail -1)"
    } >> "${OUTPUT_DIR}/run-manifest.txt"
}

find_matching_descendant()
{
    local root_pid="$1" deadline=$((SECONDS + 20))
    while (( SECONDS < deadline )); do
        process_identity_matches \
            "${root_pid}" "${LAUNCHER_STARTTIME}" "${LAUNCHER_PGID}" || return 3
        local queue=("${root_pid}") matches=() current child exe
        while (( ${#queue[@]} > 0 )); do
            current="${queue[0]}"
            queue=("${queue[@]:1}")
            exe="$(readlink -f "/proc/${current}/exe" 2>/dev/null || true)"
            [[ "${exe}" == "$(readlink -f "${EXPECTED_TARGET_EXE}")" ]] && matches+=("${current}")
            while read -r child; do
                [[ -n "${child}" ]] && queue+=("${child}")
            done < <(pgrep -P "${current}" 2>/dev/null || true)
        done
        if (( ${#matches[@]} == 1 )); then
            echo "${matches[0]}"
            return 0
        fi
        (( ${#matches[@]} > 1 )) && return 2
        sleep 0.1
    done
    return 1
}

start_target()
{
    local target_args=("${EXPECTED_TARGET_EXE}" --ros-args --params-file "${WORKLOAD_YAML}" -r __node:=profile_local_map)
    local heaptrack_process_model=""
    case "${MODE}" in
        heaptrack)
            setsid taskset -c 0 heaptrack -o "${OUTPUT_DIR}/heaptrack.%p" \
                "${target_args[@]}" > "${OUTPUT_DIR}/target.log" 2>&1 &
            LAUNCHER_PID=$!
            TOOL_PID="${LAUNCHER_PID}"
            ;;
        valgrind-memcheck)
            setsid taskset -c 0 valgrind --tool=memcheck --leak-check=full \
                --show-leak-kinds=all --errors-for-leak-kinds=definite,indirect \
                --track-origins=yes --error-exitcode=42 \
                --log-file="${OUTPUT_DIR}/memcheck.%p.log" \
                "${target_args[@]}" > "${OUTPUT_DIR}/target.log" 2>&1 &
            LAUNCHER_PID=$!
            TOOL_PID="${LAUNCHER_PID}"
            TRACEE_PID="${LAUNCHER_PID}"
            ;;
        valgrind-massif)
            setsid taskset -c 0 valgrind --tool=massif --time-unit=ms --stacks=yes \
                --detailed-freq=1 --max-snapshots=200 \
                --massif-out-file="${OUTPUT_DIR}/massif.%p.out" \
                --log-file="${OUTPUT_DIR}/massif.%p.log" \
                "${target_args[@]}" > "${OUTPUT_DIR}/target.log" 2>&1 &
            LAUNCHER_PID=$!
            TOOL_PID="${LAUNCHER_PID}"
            TRACEE_PID="${LAUNCHER_PID}"
            ;;
        asan-smoke)
            setsid env ASAN_OPTIONS="detect_leaks=0:halt_on_error=1:exitcode=22:log_path=${OUTPUT_DIR}/asan" \
                taskset -c 0 "${target_args[@]}" > "${OUTPUT_DIR}/target.log" 2>&1 &
            LAUNCHER_PID=$!
            TRACEE_PID="${LAUNCHER_PID}"
            ;;
        lsan-smoke)
            setsid env ASAN_OPTIONS="detect_leaks=1:halt_on_error=1:exitcode=23:log_path=${OUTPUT_DIR}/lsan" \
                LSAN_OPTIONS="exitcode=23:log_threads=1" \
                taskset -c 0 "${target_args[@]}" > "${OUTPUT_DIR}/target.log" 2>&1 &
            LAUNCHER_PID=$!
            TRACEE_PID="${LAUNCHER_PID}"
            ;;
        *)
            setsid taskset -c 0 "${target_args[@]}" > "${OUTPUT_DIR}/target.log" 2>&1 &
            LAUNCHER_PID=$!
            TRACEE_PID="${LAUNCHER_PID}"
            ;;
    esac
    LAUNCHER_STARTTIME="$(process_starttime "${LAUNCHER_PID}" 2>/dev/null || true)"
    if [[ -z "${LAUNCHER_STARTTIME}" ]] || ! LAUNCHER_PGID="$(wait_for_isolated_pgid "${LAUNCHER_PID}")"; then
        invalidate "target launcher identity did not stabilize"
        return 1
    fi
    if [[ "${MODE}" == heaptrack ]]; then
        TRACEE_PID="$(find_matching_descendant "${LAUNCHER_PID}" || true)"
        if [[ -n "${TRACEE_PID}" && "${TRACEE_PID}" == "${LAUNCHER_PID}" ]]; then
            heaptrack_process_model=exec_in_place
        elif [[ -n "${TRACEE_PID}" ]]; then
            local heaptrack_path
            heaptrack_path="$(command -v heaptrack)"
            if ! process_identity_matches \
                    "${LAUNCHER_PID}" "${LAUNCHER_STARTTIME}" "${LAUNCHER_PGID}" \
                || ! process_cmdline_has_exact_arguments \
                    "${LAUNCHER_PID}" "${heaptrack_path}" "${EXPECTED_TARGET_EXE}"; then
                invalidate "Heaptrack wrapper identity or exact target argv did not stabilize"
                return 1
            fi
            heaptrack_process_model=wrapper_with_unique_descendant
        fi
    fi
    if [[ -z "${TRACEE_PID}" ]]; then
        invalidate "unable to resolve a unique target PID"
        return 1
    fi
    local actual_exe cmdline target_affinity deadline=$((SECONDS + 10))
    while (( SECONDS < deadline )); do
        actual_exe="$(readlink -f "/proc/${TRACEE_PID}/exe" 2>/dev/null || true)"
        cmdline="$(tr '\0' ' ' < "/proc/${TRACEE_PID}/cmdline" 2>/dev/null || true)"
        if [[ "${MODE}" == valgrind-* && "${cmdline}" == *"${EXPECTED_TARGET_EXE}"* ]] \
            || [[ "${actual_exe}" == "$(readlink -f "${EXPECTED_TARGET_EXE}")" ]]; then
            break
        fi
        sleep 0.1
    done
    if [[ "${MODE}" != valgrind-* && "${actual_exe}" != "$(readlink -f "${EXPECTED_TARGET_EXE}")" ]] \
        || [[ "${MODE}" == valgrind-* && "${cmdline}" != *"${EXPECTED_TARGET_EXE}"* ]]; then
        invalidate "target process identity did not stabilize on the requested ELF"
        return 1
    fi
    TRACEE_PGID="$(wait_for_isolated_pgid "${TRACEE_PID}")" || return 1
    TRACEE_STARTTIME="$(process_starttime "${TRACEE_PID}")"
    target_affinity="$(process_affinity_list "${TRACEE_PID}" 2>/dev/null || true)"
    if [[ "${target_affinity}" != 0 ]]; then
        invalidate "target affinity is not the required normalized CPU list 0"
        return 1
    fi
    echo "target_affinity=${target_affinity}" >> "${OUTPUT_DIR}/run-manifest.txt"
    if [[ "${MODE}" == heaptrack && "${TRACEE_PGID}" != "${LAUNCHER_PGID}" ]]; then
        invalidate "Heaptrack target escaped the verified instrumentation process group"
        return 1
    fi
    if [[ -n "${TOOL_PID}" ]]; then
        TOOL_PGID="${LAUNCHER_PGID}"
        TOOL_STARTTIME="${LAUNCHER_STARTTIME}"
    fi
    record_role launcher "${LAUNCHER_PID}" "${LAUNCHER_PGID}" "${LAUNCHER_STARTTIME}"
    record_role tracee "${TRACEE_PID}" "${TRACEE_PGID}" "${TRACEE_STARTTIME}"
    [[ -n "${TOOL_PID}" ]] && record_role tool "${TOOL_PID}" "${TOOL_PGID}" "${TOOL_STARTTIME}"
    if [[ "${MODE}" == heaptrack ]]; then
        tr '\0' ' ' < "/proc/${LAUNCHER_PID}/cmdline" > "${OUTPUT_DIR}/launcher-cmdline.txt"
        {
            echo "heaptrack_process_model=${heaptrack_process_model}"
            echo "launcher_exe=$(readlink -f "/proc/${LAUNCHER_PID}/exe")"
        } >> "${OUTPUT_DIR}/run-manifest.txt"
    fi
    tr '\0' ' ' < "/proc/${TRACEE_PID}/cmdline" > "${OUTPUT_DIR}/tracee-cmdline.txt"
    TARGET_IDENTITY_RECORDED=true
}

start_sink()
{
    setsid taskset -c 1 "${SINK_EXE}" --ros-args \
        -p mode:="${WORKLOAD}" -p output_directory:="${OUTPUT_DIR}" \
        > "${OUTPUT_DIR}/sink.log" 2>&1 &
    SINK_PID=$!
    SINK_PGID="$(wait_for_isolated_pgid "${SINK_PID}")" || return 1
    wait_for_process_identity "${SINK_PID}" "${SINK_EXE}" || return 1
    SINK_STARTTIME="$(process_starttime "${SINK_PID}")"
    record_role sink "${SINK_PID}" "${SINK_PGID}" "${SINK_STARTTIME}"
}

start_fixture()
{
    setsid taskset -c 1 "${FIXTURE_EXE}" --ros-args \
        -p mode:="${WORKLOAD}" -p sequence_limit:=0 -p startup_delay_s:=2.0 \
        > "${OUTPUT_DIR}/fixture.log" 2>&1 &
    FIXTURE_PID=$!
    FIXTURE_PGID="$(wait_for_isolated_pgid "${FIXTURE_PID}")" || return 1
    wait_for_process_identity "${FIXTURE_PID}" "${FIXTURE_EXE}" || return 1
    FIXTURE_STARTTIME="$(process_starttime "${FIXTURE_PID}")"
    record_role fixture "${FIXTURE_PID}" "${FIXTURE_PGID}" "${FIXTURE_STARTTIME}"
}

wait_for_graph()
{
    local deadline=$((SECONDS + 30)) nodes
    while (( SECONDS < deadline )); do
        nodes="$(ros2 node list 2>/dev/null || true)"
        if grep -Fxq /profile_local_map <<< "${nodes}" \
            && grep -Fxq /perception_profile_fixture <<< "${nodes}" \
            && grep -Fxq /perception_profile_sink <<< "${nodes}"; then
            return 0
        fi
        process_is_alive "${TRACEE_PID}" || return 1
        process_is_alive "${FIXTURE_PID}" || return 1
        process_is_alive "${SINK_PID}" || return 1
        sleep 0.25
    done
    return 1
}

capture_graph()
{
    ros2 node list > "${OUTPUT_DIR}/node-list.txt" 2>&1 || return 1
    ros2 topic list -t > "${OUTPUT_DIR}/topic-list.txt" 2>&1 || return 1
    if grep -Eiq 'scan_accumulator|octomap_builder' "${OUTPUT_DIR}/node-list.txt" \
        || grep -Eq '^/scan_returns([[:space:]]|$)|cloud_map' "${OUTPUT_DIR}/topic-list.txt"; then
        return 2
    fi
    local topic safe
    for topic in /profile/perception/observations /profile/perception/pose \
        /profile/perception/health /profile/local_map/state /profile/local_map/octomap /tf_static; do
        safe="$(tr '/' '_' <<< "${topic}" | sed 's/^_//')"
        ros2 topic info "${topic}" --verbose > "${OUTPUT_DIR}/graph-${safe}.txt" 2>&1 || return 1
    done
    grep -Eq '^Publisher count: 1$' "${OUTPUT_DIR}/graph-profile_perception_observations.txt" || return 1
    grep -Eq '^Subscription count: 2$' "${OUTPUT_DIR}/graph-profile_perception_observations.txt" || return 1
    grep -Eq '^Publisher count: 1$' "${OUTPUT_DIR}/graph-profile_perception_pose.txt" || return 1
    grep -Eq '^Subscription count: 1$' "${OUTPUT_DIR}/graph-profile_perception_pose.txt" || return 1
    grep -Eq '^Publisher count: 1$' "${OUTPUT_DIR}/graph-profile_perception_health.txt" || return 1
    grep -Eq '^Subscription count: 2$' "${OUTPUT_DIR}/graph-profile_perception_health.txt" || return 1
    for topic in profile_local_map_state profile_local_map_octomap; do
        grep -Eq '^Publisher count: 1$' "${OUTPUT_DIR}/graph-${topic}.txt" || return 1
        grep -Eq '^Subscription count: 1$' "${OUTPUT_DIR}/graph-${topic}.txt" || return 1
    done
    grep -q 'Node name: perception_profile_fixture' "${OUTPUT_DIR}/graph-tf_static.txt" || return 1
}

wait_for_graph_snapshot()
{
    local deadline=$((SECONDS + 30)) rc
    while (( SECONDS < deadline )); do
        if capture_graph; then
            return 0
        else
            rc=$?
        fi
        (( rc == 2 )) && return 2
        process_is_alive "${TRACEE_PID}" || return 1
        process_is_alive "${FIXTURE_PID}" || return 1
        process_is_alive "${SINK_PID}" || return 1
        sleep 0.25
    done
    return 1
}

capture_parameters_and_tf()
{
    ros2 param dump /profile_local_map > "${OUTPUT_DIR}/target-params.yaml" 2>&1 || return 1
    ros2 param dump /perception_profile_fixture > "${OUTPUT_DIR}/fixture-params.yaml" 2>&1 || return 1
    python3 - "${OUTPUT_DIR}/target-params.yaml" "${OUTPUT_DIR}/fixture-params.yaml" \
        "${WORKLOAD}" <<'PY' || return 1
import math
import sys
import yaml

target_path, fixture_path, workload = sys.argv[1:]

def parameters(path, node):
    with open(path, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    return document[node]["ros__parameters"]

target = parameters(target_path, "/profile_local_map")
fixture = parameters(fixture_path, "/perception_profile_fixture")
expected = {
    "backend_type": "octomap",
    "source_local_map_frame": "map",
    "body_frame": "base_link",
    "resolution_m": 0.2,
    "requires_pose": True,
    "minimum_lidar_ray_evidence": "full_ray",
}
for key, value in expected.items():
    actual = target.get(key)
    if isinstance(value, float):
        matches = isinstance(actual, float) and math.isclose(actual, value, abs_tol=1e-12)
    else:
        matches = actual == value
    if not matches:
        raise SystemExit(f"target parameter {key}: expected {value!r}, got {actual!r}")
sensor = target.get("sensor", {}).get("profile_scan", {})
if sensor.get("ray_evidence") != "full_ray" or sensor.get("frame_id") != "profile_scan_link":
    raise SystemExit("profile_scan descriptor is not the frozen FullRay sensor")
if fixture.get("mode") != workload or fixture.get("sequence_limit") != 0:
    raise SystemExit("fixture mode or sequence limit differs from the requested workload")
PY
    timeout 10 ros2 topic echo /tf_static tf2_msgs/msg/TFMessage --once \
        --qos-reliability reliable --qos-durability transient_local \
        > "${OUTPUT_DIR}/tf-static.yaml" 2>&1 || return 1
    grep -Fq 'frame_id: base_link' "${OUTPUT_DIR}/tf-static.yaml" || return 1
    grep -Fq 'child_frame_id: profile_scan_link' "${OUTPUT_DIR}/tf-static.yaml" || return 1
    [[ "$(grep -Ec '^ *[xyzw]: 0\.5$' "${OUTPUT_DIR}/tf-static.yaml")" -ge 4 ]] || return 1
}

latest_revision()
{
    awk -F, 'NR > 1 && $4 ~ /^[0-9]+$/ {if ($4 > maximum) maximum=$4} END {print maximum+0}' \
        "${OUTPUT_DIR}/states.csv" 2>/dev/null || echo 0
}

wait_for_warmup()
{
    local deadline=$((SECONDS + WARMUP_REVISION / 10 + 90)) revision=0
    while (( SECONDS < deadline )); do
        revision="$(latest_revision)"
        if (( revision >= WARMUP_REVISION )); then
            echo "warmup_end_revision=${revision}" >> "${OUTPUT_DIR}/run-manifest.txt"
            return 0
        fi
        process_is_alive "${TRACEE_PID}" || return 1
        process_is_alive "${FIXTURE_PID}" || return 1
        process_is_alive "${SINK_PID}" || return 1
        sleep 0.25
    done
    return 1
}

record_perf_tool_role()
{
    TOOL_PGID="$(wait_for_isolated_pgid "${TOOL_PID}")" || return 1
    local deadline=$((SECONDS + 5)) tool_exe=""
    while (( SECONDS < deadline )); do
        tool_exe="$(readlink -f "/proc/${TOOL_PID}/exe" 2>/dev/null || true)"
        [[ "$(basename "${tool_exe}")" == perf ]] && break
        sleep 0.05
    done
    [[ "$(basename "${tool_exe}")" == perf ]] || return 1
    TOOL_STARTTIME="$(process_starttime "${TOOL_PID}")"
    record_role tool "${TOOL_PID}" "${TOOL_PGID}" "${TOOL_STARTTIME}"
}

start_perf()
{
    [[ -x "${PERF_PATH}" ]] || return 1
    mkfifo "${OUTPUT_DIR}/perf-control.fifo" "${OUTPUT_DIR}/perf-ack.fifo"
    exec 8<> "${OUTPUT_DIR}/perf-control.fifo"
    exec 9<> "${OUTPUT_DIR}/perf-ack.fifo"
    PERF_CTL_FD_OPEN=true
    TRACEE_TIDS="$(find "/proc/${TRACEE_PID}/task" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' \
        | sort -n | paste -sd, -)"
    [[ -n "${TRACEE_TIDS}" ]] || return 1
    echo "tracee_tids=${TRACEE_TIDS}" >> "${OUTPUT_DIR}/run-manifest.txt"
    if [[ "${MODE}" == perf-stat ]]; then
        setsid taskset -c 1 "${PERF_PATH}" stat -D -1 -t "${TRACEE_TIDS}" \
            --control "fifo:${OUTPUT_DIR}/perf-control.fifo,${OUTPUT_DIR}/perf-ack.fifo" \
            -x, --no-big-num -e task-clock,context-switches,cpu-migrations,page-faults \
            -e cycles:u,instructions:u,branches:u,branch-misses:u,cache-references:u,cache-misses:u \
            -o "${OUTPUT_DIR}/perf-stat.csv" > "${OUTPUT_DIR}/perf-stat.stdout.log" \
            2> "${OUTPUT_DIR}/perf-stat.stderr.log" &
    else
        setsid taskset -c 1 "${PERF_PATH}" record -D -1 -t "${TRACEE_TIDS}" \
            --control "fifo:${OUTPUT_DIR}/perf-control.fifo,${OUTPUT_DIR}/perf-ack.fifo" \
            -e cpu-clock:u -F 9999 --call-graph fp -o "${OUTPUT_DIR}/perf.data" \
            > "${OUTPUT_DIR}/perf-record.stdout.log" 2> "${OUTPUT_DIR}/perf-record.stderr.log" &
    fi
    TOOL_PID=$!
    sleep 0.5
    process_is_alive "${TOOL_PID}" || return 1
    record_perf_tool_role || return 1
    perf_control enable
}

stop_perf()
{
    local final_tids rc=0
    final_tids="$(find "/proc/${TRACEE_PID}/task" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' \
        | sort -n | paste -sd, -)"
    [[ "${final_tids}" == "${TRACEE_TIDS}" ]] || invalidate "target thread set changed during perf"
    perf_control disable || true
    perf_control stop || true
    exec 8>&- 9>&-
    PERF_CTL_FD_OPEN=false
    signal_process INT "${TOOL_PID}" "${TOOL_PGID}" "${TOOL_STARTTIME}" || return 1
    if ! wait_for_dead "${TOOL_PID}" 20; then
        FORCED_STOP=true
        invalidate "perf did not finalize after SIGINT"
        signal_process TERM "${TOOL_PID}" "${TOOL_PGID}" "${TOOL_STARTTIME}" || true
        wait_for_dead "${TOOL_PID}" 5 \
            || signal_process KILL "${TOOL_PID}" "${TOOL_PGID}" "${TOOL_STARTTIME}" \
            || true
        wait_child "${TOOL_PID}" perf || true
        TOOL_REAPED=true
        return 1
    fi
    wait_child "${TOOL_PID}" perf || rc=$?
    TOOL_REAPED=true
    if (( rc != 0 && rc != 130 )); then
        ROLE_EXIT_FAILURE=true
        invalidate "perf exited abnormally (${rc})"
    fi
    python3 "${PROFILE_REPORT_PARSER}" perf-control "${OUTPUT_DIR}/perf-control.txt" \
        "${T0_MONOTONIC_NS}" "${T1_MONOTONIC_NS}" "${OUTPUT_DIR}/perf-window-quality.txt" \
        || invalidate "perf control ACKs do not enclose the formal window"
}

start_trace()
{
    TRACE_SESSION="alien-local-map-${ROS_DOMAIN_ID}-$$"
    mkdir -p "${OUTPUT_DIR}/trace"
    list_ros_trace_events > "${OUTPUT_DIR}/trace-event-list.txt" 2>&1 || return 1
    grep -Eq 'ros2:callback_(start|end)' "${OUTPUT_DIR}/trace-event-list.txt" || return 1
    ros2 trace start "${TRACE_SESSION}" -p "${OUTPUT_DIR}/trace" -l \
        > "${OUTPUT_DIR}/trace-start.log" 2>&1 || return 1
    lttng list "${TRACE_SESSION}" > "${OUTPUT_DIR}/trace-session-start.txt" 2>&1 || return 1
    grep -Eq "(Recording|Tracing) session ${TRACE_SESSION}: \[active\]" \
        "${OUTPUT_DIR}/trace-session-start.txt" || return 1
    TRACE_STARTED=true
    echo "trace_session=${TRACE_SESSION}" >> "${OUTPUT_DIR}/run-manifest.txt"
}

stop_trace()
{
    ros2 trace stop "${TRACE_SESSION}" > "${OUTPUT_DIR}/trace-stop.log" 2>&1 \
        || invalidate "ros2 trace stop failed"
    TRACE_STARTED=false
}

start_checkpoint_sampler()
{
    local safety_args=()
    [[ "${MODE}" == capacity-ramp ]] && safety_args+=(--capacity-safety)
    setsid taskset -c 1 env PYTHONPATH="${SCRIPT_DIR}" python3 -m lib.profile_local_map_sampler \
        "${TRACEE_PID}" "${TRACEE_STARTTIME}" "${OUTPUT_DIR}/states.csv" \
        "${OUTPUT_DIR}/resource-samples.csv" "${OUTPUT_DIR}/memory-checkpoints.csv" \
        "${OUTPUT_DIR}/safety-stop.txt" "${safety_args[@]}" \
        > "${OUTPUT_DIR}/sampler.log" 2>&1 &
    SAMPLER_PID=$!
    SAMPLER_PGID="$(wait_for_isolated_pgid "${SAMPLER_PID}")" || return 1
    SAMPLER_STARTTIME="$(process_starttime "${SAMPLER_PID}")"
    wait_for_process_identity "${SAMPLER_PID}" "$(command -v python3)" \
        lib.profile_local_map_sampler || return 1
    record_role checkpoint_sampler "${SAMPLER_PID}" "${SAMPLER_PGID}" "${SAMPLER_STARTTIME}"
}

start_pidstat()
{
    setsid taskset -c 1 stdbuf -oL -eL pidstat -h -u -r -p "${TRACEE_PID}" \
        "${PIDSTAT_INTERVAL_S}" > "${OUTPUT_DIR}/pidstat.txt" 2>&1 &
    PIDSTAT_PID=$!
    PIDSTAT_PGID="$(wait_for_isolated_pgid "${PIDSTAT_PID}")" || return 1
    wait_for_process_identity "${PIDSTAT_PID}" pidstat "-p ${TRACEE_PID}" || return 1
    PIDSTAT_STARTTIME="$(process_starttime "${PIDSTAT_PID}")"
    record_role pidstat "${PIDSTAT_PID}" "${PIDSTAT_PGID}" "${PIDSTAT_STARTTIME}"
    PIDSTAT_START_MONOTONIC_NS="$(monotonic_ns)"
    {
        echo "pidstat_start_monotonic_ns=${PIDSTAT_START_MONOTONIC_NS}"
        echo "pidstat_interval_s=${PIDSTAT_INTERVAL_S}"
    } >> "${OUTPUT_DIR}/run-manifest.txt"
}

prepare_stage_trace()
{
    local shm_report="${OUTPUT_DIR}/stage-trace-shm-headroom.txt"
    if ! lttng_ust_shm_headroom "$(nproc)" "${STAGE_TRACE_SUBBUF_SIZE_BYTES}" \
        "${STAGE_TRACE_NUM_SUBBUF}" "${STAGE_TRACE_SHM_MARGIN_BYTES}" \
        > "${shm_report}"; then
        cat "${shm_report}" >> "${OUTPUT_DIR}/run-manifest.txt"
        invalidate "insufficient /dev/shm headroom for the LTTng UST stage channel"
        return 1
    fi
    cat "${shm_report}" >> "${OUTPUT_DIR}/run-manifest.txt"
    STAGE_TRACE_SESSION="alien-local-map-stage-${ROS_DOMAIN_ID}-$$"
    mkdir -p "${OUTPUT_DIR}/stage-trace"
    lttng create "${STAGE_TRACE_SESSION}" --output="${OUTPUT_DIR}/stage-trace" \
        > "${OUTPUT_DIR}/stage-trace-create.log" 2>&1 || return 1
    STAGE_TRACE_CREATED=true
    lttng enable-channel --userspace --session="${STAGE_TRACE_SESSION}" \
        --discard --subbuf-size="${STAGE_TRACE_SUBBUF_SIZE_BYTES}" \
        --num-subbuf="${STAGE_TRACE_NUM_SUBBUF}" stage \
        > "${OUTPUT_DIR}/stage-trace-channel.log" 2>&1 || return 1
    : > "${OUTPUT_DIR}/stage-trace-events.log"
    local event
    for event in "${STAGE_SELECTED_EVENTS[@]}"; do
        lttng enable-event --userspace --session="${STAGE_TRACE_SESSION}" --channel=stage \
            "perception_local_map_stage:${event}" \
            >> "${OUTPUT_DIR}/stage-trace-events.log" 2>&1 || return 1
    done
    lttng add-context --userspace --session="${STAGE_TRACE_SESSION}" --channel=stage \
        --type=vpid --type=vtid --type=procname \
        > "${OUTPUT_DIR}/stage-trace-context.log" 2>&1 || return 1
    lttng start "${STAGE_TRACE_SESSION}" > "${OUTPUT_DIR}/stage-trace-start.log" 2>&1 \
        || return 1
    STAGE_TRACE_STARTED=true
}

validate_stage_session_event_set()
{
    local session_evidence="$1" event selected expected
    for event in "${STAGE_PROVIDER_EVENTS[@]}"; do
        expected=false
        for selected in "${STAGE_SELECTED_EVENTS[@]}"; do
            if [[ "${event}" == "${selected}" ]]; then
                expected=true
                break
            fi
        done
        if [[ "${expected}" == true ]]; then
            grep -Fq "perception_local_map_stage:${event} (type: tracepoint) [enabled]" \
                "${session_evidence}" || return 1
        elif grep -Fq "perception_local_map_stage:${event}" "${session_evidence}"; then
            return 1
        fi
    done
}

validate_stage_trace()
{
    local event deadline=$((SECONDS + 10)) events_ready=false
    while (( SECONDS < deadline )); do
        if lttng list --userspace > "${OUTPUT_DIR}/stage-ust-events.txt" 2>&1; then
            events_ready=true
            for event in "${STAGE_PROVIDER_EVENTS[@]}"; do
                if ! grep -Fq "perception_local_map_stage:${event}" \
                    "${OUTPUT_DIR}/stage-ust-events.txt"; then
                    events_ready=false
                    break
                fi
            done
            [[ "${events_ready}" == true ]] && break
        fi
        sleep 0.1
    done
    [[ "${events_ready}" == true ]] || return 1
    lttng list "${STAGE_TRACE_SESSION}" > "${OUTPUT_DIR}/stage-session-start.txt" 2>&1 \
        || return 1
    grep -Eq "(Recording|Tracing) session ${STAGE_TRACE_SESSION}: \[active\]" \
        "${OUTPUT_DIR}/stage-session-start.txt" || return 1
    validate_stage_session_event_set "${OUTPUT_DIR}/stage-session-start.txt" || return 1
    echo "stage_trace_session=${STAGE_TRACE_SESSION}" >> "${OUTPUT_DIR}/run-manifest.txt"
    sleep 0.25
}

stop_stage_trace()
{
    sleep 0.25
    local failed=false
    lttng stop "${STAGE_TRACE_SESSION}" > "${OUTPUT_DIR}/stage-trace-stop.log" 2>&1 \
        || failed=true
    lttng list "${STAGE_TRACE_SESSION}" > "${OUTPUT_DIR}/stage-session-final.txt" 2>&1 \
        || failed=true
    if [[ "${failed}" == false ]] \
        && ! validate_stage_session_event_set "${OUTPUT_DIR}/stage-session-final.txt"; then
        failed=true
    fi
    lttng destroy "${STAGE_TRACE_SESSION}" > "${OUTPUT_DIR}/stage-trace-destroy.log" 2>&1 \
        || failed=true
    STAGE_TRACE_STARTED=false
    STAGE_TRACE_CREATED=false
    if [[ "${failed}" == true ]]; then
        invalidate "stage trace did not stop, report counters, and destroy normally"
        return 1
    fi
    STAGE_TRACE_NORMAL_COMPLETION=true
}

monitor_window()
{
    local roles=(
        "fixture=${FIXTURE_PID}=${FIXTURE_STARTTIME}"
        "sink=${SINK_PID}=${SINK_STARTTIME}"
        "tracee=${TRACEE_PID}=${TRACEE_STARTTIME}"
        "pidstat=${PIDSTAT_PID}=${PIDSTAT_STARTTIME}"
        "checkpoint_sampler=${SAMPLER_PID}=${SAMPLER_STARTTIME}"
    )
    [[ "${MODE}" == perf-stat || "${MODE}" == perf-record ]] \
        && roles+=("perf=${TOOL_PID}=${TOOL_STARTTIME}")
    [[ "${MODE}" == heaptrack || "${MODE}" == valgrind-* ]] \
        && roles+=("instrumentation_controller=${LAUNCHER_PID}=${LAUNCHER_STARTTIME}")
    local trace_session=""
    [[ "${MODE}" == ros-trace ]] && trace_session="${TRACE_SESSION}"
    [[ "${MODE}" == stage-latency ]] && trace_session="${STAGE_TRACE_SESSION}"
    python3 "${PROFILE_ROLE_MONITOR}" "${DURATION}" "${trace_session}" \
        "${OUTPUT_DIR}/trace-session-monitor.txt" "${OUTPUT_DIR}/role-monitor.txt" "${roles[@]}"
}

monitor_capacity_window()
{
    local deadline=$((SECONDS + DURATION)) reached_at="" revision
    while (( SECONDS < deadline )); do
        for role in "${FIXTURE_PID}:${FIXTURE_STARTTIME}" "${SINK_PID}:${SINK_STARTTIME}" \
            "${TRACEE_PID}:${TRACEE_STARTTIME}" "${SAMPLER_PID}:${SAMPLER_STARTTIME}" \
            "${PIDSTAT_PID}:${PIDSTAT_STARTTIME}"; do
            process_identity_matches "${role%%:*}" "${role#*:}" || return 1
        done
        if [[ -f "${OUTPUT_DIR}/safety-stop.txt" ]]; then
            invalidate "capacity safety gate requested an early normal stop"
            return 0
        fi
        revision="$(latest_revision)"
        if [[ "${ORACLE_STATUS}" == covered && "${REQUIRED_END_REVISION}" =~ ^[0-9]+$ \
            && "${revision}" -ge "${REQUIRED_END_REVISION}" ]]; then
            [[ -n "${reached_at}" ]] || reached_at="${SECONDS}"
            if (( SECONDS - reached_at >= 2 )); then
                echo "capacity_end_revision=${revision}" >> "${OUTPUT_DIR}/run-manifest.txt"
                return 0
            fi
        fi
        sleep 0.25
    done
    if [[ "${ORACLE_STATUS}" == covered ]]; then
        invalidate "capacity ramp did not reach its required post window"
    fi
}

stop_target()
{
    local rc=0
    if process_identity_matches "${TRACEE_PID}" "${TRACEE_STARTTIME}" "${TRACEE_PGID}"; then
        kill -INT "${TRACEE_PID}" 2>/dev/null || true
    else
        ROLE_EXIT_FAILURE=true
        invalidate "target identity changed before normal stop"
    fi
    if ! wait_for_dead "${TRACEE_PID}" 30; then
        FORCED_STOP=true
        invalidate "target did not exit after SIGINT"
        kill -TERM "${TRACEE_PID}" 2>/dev/null || true
        wait_for_dead "${TRACEE_PID}" 5 || kill -KILL "${TRACEE_PID}" 2>/dev/null || true
    fi
    if ! wait_for_dead "${LAUNCHER_PID}" 30; then
        FORCED_STOP=true
        invalidate "target launcher/tool did not finalize"
        signal_process TERM "${LAUNCHER_PID}" "${LAUNCHER_PGID}" "${LAUNCHER_STARTTIME}" || true
    fi
    wait_child "${LAUNCHER_PID}" launcher || rc=$?
    LAUNCHER_REAPED=true
    TARGET_STOPPED=true
    [[ "${TOOL_PID}" == "${LAUNCHER_PID}" ]] && TOOL_REAPED=true
    echo "tracee_exit_code=${rc}" >> "${OUTPUT_DIR}/exit-codes.txt"
    case "${MODE}" in
        valgrind-memcheck)
            MEMCHECK_EXIT_CODE="${rc}"
            [[ "${rc}" -eq 0 || "${rc}" -eq 42 ]] || invalidate "Memcheck exited abnormally (${rc})"
            ;;
        *)
            [[ "${rc}" -eq 0 ]] || invalidate "target/tool exited abnormally (${rc})"
            ;;
    esac
}

generate_reports()
{
    case "${MODE}" in
        perf-stat)
            python3 "${PROFILE_REPORT_PARSER}" perf-stat "${OUTPUT_DIR}/perf-stat.csv" \
                "${OUTPUT_DIR}/perf-stat-quality.txt" || invalidate "perf stat report gate failed"
            ;;
        perf-record)
            "${PERF_PATH}" report -i "${OUTPUT_DIR}/perf.data" --stdio --header --no-children \
                --sort comm,dso,symbol --percent-limit 0 > "${OUTPUT_DIR}/perf-report.txt" \
                2> "${OUTPUT_DIR}/perf-report.stderr.log" || invalidate "perf report failed"
            "${PERF_PATH}" buildid-list -i "${OUTPUT_DIR}/perf.data" \
                > "${OUTPUT_DIR}/perf-buildid-list.txt" 2>&1 || invalidate "perf build-id list failed"
            grep -q "${TARGET_BUILD_ID}" "${OUTPUT_DIR}/perf-buildid-list.txt" \
                || invalidate "perf data lacks target build ID"
            "${PERF_PATH}" report -i "${OUTPUT_DIR}/perf.data" --stdio --no-children \
                --call-graph none --sort overhead,comm,dso,symbol --percent-limit 0 \
                -t '|' -F overhead,sample,comm,dso,symbol > "${OUTPUT_DIR}/perf-symbols.txt" \
                2> "${OUTPUT_DIR}/perf-symbols.stderr.log" || invalidate "perf symbols failed"
            python3 "${PROFILE_REPORT_PARSER}" perf-record "${OUTPUT_DIR}/perf-symbols.txt" \
                "${OUTPUT_DIR}/perf-report.txt" "${OUTPUT_DIR}/perf-quality.txt" \
                "${OUTPUT_DIR}/perf-top10.txt" "${DURATION}" || invalidate "perf record report gate failed"
            ;;
        heaptrack)
            mapfile -t files < <(find "${OUTPUT_DIR}" -maxdepth 1 -name 'heaptrack*.gz' -type f)
            if (( ${#files[@]} != 1 )); then
                invalidate "Heaptrack must produce exactly one primary artifact"
            else
                heaptrack_print "${files[0]}" > "${OUTPUT_DIR}/heaptrack-report.txt" 2>&1 \
                    || invalidate "heaptrack_print failed"
                heaptrack_print -f "${files[0]}" -M "${OUTPUT_DIR}/heaptrack-massif.out" \
                    > "${OUTPUT_DIR}/heaptrack-massif.stdout.log" \
                    2> "${OUTPUT_DIR}/heaptrack-massif.stderr.log" \
                    || invalidate "heaptrack Massif timeline generation failed"
                python3 "${PROFILE_REPORT_PARSER}" heaptrack-massif \
                    "${OUTPUT_DIR}/heaptrack-massif.out" \
                    "${OUTPUT_DIR}/heaptrack-massif-quality.txt" \
                    "${EXPECTED_TARGET_EXE}" "${DURATION}" \
                    || invalidate "heaptrack Massif timeline gate failed"
                python3 "${PROFILE_REPORT_PARSER}" heaptrack "${OUTPUT_DIR}/heaptrack-report.txt" \
                    "${OUTPUT_DIR}/heaptrack-quality.txt" "${DURATION}" \
                    || invalidate "Heaptrack report gate failed"
            fi
            ;;
        valgrind-massif)
            mapfile -t files < <(find "${OUTPUT_DIR}" -maxdepth 1 -name 'massif.*.out' -type f)
            if (( ${#files[@]} != 1 )); then
                invalidate "Massif must produce exactly one primary artifact"
            else
                ms_print "${files[0]}" > "${OUTPUT_DIR}/massif-report.txt" 2>&1 || true
                python3 "${PROFILE_REPORT_PARSER}" massif "${files[0]}" \
                    "${OUTPUT_DIR}/massif-quality.txt" "${EXPECTED_TARGET_EXE}" "${DURATION}" \
                    || invalidate "Massif report gate failed"
            fi
            ;;
        valgrind-memcheck)
            mapfile -t files < <(find "${OUTPUT_DIR}" -maxdepth 1 -name 'memcheck.*.log' -type f)
            if (( ${#files[@]} != 1 )); then
                invalidate "Memcheck must produce exactly one primary artifact"
            else
                python3 "${PROFILE_REPORT_PARSER}" memcheck "${files[0]}" \
                    "${OUTPUT_DIR}/memcheck-summary.txt" "${OUTPUT_DIR}/memcheck-quality.txt" \
                    "${EXPECTED_TARGET_EXE}" || invalidate "Memcheck report gate failed"
                local finding="$(awk -F= '$1 == "finding" {print $2}' "${OUTPUT_DIR}/memcheck-quality.txt" 2>/dev/null)"
                if [[ "${MEMCHECK_EXIT_CODE}" == 42 && "${finding}" == true ]]; then
                    cp "${OUTPUT_DIR}/memcheck-quality.txt" "${OUTPUT_DIR}/memcheck-finding.txt"
                elif [[ "${MEMCHECK_EXIT_CODE}" != 0 || "${finding}" != false ]]; then
                    invalidate "Memcheck exit code and parsed finding disagree"
                fi
            fi
            ;;
        ros-trace)
            local trace_root callback_count take_count publish_count
            trace_root="$(find "${OUTPUT_DIR}/trace" -mindepth 1 -maxdepth 2 -type d -name ust -printf '%h\n' | head -1)"
            [[ -n "${trace_root}" ]] || { invalidate "LTTng trace data is missing"; return; }
            babeltrace2 convert --clock-seconds --begin="${T0_REALTIME}" --end="${T1_REALTIME}" \
                "${trace_root}" -w "${OUTPUT_DIR}/trace-window.txt" || invalidate "trace conversion failed"
            callback_count="$(grep -E "ros2:callback_(start|end).*vpid = ${TRACEE_PID}([, }]|$)" "${OUTPUT_DIR}/trace-window.txt" | wc -l)"
            take_count="$(grep -E "ros2:(rcl|rmw)_take.*vpid = ${TRACEE_PID}([, }]|$)" "${OUTPUT_DIR}/trace-window.txt" | wc -l)"
            publish_count="$(grep -E "ros2:(rclcpp|rcl)_publish.*vpid = ${TRACEE_PID}([, }]|$)" "${OUTPUT_DIR}/trace-window.txt" | wc -l)"
            printf 'trace_callback=%s\ntrace_take=%s\ntrace_publish=%s\n' \
                "${callback_count}" "${take_count}" "${publish_count}" > "${OUTPUT_DIR}/trace-counts.txt"
            ;;
        stage-latency)
            local trace_root
            trace_root="$(find "${OUTPUT_DIR}/stage-trace" -type f -name metadata -printf '%h\n' \
                | head -1)"
            if [[ -z "${trace_root}" ]]; then
                invalidate "stage trace CTF metadata is missing"
                return
            fi
            babeltrace2 convert --clock-seconds \
                "${trace_root}" -w "${OUTPUT_DIR}/stage-trace-events.txt" \
                || invalidate "stage trace event conversion failed"
            babeltrace2 convert -c sink.text.details "${trace_root}" \
                > "${OUTPUT_DIR}/stage-trace-details.txt" 2>&1 \
                || invalidate "stage trace detail conversion failed"
            local completion_args=()
            [[ "${STAGE_TRACE_NORMAL_COMPLETION}" == true ]] \
                && completion_args+=(--normal-completion)
            PYTHONPATH="${SCRIPT_DIR}" python3 -m lib.stage_latency_analysis \
                "${OUTPUT_DIR}/stage-trace-events.txt" \
                "${OUTPUT_DIR}/stage-session-final.txt" "${TRACEE_PID}" \
                "${T0_REALTIME}" "${T1_REALTIME}" \
                "${OUTPUT_DIR}/stage-latency.csv" \
                "${OUTPUT_DIR}/stage-probe-quality.txt" \
                --event-set "${STAGE_EVENT_SET}" "${completion_args[@]}" \
                || invalidate "stage trace pairing, loss, or completeness gate failed"
            ;;
    esac
}

if [[ "${MODE}" == stage-latency ]]; then
    prepare_stage_trace || {
        [[ "${VALID}" == false ]] \
            || invalidate "profiling-only UST stage session did not start"
        exit 1
    }
fi
start_target || exit 1
start_sink || { invalidate "sink did not start"; exit 1; }
start_fixture || { invalidate "fixture did not start"; exit 1; }
wait_for_graph || { invalidate "required C2 nodes did not become ready"; exit 1; }
wait_for_graph_snapshot || { invalidate "C2 graph gate failed or legacy path is present"; exit 1; }
capture_parameters_and_tf || { invalidate "parameter or frozen TF gate failed"; exit 1; }
wait_for_warmup || { invalidate "workload did not reach its required warmup revision"; exit 1; }
start_checkpoint_sampler || { invalidate "checkpoint sampler did not start"; exit 1; }

case "${MODE}" in
    perf-stat|perf-record)
        start_perf || { invalidate "perf did not start with control ACK"; exit 1; }
        ;;
    ros-trace)
        start_trace || { invalidate "ROS trace did not start"; exit 1; }
        ;;
    stage-latency)
        validate_stage_trace || { invalidate "profiling-only UST stage provider is incomplete"; exit 1; }
        ;;
esac

start_pidstat || { invalidate "pidstat sampler did not start"; exit 1; }
T0_MONOTONIC_NS="$(monotonic_ns)"
T0_REALTIME="$(realtime_stamp)"
if (( PIDSTAT_START_MONOTONIC_NS > T0_MONOTONIC_NS \
    || T0_MONOTONIC_NS - PIDSTAT_START_MONOTONIC_NS > PIDSTAT_BRACKET_MARGIN_NS )); then
    invalidate "pidstat did not start within two seconds before the formal window"
    exit 1
fi
echo "pidstat_start_margin_ns=$((T0_MONOTONIC_NS - PIDSTAT_START_MONOTONIC_NS))" \
    >> "${OUTPUT_DIR}/run-manifest.txt"
if [[ "${MODE}" == capacity-ramp ]]; then
    monitor_capacity_window || { invalidate "required role exited during capacity ramp"; exit 1; }
else
    window_failure=""
    if ! window_failure="$(monitor_window)"; then
        invalidate "${window_failure:-required role} exited during the formal window"
        exit 1
    fi
fi
T1_REALTIME="$(realtime_stamp)"
T1_MONOTONIC_NS="$(monotonic_ns)"
PIDSTAT_STOP_MONOTONIC_NS="$(monotonic_ns)"
{
    echo "t0_monotonic_ns=${T0_MONOTONIC_NS}"
    echo "t1_monotonic_ns=${T1_MONOTONIC_NS}"
    echo "t0_realtime=${T0_REALTIME}"
    echo "t1_realtime=${T1_REALTIME}"
    echo "pidstat_stop_monotonic_ns=${PIDSTAT_STOP_MONOTONIC_NS}"
    echo "pidstat_stop_margin_ns=$((PIDSTAT_STOP_MONOTONIC_NS - T1_MONOTONIC_NS))"
} >> "${OUTPUT_DIR}/run-manifest.txt"
if ! validate_pidstat_bracket "${PIDSTAT_START_MONOTONIC_NS}" \
    "${T0_MONOTONIC_NS}" "${T1_MONOTONIC_NS}" "${PIDSTAT_STOP_MONOTONIC_NS}" \
    "${PIDSTAT_BRACKET_MARGIN_NS}"; then
    invalidate "pidstat did not tightly bracket the formal window"
fi
stop_group "${PIDSTAT_PID}" "${PIDSTAT_PGID}" pidstat "${PIDSTAT_STARTTIME}" TERM
PIDSTAT_PID=""

case "${MODE}" in
    perf-stat|perf-record) stop_perf ;;
    ros-trace) stop_trace ;;
    stage-latency) stop_stage_trace ;;
esac
stop_group "${SAMPLER_PID}" "${SAMPLER_PGID}" checkpoint_sampler "${SAMPLER_STARTTIME}" TERM
SAMPLER_PID=""
stop_target
stop_group "${SINK_PID}" "${SINK_PGID}" sink "${SINK_STARTTIME}"
SINK_PID=""
stop_group "${FIXTURE_PID}" "${FIXTURE_PGID}" fixture "${FIXTURE_STARTTIME}"
FIXTURE_PID=""

generate_reports
if [[ "${MODE}" == asan-smoke || "${MODE}" == lsan-smoke ]]; then
    if grep -R -E 'ERROR: (AddressSanitizer|LeakSanitizer)|SUMMARY: (AddressSanitizer|LeakSanitizer)' \
        "${OUTPUT_DIR}"/asan* "${OUTPUT_DIR}"/lsan* "${OUTPUT_DIR}/target.log" 2>/dev/null; then
        invalidate "sanitizer reported an error"
    fi
fi
if [[ "${FORCED_STOP}" == false && "${ROLE_EXIT_FAILURE}" == false ]]; then
    NORMAL_COMPLETION=true
fi
{
    echo "valid=${VALID}"
    echo "normal_completion=${NORMAL_COMPLETION}"
} >> "${OUTPUT_DIR}/run-manifest.txt"
if [[ "${VALID}" == true && "${NORMAL_COMPLETION}" == true ]]; then
    python3 "${PROFILE_ANALYZER}" "${OUTPUT_DIR}" --output "${OUTPUT_DIR}/analysis-summary.json" \
        > "${OUTPUT_DIR}/analysis-summary.stdout.json" 2> "${OUTPUT_DIR}/analysis-summary.stderr.log" \
        || invalidate "C2 analyzer rejected the completed raw evidence"
fi

exit 0
