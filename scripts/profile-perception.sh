#!/usr/bin/env bash
set -uo pipefail

usage()
{
    echo "usage: $0 <mode> <install-prefix> <new-output-dir> <duration-seconds>" >&2
    echo "modes: plain-sample perf-stat perf-record ros-trace heaptrack asan-smoke lsan-smoke valgrind-memcheck valgrind-massif" >&2
}

if [[ $# -ne 4 ]]; then
    usage
    exit 2
fi

MODE="$1"
INSTALL_PREFIX="$(readlink -f "$2")"
OUTPUT_DIR="$3"
DURATION="$4"

case "${MODE}" in
    plain-sample|perf-stat|perf-record|ros-trace|heaptrack|asan-smoke|lsan-smoke|valgrind-memcheck|valgrind-massif)
        ;;
    *)
        usage
        exit 2
        ;;
esac

if [[ ! "${DURATION}" =~ ^[1-9][0-9]*$ ]]; then
    echo "duration must be a positive integer" >&2
    exit 2
fi
if [[ ! -f "${INSTALL_PREFIX}/setup.bash" ]]; then
    echo "missing install setup: ${INSTALL_PREFIX}/setup.bash" >&2
    exit 2
fi
if [[ -e "${OUTPUT_DIR}" ]]; then
    echo "output directory already exists: ${OUTPUT_DIR}" >&2
    exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKLOAD_YAML="${SCRIPT_DIR}/perception-profile-mixed.yaml"
OUTPUT_DIR="$(readlink -m "${OUTPUT_DIR}")"
mkdir -p "${OUTPUT_DIR}"

VALID=true
INVALID_REASONS=()
EXPECTED_TARGET_EXE=""
TARGET_SHA256=""
TARGET_BUILD_ID=""
FIXTURE_PID=""
FIXTURE_PGID=""
FIXTURE_STARTTIME=""
PREFLIGHT_PID=""
PREFLIGHT_PGID=""
PREFLIGHT_STARTTIME=""
SINK_PID=""
SINK_PGID=""
SINK_STARTTIME=""
LAUNCHER_PID=""
LAUNCHER_PGID=""
LAUNCHER_STARTTIME=""
LAUNCHER_REAPED=false
TOOL_PID=""
TOOL_PGID=""
TOOL_STARTTIME=""
TOOL_REAPED=false
TRACEE_PID=""
TRACEE_PGID=""
TRACEE_STARTTIME=""
PIDSTAT_PID=""
PIDSTAT_PGID=""
PIDSTAT_STARTTIME=""
SMEM_PID=""
SMEM_PGID=""
SMEM_STARTTIME=""
PERF_PATH=""
TRACEE_TIDS=""
PERF_CTL_FD_OPEN=false
TRACE_SESSION=""
TRACE_STARTED=false
MEMCHECK_EXIT_CODE=""
NORMAL_COMPLETION=false
FORCED_STOP=false
ROLE_EXIT_FAILURE=false
SCRIPT_PGID="$(ps -o pgid= -p "$$" | tr -d ' ')"

invalidate()
{
    local reason="$1"
    VALID=false
    INVALID_REASONS+=("${reason}")
    echo "INVALID: ${reason}" | tee -a "${OUTPUT_DIR}/invalid-reasons.txt" >&2
}

monotonic_ns()
{
    python3 -c 'import time; print(time.monotonic_ns())'
}

realtime_stamp()
{
    python3 -c 'import time; value=time.time_ns(); print(f"{value // 1000000000}.{value % 1000000000:09d}")'
}

wait_for_dead()
{
    local pid="$1"
    local timeout_seconds="$2"
    local deadline=$((SECONDS + timeout_seconds))
    while kill -0 "${pid}" 2>/dev/null; do
        if [[ -r "/proc/${pid}/stat" ]] \
            && [[ "$(awk '{print $3}' "/proc/${pid}/stat")" == Z ]]; then
            return 0
        fi
        if (( SECONDS >= deadline )); then
            return 1
        fi
        sleep 0.1
    done
    return 0
}

process_is_alive()
{
    local pid="$1"
    local state=""
    [[ -n "${pid}" ]] || return 1
    kill -0 "${pid}" 2>/dev/null || return 1
    state="$(awk '{print $3}' "/proc/${pid}/stat" 2>/dev/null || true)"
    if [[ -z "${state}" || "${state}" == Z ]]; then
        return 1
    fi
    return 0
}

process_starttime()
{
    local pid="$1"
    python3 - "${pid}" <<'PY'
import sys

with open(f"/proc/{sys.argv[1]}/stat", encoding="ascii") as stream:
    fields = stream.read().rpartition(")")[2].split()
print(fields[19])
PY
}

process_identity_matches()
{
    local pid="$1"
    local expected_starttime="$2"
    local expected_pgid="${3:-}"
    local actual_starttime actual_pgid
    process_is_alive "${pid}" || return 1
    actual_starttime="$(process_starttime "${pid}" 2>/dev/null || true)"
    [[ -n "${expected_starttime}" && "${actual_starttime}" == "${expected_starttime}" ]] \
        || return 1
    if [[ -n "${expected_pgid}" ]]; then
        actual_pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
        [[ "${actual_pgid}" == "${expected_pgid}" ]] || return 1
    fi
}

wait_for_process_identity()
{
    local pid="$1"
    local expected_exe="${2:-}"
    local cmdline_fragment="${3:-}"
    local deadline=$((SECONDS + 5))
    local actual_exe cmdline
    while (( SECONDS < deadline )); do
        process_is_alive "${pid}" || return 1
        actual_exe="$(readlink -f "/proc/${pid}/exe" 2>/dev/null || true)"
        cmdline="$(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null || true)"
        if { [[ -z "${expected_exe}" ]] \
                || [[ "${actual_exe}" == "$(readlink -f "${expected_exe}" 2>/dev/null || true)" ]] \
                || [[ "$(basename "${actual_exe}")" == "${expected_exe}" ]]; } \
            && { [[ -z "${cmdline_fragment}" ]] || [[ "${cmdline}" == *"${cmdline_fragment}"* ]]; }; then
            return 0
        fi
        sleep 0.05
    done
    return 1
}

wait_for_isolated_pgid()
{
    local pid="$1"
    local deadline=$((SECONDS + 5))
    local pgid=""
    while (( SECONDS < deadline )); do
        process_is_alive "${pid}" || return 1
        pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
        if [[ -n "${pgid}" && "${pgid}" != "${SCRIPT_PGID}" ]]; then
            echo "${pgid}"
            return 0
        fi
        sleep 0.05
    done
    return 1
}

wait_child()
{
    local pid="$1"
    local role="$2"
    local rc=0
    if wait "${pid}" 2>/dev/null; then
        rc=0
    else
        rc=$?
    fi
    printf '%s_exit_code=%s\n' "${role}" "${rc}" >> "${OUTPUT_DIR}/exit-codes.txt"
    return "${rc}"
}

signal_process()
{
    local signal="$1"
    local pid="$2"
    local pgid="$3"
    local starttime="$4"
    if ! process_identity_matches "${pid}" "${starttime}" "${pgid}"; then
        ROLE_EXIT_FAILURE=true
        invalidate "refusing to signal PID ${pid}: starttime or process group changed"
        return 1
    fi
    if [[ -n "${pgid}" && "${pgid}" != "${SCRIPT_PGID}" ]]; then
        kill -"${signal}" -- "-${pgid}" 2>/dev/null || true
    else
        kill -"${signal}" "${pid}" 2>/dev/null || true
    fi
}

stop_group()
{
    local pid="$1"
    local pgid="$2"
    local role="$3"
    local starttime="$4"
    local initial_signal="${5:-INT}"
    local rc=0
    [[ -n "${pid}" ]] || return 0
    if ! process_is_alive "${pid}"; then
        if wait_child "${pid}" "${role}"; then
            rc=0
        else
            rc=$?
        fi
        ROLE_EXIT_FAILURE=true
        invalidate "${role} exited before the requested normal stop (${rc})"
        return 0
    fi
    if [[ -n "${pgid}" && "${pgid}" == "${SCRIPT_PGID}" ]]; then
        ROLE_EXIT_FAILURE=true
        invalidate "${role} did not obtain an isolated process group"
    fi
    signal_process "${initial_signal}" "${pid}" "${pgid}" "${starttime}" || return 1
    if ! wait_for_dead "${pid}" 10; then
        FORCED_STOP=true
        invalidate "${role} did not exit after SIG${initial_signal}"
        signal_process TERM "${pid}" "${pgid}" "${starttime}" || return 1
        sleep 1
        if process_is_alive "${pid}"; then
            signal_process KILL "${pid}" "${pgid}" "${starttime}" || return 1
        fi
    fi
    if wait_child "${pid}" "${role}"; then
        rc=0
    else
        rc=$?
    fi
    if [[ "${rc}" -ne 0 \
        && ! ( "${initial_signal}" == INT && "${rc}" -eq 130 ) \
        && ! ( "${initial_signal}" == TERM && "${rc}" -eq 143 ) ]]; then
        ROLE_EXIT_FAILURE=true
        invalidate "${role} exited unexpectedly (${rc})"
    fi
}

stop_tracee()
{
    [[ -n "${TRACEE_PID}" ]] || return 0
    if ! process_is_alive "${TRACEE_PID}"; then
        ROLE_EXIT_FAILURE=true
        invalidate "tracee exited before the requested normal stop"
        return 0
    fi
    if ! process_identity_matches "${TRACEE_PID}" "${TRACEE_STARTTIME}" "${TRACEE_PGID}"; then
        ROLE_EXIT_FAILURE=true
        invalidate "refusing to signal tracee: PID identity changed"
        return 1
    fi
    kill -INT "${TRACEE_PID}" 2>/dev/null || true
    if ! wait_for_dead "${TRACEE_PID}" 20; then
        FORCED_STOP=true
        invalidate "tracee did not exit after SIGINT"
        kill -TERM "${TRACEE_PID}" 2>/dev/null || true
        sleep 1
        if process_identity_matches "${TRACEE_PID}" "${TRACEE_STARTTIME}" "${TRACEE_PGID}"; then
            kill -KILL "${TRACEE_PID}" 2>/dev/null || true
        fi
    fi
}

cleanup()
{
    local original_rc=$?
    trap - EXIT INT TERM

    if [[ "${TRACE_STARTED}" == true && -n "${TRACE_SESSION}" ]]; then
        taskset -c 1 ros2 trace stop "${TRACE_SESSION}" \
            >> "${OUTPUT_DIR}/trace-stop.log" 2>&1 || true
    fi
    if [[ -n "${PIDSTAT_PID}" ]]; then
        stop_group "${PIDSTAT_PID}" "${PIDSTAT_PGID}" pidstat "${PIDSTAT_STARTTIME}"
        PIDSTAT_PID=""
    fi
    if [[ -n "${SMEM_PID}" ]]; then
        stop_group "${SMEM_PID}" "${SMEM_PGID}" smem_sampler "${SMEM_STARTTIME}" TERM
        SMEM_PID=""
    fi
    if [[ "${PERF_CTL_FD_OPEN}" == true ]]; then
        exec 8>&- 9>&-
    fi
    if [[ "${NORMAL_COMPLETION}" != true ]]; then
        if [[ "${LAUNCHER_REAPED}" != true ]]; then
            stop_tracee
        fi
        if [[ -n "${TOOL_PID}" && "${TOOL_PID}" != "${LAUNCHER_PID}" \
            && "${TOOL_REAPED}" != true ]]; then
            if process_is_alive "${TOOL_PID}" && ! wait_for_dead "${TOOL_PID}" 5; then
                FORCED_STOP=true
                signal_process TERM "${TOOL_PID}" "${TOOL_PGID}" "${TOOL_STARTTIME}" || true
                wait_for_dead "${TOOL_PID}" 5 || true
            fi
            wait_child "${TOOL_PID}" cleanup_tool || true
            TOOL_REAPED=true
        fi
        if [[ -n "${LAUNCHER_PID}" && "${LAUNCHER_REAPED}" != true ]]; then
            if process_is_alive "${LAUNCHER_PID}" && ! wait_for_dead "${LAUNCHER_PID}" 20; then
                FORCED_STOP=true
                signal_process TERM "${LAUNCHER_PID}" "${LAUNCHER_PGID}" "${LAUNCHER_STARTTIME}" || true
                wait_for_dead "${LAUNCHER_PID}" 5 || true
            fi
            wait_child "${LAUNCHER_PID}" cleanup_launcher || true
            LAUNCHER_REAPED=true
            if [[ "${TOOL_PID}" == "${LAUNCHER_PID}" ]]; then
                TOOL_REAPED=true
            fi
        fi
    fi
    stop_group "${SINK_PID}" "${SINK_PGID}" "measurement_sink" "${SINK_STARTTIME}"
    stop_group "${PREFLIGHT_PID}" "${PREFLIGHT_PGID}" "preflight_sink" "${PREFLIGHT_STARTTIME}"
    stop_group "${FIXTURE_PID}" "${FIXTURE_PGID}" "fixture" "${FIXTURE_STARTTIME}"

    if [[ -d "${OUTPUT_DIR}" ]]; then
        {
            if [[ -f "${OUTPUT_DIR}/exit-codes.txt" ]]; then
                sed 's/^/role_/' "${OUTPUT_DIR}/exit-codes.txt"
            fi
            echo "valid=${VALID}"
            echo "normal_completion=${NORMAL_COMPLETION}"
            echo "forced_stop=${FORCED_STOP}"
            echo "role_exit_failure=${ROLE_EXIT_FAILURE}"
            echo "script_exit_code=${original_rc}"
            echo "finished_at=$(date --iso-8601=ns)"
        } >> "${OUTPUT_DIR}/run-manifest.txt"
        find "${OUTPUT_DIR}" -type f ! -name sha256sum.txt -print0 \
            | sort -z \
            | xargs -0 -r sha256sum > "${OUTPUT_DIR}/sha256sum.txt"
    fi

    if [[ "${VALID}" != true || "${NORMAL_COMPLETION}" != true ]]; then
        exit 1
    fi
    exit "${original_rc}"
}
trap cleanup EXIT INT TERM

set +u
SETUP_OK=true
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" || SETUP_OK=false
source "${INSTALL_PREFIX}/setup.bash" || SETUP_OK=false
set -u
if [[ "${SETUP_OK}" != true ]]; then
    invalidate "ROS or profiling install setup failed"
    exit 1
fi

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ALIEN_PROFILE_ROS_DOMAIN_ID:-$(( ( $(date +%s) + $$ ) % 200 + 20 ))}"
export RCUTILS_COLORIZED_OUTPUT=0
export LC_ALL=C

PROFILE_IMAGE_ID="${ALIEN_PROFILE_IMAGE_ID:-}"
if [[ ! "${PROFILE_IMAGE_ID}" =~ ^sha256:[0-9a-f]{64}$ ]]; then
    invalidate "ALIEN_PROFILE_IMAGE_ID must be the host-reported sha256 image ID"
    exit 1
fi

if (( $(nproc) < 2 )); then
    invalidate "at least two CPUs are required"
    exit 1
fi
if ! taskset -c 0 true || ! taskset -c 1 true; then
    invalidate "CPU 0 and CPU 1 must both be available"
    exit 1
fi

INPUT_PREFIX="$(ros2 pkg prefix perception_input_node 2>/dev/null || true)"
FIXTURE_PREFIX="$(ros2 pkg prefix perception_fixtures 2>/dev/null || true)"
INPUT_PREFIX_RESOLVED="$(readlink -f "${INPUT_PREFIX}" 2>/dev/null || true)"
FIXTURE_PREFIX_RESOLVED="$(readlink -f "${FIXTURE_PREFIX}" 2>/dev/null || true)"
if [[ -z "${INPUT_PREFIX_RESOLVED}" || -z "${FIXTURE_PREFIX_RESOLVED}" \
    || ( "${INPUT_PREFIX_RESOLVED}" != "${INSTALL_PREFIX}" \
        && "${INPUT_PREFIX_RESOLVED}" != "${INSTALL_PREFIX}/"* ) \
    || ( "${FIXTURE_PREFIX_RESOLVED}" != "${INSTALL_PREFIX}" \
        && "${FIXTURE_PREFIX_RESOLVED}" != "${INSTALL_PREFIX}/"* ) ]]; then
    invalidate "perception packages do not resolve from the requested install prefix"
    exit 1
fi
EXPECTED_TARGET_EXE="${INPUT_PREFIX}/lib/perception_input_node/perception_input_node"
FIXTURE_EXE="${FIXTURE_PREFIX}/lib/perception_fixtures/perception_fixture_publisher"
if [[ ! -x "${EXPECTED_TARGET_EXE}" || ! -x "${FIXTURE_EXE}" ]]; then
    invalidate "installed perception executables are missing"
    exit 1
fi
TARGET_SHA256="$(sha256sum "${EXPECTED_TARGET_EXE}" | awk '{print $1}')"
TARGET_BUILD_ID="$(readelf -n "${EXPECTED_TARGET_EXE}" 2>/dev/null | awk '/Build ID/ {print $3; exit}')"
if [[ ! "${TARGET_SHA256}" =~ ^[0-9a-f]{64}$ || -z "${TARGET_BUILD_ID}" ]]; then
    invalidate "target SHA-256 or build ID is unavailable"
    exit 1
fi

find_perf()
{
    local candidate
    if /usr/bin/perf --version >/dev/null 2>&1; then
        echo /usr/bin/perf
        return 0
    fi
    candidate="$(find /usr/lib/linux-tools -mindepth 2 -maxdepth 2 -name perf 2>/dev/null | sort -V | tail -1)"
    [[ -x "${candidate}" ]] || return 1
    "${candidate}" --version >/dev/null 2>&1 || return 1
    echo "${candidate}"
}

PERF_PATH="$(find_perf || true)"

{
    echo "mode=${MODE}"
    echo "duration_requested_s=${DURATION}"
    echo "started_at=$(date --iso-8601=ns)"
    echo "source_revision=$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || echo unavailable)"
    echo "source_dirty_count=$(git -C "${REPO_ROOT}" status --short 2>/dev/null | wc -l)"
    echo "profile_script_sha256=$(sha256sum "${BASH_SOURCE[0]}" | awk '{print $1}')"
    echo "workload_yaml_sha256=$(sha256sum "${WORKLOAD_YAML}" | awk '{print $1}')"
    if [[ -f "${SCRIPT_DIR}/analyze-perception-profile.py" ]]; then
        echo "analysis_script_sha256=$(sha256sum "${SCRIPT_DIR}/analyze-perception-profile.py" | awk '{print $1}')"
    else
        echo "analysis_script_sha256=unavailable"
    fi
    echo "container_id=$(hostname)"
    echo "image_id=${PROFILE_IMAGE_ID}"
    echo "kernel=$(uname -srvm)"
    echo "os=$(grep '^PRETTY_NAME=' /etc/os-release | cut -d= -f2-)"
    echo "compiler=$(c++ --version | head -1)"
    echo "rmw=${RMW_IMPLEMENTATION}"
    echo "ros_domain_id=${ROS_DOMAIN_ID}"
    echo "cpu_target=0"
    echo "cpu_fixture_sink=1"
    echo "cpu_allowed=$(awk '/Cpus_allowed_list/ {print $2}' /proc/self/status)"
    echo "cpu_max=$(cat /sys/fs/cgroup/cpu.max 2>/dev/null || echo unavailable)"
    echo "perf_event_paranoid=$(cat /proc/sys/kernel/perf_event_paranoid 2>/dev/null || echo unavailable)"
    awk '/^(CapInh|CapPrm|CapEff|CapBnd|CapAmb|NoNewPrivs|Seccomp|Seccomp_filters):/ {
        key = tolower($1)
        sub(/:$/, "", key)
        print "container_" key "=" $2
    }' /proc/self/status
    echo "loadavg=$(cat /proc/loadavg)"
    echo "target_exe=${EXPECTED_TARGET_EXE}"
    echo "target_sha256=${TARGET_SHA256}"
    echo "target_build_id=${TARGET_BUILD_ID}"
} > "${OUTPUT_DIR}/run-manifest.txt"
git -C "${REPO_ROOT}" status --short > "${OUTPUT_DIR}/git-status.txt" 2>&1 || true
ldd "${EXPECTED_TARGET_EXE}" > "${OUTPUT_DIR}/target-ldd.txt" 2>&1 || true
readelf -n "${EXPECTED_TARGET_EXE}" > "${OUTPUT_DIR}/target-notes.txt" 2>&1 || true
readelf -d "${EXPECTED_TARGET_EXE}" > "${OUTPUT_DIR}/target-dynamic.txt" 2>&1 || true
if [[ "${MODE}" == asan-smoke || "${MODE}" == lsan-smoke ]]; then
    if ! grep -Eq 'Shared library: \[libasan\.so' "${OUTPUT_DIR}/target-dynamic.txt"; then
        invalidate "sanitizer mode requires a target linked with libasan"
        exit 1
    fi
    echo "target_sanitizer=address" >> "${OUTPUT_DIR}/run-manifest.txt"
fi
{
    echo "perf=$(${PERF_PATH:-false} --version 2>&1 | head -1)"
    echo "heaptrack=$(heaptrack --version 2>&1 | head -1)"
    echo "heaptrack_print=$(heaptrack_print --version 2>&1 | head -1)"
    echo "valgrind=$(valgrind --version 2>&1 | head -1)"
    echo "pidstat=$(pidstat -V 2>&1 | head -1)"
    echo "smem=$(dpkg-query -W -f='${Version}' smem 2>/dev/null || echo unavailable)"
    echo "lttng=$(lttng --version 2>&1 | head -1)"
    echo "babeltrace2=$(babeltrace2 --version 2>&1 | head -1)"
    echo "ros2trace=$(ros2 trace -h 2>&1 | head -1)"
} > "${OUTPUT_DIR}/tool-versions.txt"

start_fixture()
{
    local fixture_env=(env)
    if [[ "${MODE}" == asan-smoke || "${MODE}" == lsan-smoke ]]; then
        fixture_env+=(ASAN_OPTIONS=detect_leaks=0:halt_on_error=1)
    fi
    setsid taskset -c 1 "${fixture_env[@]}" "${FIXTURE_EXE}" \
        --ros-args --params-file "${WORKLOAD_YAML}" \
        > "${OUTPUT_DIR}/fixture.log" 2>&1 &
    FIXTURE_PID=$!
    if ! FIXTURE_PGID="$(wait_for_isolated_pgid "${FIXTURE_PID}")"; then
        invalidate "fixture did not obtain an isolated process group"
        return 1
    fi
    if ! wait_for_process_identity "${FIXTURE_PID}" "${FIXTURE_EXE}"; then
        invalidate "fixture process identity did not stabilize"
        return 1
    fi
    FIXTURE_STARTTIME="$(process_starttime "${FIXTURE_PID}")"
    {
        echo "fixture_pid=${FIXTURE_PID}"
        echo "fixture_pgid=${FIXTURE_PGID:-unavailable}"
        echo "fixture_starttime=${FIXTURE_STARTTIME}"
        echo "fixture_affinity=$(taskset -pc "${FIXTURE_PID}" 2>&1 | tail -1)"
    } >> "${OUTPUT_DIR}/run-manifest.txt"
}

start_sink()
{
    local kind="$1"
    local output_file="$2"
    local sink_env=(env PYTHONUNBUFFERED=1)
    if [[ "${MODE}" == asan-smoke || "${MODE}" == lsan-smoke ]]; then
        local asan_runtime cxx_runtime
        asan_runtime="$(gcc -print-file-name=libasan.so)"
        cxx_runtime="$(c++ -print-file-name=libstdc++.so)"
        sink_env+=(
            "LD_PRELOAD=${asan_runtime}:${cxx_runtime}"
            ASAN_OPTIONS=detect_leaks=0:halt_on_error=1
        )
    fi
    setsid taskset -c 1 "${sink_env[@]}" ros2 topic echo \
        /perception/observations perception_interfaces/msg/LidarObservation \
        --qos-profile sensor_data --csv --field sensor_id \
        > "${output_file}" 2> "${OUTPUT_DIR}/${kind}-sink.stderr.log" &
    local pid=$!
    local pgid
    if [[ "${kind}" == preflight ]]; then
        PREFLIGHT_PID="${pid}"
    else
        SINK_PID="${pid}"
    fi
    if ! pgid="$(wait_for_isolated_pgid "${pid}")"; then
        invalidate "${kind} sink did not obtain an isolated process group"
        return 1
    fi
    if ! wait_for_process_identity "${pid}" "" "/perception/observations"; then
        invalidate "${kind} sink process identity did not stabilize"
        return 1
    fi
    local starttime
    starttime="$(process_starttime "${pid}")"
    if [[ "${kind}" == preflight ]]; then
        PREFLIGHT_PGID="${pgid}"
        PREFLIGHT_STARTTIME="${starttime}"
    else
        SINK_PGID="${pgid}"
        SINK_STARTTIME="${starttime}"
    fi
    {
        echo "${kind}_sink_pid=${pid}"
        echo "${kind}_sink_pgid=${pgid:-unavailable}"
        echo "${kind}_sink_starttime=${starttime}"
        echo "${kind}_sink_affinity=$(taskset -pc "${pid}" 2>&1 | tail -1)"
    } >> "${OUTPUT_DIR}/run-manifest.txt"
}

find_matching_descendant()
{
    local root_pid="$1"
    local deadline=$((SECONDS + 15))
    while (( SECONDS < deadline )); do
        local queue=("${root_pid}")
        local matches=()
        local current child exe
        while (( ${#queue[@]} > 0 )); do
            current="${queue[0]}"
            queue=("${queue[@]:1}")
            exe="$(readlink -f "/proc/${current}/exe" 2>/dev/null || true)"
            if [[ "${exe}" == "$(readlink -f "${EXPECTED_TARGET_EXE}")" ]]; then
                matches+=("${current}")
            fi
            while read -r child; do
                [[ -n "${child}" ]] && queue+=("${child}")
            done < <(pgrep -P "${current}" 2>/dev/null || true)
        done
        if (( ${#matches[@]} == 1 )); then
            echo "${matches[0]}"
            return 0
        fi
        if (( ${#matches[@]} > 1 )); then
            return 2
        fi
        sleep 0.1
    done
    return 1
}

wait_for_tracee_identity()
{
    local deadline=$((SECONDS + 5))
    local actual_exe=""
    local cmdline=""
    while (( SECONDS < deadline )); do
        process_is_alive "${TRACEE_PID}" || return 1
        if [[ "${MODE}" == valgrind-* ]]; then
            cmdline="$(tr '\0' ' ' < "/proc/${TRACEE_PID}/cmdline" 2>/dev/null || true)"
            [[ "${cmdline}" == *"${EXPECTED_TARGET_EXE}"* ]] && return 0
        else
            actual_exe="$(readlink -f "/proc/${TRACEE_PID}/exe" 2>/dev/null || true)"
            [[ "${actual_exe}" == "$(readlink -f "${EXPECTED_TARGET_EXE}")" ]] && return 0
        fi
        sleep 0.05
    done
    return 1
}

start_target()
{
    local target_args=("${EXPECTED_TARGET_EXE}" --ros-args --params-file "${WORKLOAD_YAML}")
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

    if ! process_is_alive "${LAUNCHER_PID}"; then
        invalidate "target launcher exited before its identity stabilized"
        return 1
    fi
    LAUNCHER_STARTTIME="$(process_starttime "${LAUNCHER_PID}")"
    if ! LAUNCHER_PGID="$(wait_for_isolated_pgid "${LAUNCHER_PID}")"; then
        invalidate "target launcher did not obtain an isolated process group"
        return 1
    fi
    if [[ "${MODE}" == heaptrack ]]; then
        if ! wait_for_process_identity "${LAUNCHER_PID}" heaptrack "${EXPECTED_TARGET_EXE}"; then
            invalidate "Heaptrack controller identity did not stabilize"
            return 1
        fi
        TRACEE_PID="$(find_matching_descendant "${LAUNCHER_PID}" || true)"
    fi
    if [[ -z "${TRACEE_PID}" ]] || ! kill -0 "${TRACEE_PID}" 2>/dev/null; then
        invalidate "unable to resolve a unique tracee PID"
        return 1
    fi
    if ! wait_for_tracee_identity; then
        invalidate "tracee process identity did not stabilize on the target ELF"
        return 1
    fi
    if ! TRACEE_PGID="$(wait_for_isolated_pgid "${TRACEE_PID}")"; then
        invalidate "tracee did not obtain an isolated process group"
        return 1
    fi
    TRACEE_STARTTIME="$(process_starttime "${TRACEE_PID}")"
    if [[ -n "${TOOL_PID}" ]]; then
        TOOL_PGID="${LAUNCHER_PGID}"
        TOOL_STARTTIME="${LAUNCHER_STARTTIME}"
    fi
    tr '\0' ' ' < "/proc/${TRACEE_PID}/cmdline" > "${OUTPUT_DIR}/tracee-cmdline.txt"

    {
        echo "launcher_pid=${LAUNCHER_PID}"
        echo "launcher_pgid=${LAUNCHER_PGID}"
        echo "launcher_starttime=${LAUNCHER_STARTTIME}"
        if [[ "${MODE}" == perf-stat || "${MODE}" == perf-record ]]; then
            echo "tool_pid=pending"
        else
            echo "tool_pid=${TOOL_PID:-none}"
            if [[ -n "${TOOL_PID}" ]]; then
                echo "tool_pgid=${TOOL_PGID}"
                echo "tool_starttime=${TOOL_STARTTIME}"
            fi
        fi
        echo "tracee_pid=${TRACEE_PID}"
        echo "tracee_pgid=${TRACEE_PGID}"
        echo "tracee_starttime=${TRACEE_STARTTIME}"
        echo "tracee_affinity=$(taskset -pc "${TRACEE_PID}" 2>&1 | tail -1)"
        echo "tracee_cgroup=$(tr '\n' ';' < "/proc/${TRACEE_PID}/cgroup")"
        awk '/^(Name|Pid|PPid|NSpid|Cpus_allowed_list|CapEff|NoNewPrivs|Seccomp|Seccomp_filters):/ {
            key = tolower($1)
            sub(/:$/, "", key)
            print "tracee_" key "=" $2
        }' "/proc/${TRACEE_PID}/status"
    } >> "${OUTPUT_DIR}/run-manifest.txt"
}

record_perf_tool_role()
{
    local deadline=$((SECONDS + 5))
    local tool_exe
    if ! TOOL_PGID="$(wait_for_isolated_pgid "${TOOL_PID}")"; then
        invalidate "perf tool did not obtain an isolated process group"
        return 1
    fi
    while (( SECONDS < deadline )); do
        process_is_alive "${TOOL_PID}" || break
        tool_exe="$(readlink -f "/proc/${TOOL_PID}/exe" 2>/dev/null || true)"
        [[ "$(basename "${tool_exe}")" == perf ]] && break
        sleep 0.05
    done
    if [[ "$(basename "${tool_exe}")" != perf ]]; then
        invalidate "perf tool process identity did not stabilize"
        return 1
    fi
    TOOL_STARTTIME="$(process_starttime "${TOOL_PID}")"
    sed -i "s/^tool_pid=pending$/tool_pid=${TOOL_PID}/" "${OUTPUT_DIR}/run-manifest.txt"
    tr '\0' ' ' < "/proc/${TOOL_PID}/cmdline" > "${OUTPUT_DIR}/tool-cmdline.txt"
    {
        echo "tool_pgid=${TOOL_PGID}"
        echo "tool_starttime=${TOOL_STARTTIME}"
        echo "tool_exe=${tool_exe}"
        echo "tool_affinity=$(taskset -pc "${TOOL_PID}" 2>&1 | tail -1)"
        echo "tool_cgroup=$(tr '\n' ';' < "/proc/${TOOL_PID}/cgroup")"
        awk '/^(Name|Pid|PPid|NSpid|Cpus_allowed_list):/ {
            key = tolower($1)
            sub(/:$/, "", key)
            print "tool_" key "=" $2
        }' "/proc/${TOOL_PID}/status"
    } >> "${OUTPUT_DIR}/run-manifest.txt"
}

wait_for_nodes()
{
    local deadline=$((SECONDS + 30))
    while (( SECONDS < deadline )); do
        local nodes
        nodes="$(ros2 node list 2>/dev/null || true)"
        if grep -Fxq /perception_fixture_publisher <<< "${nodes}" \
            && grep -Fxq /perception_input_node <<< "${nodes}"; then
            return 0
        fi
        if ! kill -0 "${FIXTURE_PID}" 2>/dev/null || ! kill -0 "${TRACEE_PID}" 2>/dev/null; then
            return 1
        fi
        sleep 0.25
    done
    return 1
}

validate_parameters()
{
    ros2 param dump /perception_fixture_publisher > "${OUTPUT_DIR}/fixture-params.yaml" 2> "${OUTPUT_DIR}/fixture-params.stderr.log" || return 1
    ros2 param dump /perception_input_node > "${OUTPUT_DIR}/input-params.yaml" 2> "${OUTPUT_DIR}/input-params.stderr.log" || return 1
    python3 - "${WORKLOAD_YAML}" "${OUTPUT_DIR}/fixture-params.yaml" "${OUTPUT_DIR}/input-params.yaml" <<'PY'
import math
import sys
import yaml

expected_path, fixture_path, input_path = sys.argv[1:]
with open(expected_path, encoding="utf-8") as stream:
    expected = yaml.safe_load(stream)

def load_actual(path, node_name):
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    for key, value in data.items():
        if key.lstrip("/") == node_name:
            return value["ros__parameters"]
    raise KeyError(f"missing node {node_name} in {path}")

def equal(expected_value, actual_value):
    if isinstance(expected_value, bool):
        return isinstance(actual_value, bool) and expected_value is actual_value
    if isinstance(expected_value, int):
        return (
            isinstance(actual_value, int)
            and not isinstance(actual_value, bool)
            and expected_value == actual_value
        )
    if isinstance(expected_value, float):
        return isinstance(actual_value, float) and math.isclose(
            expected_value, actual_value, rel_tol=1e-8, abs_tol=1e-9)
    if isinstance(expected_value, list):
        return isinstance(actual_value, list) and len(expected_value) == len(actual_value) and all(
            equal(left, right) for left, right in zip(expected_value, actual_value))
    return expected_value == actual_value

def lookup(actual, dotted_key):
    if dotted_key in actual:
        return actual[dotted_key]
    value = actual
    for part in dotted_key.split("."):
        if not isinstance(value, dict) or part not in value:
            raise KeyError(dotted_key)
        value = value[part]
    return value

for node_name, actual_path in (
    ("perception_fixture_publisher", fixture_path),
    ("perception_input_node", input_path),
):
    actual = load_actual(actual_path, node_name)
    expected_parameters = expected[node_name]["ros__parameters"]
    for key, expected_value in expected_parameters.items():
        try:
            actual_value = lookup(actual, key)
        except KeyError:
            raise SystemExit(f"{node_name}: missing parameter {key}")
        if not equal(expected_value, actual_value):
            raise SystemExit(
                f"{node_name}: {key}: expected {expected_value!r}, got {actual_value!r}")
PY
}

capture_topic_info()
{
    local prefix="$1"
    local topic file
    for topic in /fixture/scan/front /fixture/scan/rear /fixture/points /perception/observations; do
        file="${OUTPUT_DIR}/${prefix}-$(tr '/' '_' <<< "${topic}" | sed 's/^_//').txt"
        ros2 topic info "${topic}" --verbose > "${file}" 2>&1 || return 1
    done
}

validate_graph()
{
    local prefix="$1"
    capture_topic_info "${prefix}" || return 1
    local file
    for topic_name in fixture_scan_front fixture_scan_rear fixture_points; do
        file="${OUTPUT_DIR}/${prefix}-${topic_name}.txt"
        grep -Eq '^Publisher count: 1$' "${file}" || return 1
        grep -Eq '^Subscription count: 1$' "${file}" || return 1
        grep -q 'Node name: perception_fixture_publisher' "${file}" || return 1
        grep -q 'Node name: perception_input_node' "${file}" || return 1
    done
    file="${OUTPUT_DIR}/${prefix}-perception_observations.txt"
    grep -Eq '^Publisher count: 1$' "${file}" || return 1
    grep -Eq '^Subscription count: 1$' "${file}" || return 1
    grep -q 'Node name: perception_input_node' "${file}" || return 1
}

wait_for_sensor_ids()
{
    local file="$1"
    local sink_pid="$2"
    local deadline=$((SECONDS + 30))
    while (( SECONDS < deadline )); do
        local normalized
        normalized="$(tr -d '\r"' < "${file}" 2>/dev/null || true)"
        if grep -Fxq front <<< "${normalized}" \
            && grep -Fxq rear <<< "${normalized}" \
            && grep -Fxq top <<< "${normalized}"; then
            return 0
        fi
        process_is_alive "${sink_pid}" || return 1
        sleep 0.25
    done
    return 1
}

wait_for_graph_pair()
{
    local prefix="$1"
    local sink_pid="$2"
    local deadline=$((SECONDS + 20))
    while (( SECONDS < deadline )); do
        if validate_graph "${prefix}" >/dev/null 2>&1; then
            return 0
        fi
        kill -0 "${sink_pid}" 2>/dev/null || return 1
        sleep 0.25
    done
    return 1
}

perf_control()
{
    local command="$1"
    printf '%s\n' "${command}" >&8
    local ack=""
    if ! IFS= read -r -t 10 ack <&9 || [[ "${ack}" != ack ]]; then
        ROLE_EXIT_FAILURE=true
        invalidate "perf control ${command} ACK failed"
        return 1
    fi
    {
        printf '%s_ack=%s\n' "${command}" "${ack}"
        printf '%s_ack_monotonic_ns=%s\n' "${command}" "$(monotonic_ns)"
    } >> "${OUTPUT_DIR}/perf-control.txt"
}

start_perf()
{
    [[ -x "${PERF_PATH}" ]] || {
        invalidate "perf executable is unavailable"
        return 1
    }
    mkfifo "${OUTPUT_DIR}/perf-control.fifo" "${OUTPUT_DIR}/perf-ack.fifo"
    exec 8<> "${OUTPUT_DIR}/perf-control.fifo"
    exec 9<> "${OUTPUT_DIR}/perf-ack.fifo"
    PERF_CTL_FD_OPEN=true
    TRACEE_TIDS="$(find "/proc/${TRACEE_PID}/task" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' \
        | sort -n | paste -sd, -)"
    if [[ -z "${TRACEE_TIDS}" ]]; then
        invalidate "unable to resolve target thread IDs"
        return 1
    fi
    echo "tracee_tids=${TRACEE_TIDS}" >> "${OUTPUT_DIR}/run-manifest.txt"

    if [[ "${MODE}" == perf-stat ]]; then
        setsid taskset -c 1 "${PERF_PATH}" stat -D -1 -t "${TRACEE_TIDS}" \
            --control "fifo:${OUTPUT_DIR}/perf-control.fifo,${OUTPUT_DIR}/perf-ack.fifo" \
            -x, --no-big-num \
            -e task-clock,context-switches,cpu-migrations,page-faults \
            -e cycles:u,instructions:u,branches:u,branch-misses:u,cache-references:u,cache-misses:u \
            -o "${OUTPUT_DIR}/perf-stat.csv" \
            > "${OUTPUT_DIR}/perf-stat.stdout.log" 2> "${OUTPUT_DIR}/perf-stat.stderr.log" &
    else
        setsid taskset -c 1 "${PERF_PATH}" record -D -1 -t "${TRACEE_TIDS}" \
            --control "fifo:${OUTPUT_DIR}/perf-control.fifo,${OUTPUT_DIR}/perf-ack.fifo" \
            -e cpu-clock:u -F 9999 --call-graph fp \
            -o "${OUTPUT_DIR}/perf.data" \
            > "${OUTPUT_DIR}/perf-record.stdout.log" 2> "${OUTPUT_DIR}/perf-record.stderr.log" &
    fi
    TOOL_PID=$!
    TOOL_STARTTIME="$(process_starttime "${TOOL_PID}" 2>/dev/null || true)"
    sleep 0.5
    if ! process_is_alive "${TOOL_PID}"; then
        invalidate "perf exited before enable"
        return 1
    fi
    record_perf_tool_role || return 1
    perf_control enable
}

start_trace()
{
    local rc=0
    TRACE_SESSION="alien-perception-${ROS_DOMAIN_ID}-$$"
    mkdir -p "${OUTPUT_DIR}/trace"
    if taskset -c 1 ros2 trace --list > "${OUTPUT_DIR}/trace-event-list.txt" 2>&1; then
        echo "ros_trace_event_list_exit_code=0" >> "${OUTPUT_DIR}/exit-codes.txt"
    else
        rc=$?
        echo "ros_trace_event_list_exit_code=${rc}" >> "${OUTPUT_DIR}/exit-codes.txt"
        invalidate "ros2 trace event listing failed"
        return 1
    fi
    if ! grep -Eq 'ros2:callback_(start|end)' "${OUTPUT_DIR}/trace-event-list.txt" \
        || ! grep -Eq 'ros2:(rcl|rmw)_take' "${OUTPUT_DIR}/trace-event-list.txt" \
        || ! grep -Eq 'ros2:(rclcpp|rcl)_publish' "${OUTPUT_DIR}/trace-event-list.txt"; then
        invalidate "Jazzy tracepoint list is missing a required callback/take/publish category"
        return 1
    fi
    if taskset -c 1 ros2 trace start "${TRACE_SESSION}" -p "${OUTPUT_DIR}/trace" -l \
        > "${OUTPUT_DIR}/trace-start.log" 2>&1; then
        echo "ros_trace_start_exit_code=0" >> "${OUTPUT_DIR}/exit-codes.txt"
    else
        rc=$?
        echo "ros_trace_start_exit_code=${rc}" >> "${OUTPUT_DIR}/exit-codes.txt"
        invalidate "ros2 trace start failed"
        return 1
    fi
    if ! taskset -c 1 lttng list "${TRACE_SESSION}" \
        > "${OUTPUT_DIR}/trace-session-start.txt" 2>&1 \
        || ! grep -Eq "(Recording|Tracing) session ${TRACE_SESSION}: \\[active\\]" \
            "${OUTPUT_DIR}/trace-session-start.txt"; then
        invalidate "LTTng trace session did not become active"
        return 1
    fi
    TRACE_STARTED=true
    {
        echo "trace_session=${TRACE_SESSION}"
        echo "trace_control_cpu=1"
        echo "trace_consumer_affinity=shared_lttng_daemon_not_rebound"
    } >> "${OUTPUT_DIR}/run-manifest.txt"
}

start_plain_samplers()
{
    setsid taskset -c 1 stdbuf -oL -eL pidstat -h -u -r -p "${TRACEE_PID}" 1 \
        > "${OUTPUT_DIR}/pidstat.txt" 2>&1 &
    PIDSTAT_PID=$!
    if ! PIDSTAT_PGID="$(wait_for_isolated_pgid "${PIDSTAT_PID}")" \
        || ! wait_for_process_identity "${PIDSTAT_PID}" pidstat "-p ${TRACEE_PID}"; then
        invalidate "pidstat sampler identity did not stabilize"
        return 1
    fi
    PIDSTAT_STARTTIME="$(process_starttime "${PIDSTAT_PID}")"

    setsid taskset -c 1 bash -c '
        tracee_pid="$1"
        while kill -0 "${tracee_pid}" 2>/dev/null; do
            printf "sample_realtime=%s sample_monotonic_ns=%s\n" \
                "$(date --iso-8601=ns)" \
                "$(python3 -c '\''import time; print(time.monotonic_ns())'\'')"
            smem -H -c "pid uss pss rss command" 2>/dev/null \
                | awk -v target="${tracee_pid}" '\''$1 == target'\''
            if [[ -r "/proc/${tracee_pid}/smaps_rollup" ]]; then
                sed -n -e "/^Rss:/p" -e "/^Pss:/p" -e "/^Private_Clean:/p" \
                    -e "/^Private_Dirty:/p" "/proc/${tracee_pid}/smaps_rollup"
            fi
            echo
            sleep 10
        done
    ' alien-smem-sampler "${TRACEE_PID}" > "${OUTPUT_DIR}/smem-smaps.txt" 2>&1 &
    SMEM_PID=$!
    if ! SMEM_PGID="$(wait_for_isolated_pgid "${SMEM_PID}")" \
        || ! wait_for_process_identity "${SMEM_PID}" bash alien-smem-sampler; then
        invalidate "smem sampler identity did not stabilize"
        return 1
    fi
    SMEM_STARTTIME="$(process_starttime "${SMEM_PID}")"
    {
        echo "pidstat_pid=${PIDSTAT_PID}"
        echo "pidstat_pgid=${PIDSTAT_PGID}"
        echo "pidstat_starttime=${PIDSTAT_STARTTIME}"
        echo "pidstat_affinity=$(taskset -pc "${PIDSTAT_PID}" 2>&1 | tail -1)"
        echo "smem_sampler_pid=${SMEM_PID}"
        echo "smem_sampler_pgid=${SMEM_PGID}"
        echo "smem_sampler_starttime=${SMEM_STARTTIME}"
        echo "smem_sampler_affinity=$(taskset -pc "${SMEM_PID}" 2>&1 | tail -1)"
    } >> "${OUTPUT_DIR}/run-manifest.txt"
}

monitor_measurement_window()
{
    local roles=(
        "fixture=${FIXTURE_PID}=${FIXTURE_STARTTIME}"
        "tracee=${TRACEE_PID}=${TRACEE_STARTTIME}"
        "measurement_sink=${SINK_PID}=${SINK_STARTTIME}"
    )
    case "${MODE}" in
        perf-stat|perf-record)
            roles+=("perf=${TOOL_PID}=${TOOL_STARTTIME}")
            ;;
        heaptrack|valgrind-memcheck|valgrind-massif)
            roles+=("instrumentation_controller=${LAUNCHER_PID}=${LAUNCHER_STARTTIME}")
            ;;
        asan-smoke|lsan-smoke)
            roles+=("sanitized_target=${LAUNCHER_PID}=${LAUNCHER_STARTTIME}")
            ;;
        plain-sample)
            roles+=(
                "pidstat=${PIDSTAT_PID}=${PIDSTAT_STARTTIME}"
                "smem_sampler=${SMEM_PID}=${SMEM_STARTTIME}"
            )
            ;;
    esac

    local trace_session=""
    if [[ "${MODE}" == ros-trace ]]; then
        trace_session="${TRACE_SESSION}"
    fi

    taskset -c 1 python3 - "${DURATION}" "${trace_session}" \
        "${OUTPUT_DIR}/trace-session-monitor.txt" \
        "${OUTPUT_DIR}/role-monitor.txt" "${roles[@]}" <<'PY'
import os
import re
import subprocess
import sys
import time

duration = int(sys.argv[1])
trace_session = sys.argv[2]
trace_monitor_path = sys.argv[3]
role_monitor_path = sys.argv[4]
roles = []

def process_state(pid):
    with open(f"/proc/{pid}/stat", encoding="ascii") as stream:
        suffix = stream.read().rpartition(")")[2].split()
    return suffix[0], suffix[19]

for specification in sys.argv[5:]:
    role, raw_pid, expected_starttime = specification.split("=", 2)
    pid = int(raw_pid)
    try:
        state, starttime = process_state(pid)
    except (FileNotFoundError, ProcessLookupError, PermissionError, IndexError):
        print(role)
        raise SystemExit(1)
    if state == "Z" or starttime != expected_starttime:
        print(role)
        raise SystemExit(1)
    roles.append((role, pid, starttime))

with open(role_monitor_path, "w", encoding="utf-8") as stream:
    for role, pid, starttime in roles:
        stream.write(f"role={role} pid={pid} starttime={starttime}\n")

deadline = time.monotonic() + duration
next_trace_check = 0.0
while True:
    for role, pid, expected_starttime in roles:
        try:
            os.kill(pid, 0)
            state, starttime = process_state(pid)
        except (FileNotFoundError, ProcessLookupError, PermissionError, IndexError):
            print(role)
            raise SystemExit(1)
        if state == "Z" or starttime != expected_starttime:
            print(role)
            raise SystemExit(1)
    now = time.monotonic()
    if trace_session and now >= next_trace_check:
        result = subprocess.run(
            ["lttng", "list", trace_session],
            check=False,
            capture_output=True,
            text=True,
        )
        active_pattern = rf"(?:Recording|Tracing) session {re.escape(trace_session)}: \[active\]"
        active = result.returncode == 0 and re.search(active_pattern, result.stdout) is not None
        with open(trace_monitor_path, "a", encoding="utf-8") as stream:
            stream.write(
                f"monotonic={now:.6f} returncode={result.returncode} active={str(active).lower()}\n"
            )
        if not active:
            print("ros_trace_session")
            raise SystemExit(1)
        next_trace_check = now + 1.0
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        break
    time.sleep(min(0.25, remaining))

with open(role_monitor_path, "a", encoding="utf-8") as stream:
    stream.write(f"completed_monotonic={time.monotonic():.9f}\n")
PY
}

stop_perf()
{
    local final_tids
    final_tids="$(find "/proc/${TRACEE_PID}/task" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' \
        | sort -n | paste -sd, -)"
    if [[ "${final_tids}" != "${TRACEE_TIDS}" ]]; then
        invalidate "target thread set changed during perf measurement"
    fi
    perf_control disable || true
    perf_control stop || true
    if ! wait_for_dead "${TOOL_PID}" 20; then
        FORCED_STOP=true
        invalidate "perf did not finalize after controlled stop"
        signal_process TERM "${TOOL_PID}" "${TOOL_PGID}" "${TOOL_STARTTIME}" || true
        return 1
    fi
    local rc=0
    if wait_child "${TOOL_PID}" perf; then
        rc=0
    else
        rc=$?
    fi
    TOOL_REAPED=true
    echo "tool_exit_code=${rc}" >> "${OUTPUT_DIR}/exit-codes.txt"
    if [[ "${rc}" -ne 0 ]]; then
        ROLE_EXIT_FAILURE=true
        invalidate "perf returned unexpected exit code ${rc}"
    fi
    exec 8>&- 9>&-
    PERF_CTL_FD_OPEN=false
}

stop_trace()
{
    local rc=0
    if taskset -c 1 ros2 trace stop "${TRACE_SESSION}" \
        > "${OUTPUT_DIR}/trace-stop.log" 2>&1; then
        echo "ros_trace_stop_exit_code=0" >> "${OUTPUT_DIR}/exit-codes.txt"
    else
        rc=$?
        ROLE_EXIT_FAILURE=true
        echo "ros_trace_stop_exit_code=${rc}" >> "${OUTPUT_DIR}/exit-codes.txt"
        invalidate "ros2 trace stop failed"
        return 1
    fi
    TRACE_STARTED=false
}

validate_perf_window_control()
{
    python3 - "${OUTPUT_DIR}/perf-control.txt" "${T0_MONOTONIC_NS}" \
        "${T1_MONOTONIC_NS}" "${OUTPUT_DIR}/perf-window-quality.txt" <<'PY'
import sys

control_path, t0, t1, quality_path = sys.argv[1:]
values = {}
with open(control_path, encoding="utf-8") as stream:
    for line in stream:
        if "=" in line:
            key, value = line.strip().split("=", 1)
            if key in values:
                raise SystemExit(f"duplicate perf control evidence: {key}")
            values[key] = value

for command in ("enable", "disable", "stop"):
    if values.get(f"{command}_ack") != "ack":
        raise SystemExit(f"missing perf {command} acknowledgement")

enable = int(values["enable_ack_monotonic_ns"])
disable = int(values["disable_ack_monotonic_ns"])
stop = int(values["stop_ack_monotonic_ns"])
t0 = int(t0)
t1 = int(t1)
if not enable <= t0 < t1 <= disable <= stop:
    raise SystemExit("perf control acknowledgements do not enclose the workload window")

with open(quality_path, "w", encoding="utf-8") as stream:
    stream.write("parse_valid=true\n")
    stream.write(f"enable_ack_monotonic_ns={enable}\n")
    stream.write(f"t0_monotonic_ns={t0}\n")
    stream.write(f"t1_monotonic_ns={t1}\n")
    stream.write(f"disable_ack_monotonic_ns={disable}\n")
    stream.write(f"stop_ack_monotonic_ns={stop}\n")
    stream.write("gate_pass=true\n")
PY
}

parse_workload()
{
    local start_line="$1"
    local end_line="$2"
    local t0_ns="$3"
    local t1_ns="$4"
    python3 - "${OUTPUT_DIR}/measurement.csv" "${start_line}" "${end_line}" \
        "${t0_ns}" "${t1_ns}" "${OUTPUT_DIR}/workload-counts.txt" <<'PY'
import csv
import math
import sys

path, start_line, end_line, t0_ns, t1_ns, output = sys.argv[1:]
start_line = int(start_line)
end_line = int(end_line)
duration = (int(t1_ns) - int(t0_ns)) / 1_000_000_000
counts = {"front": 0, "rear": 0, "top": 0}
unknown = []
with open(path, newline="", encoding="utf-8") as stream:
    rows = list(csv.reader(stream))
for row in rows[start_line:end_line]:
    if not row:
        continue
    value = row[-1].strip().strip('"')
    if value in counts:
        counts[value] += 1
    elif value and value not in {"sensor_id", "---"}:
        unknown.append(value)
total = sum(counts.values())
required_total = math.ceil(duration * 27.0)
required_each = math.ceil(duration * 9.0)
with open(output, "w", encoding="utf-8") as stream:
    stream.write(f"duration_actual_s={duration:.9f}\n")
    stream.write(f"line_start={start_line}\nline_end={end_line}\n")
    stream.write(f"front={counts['front']}\nrear={counts['rear']}\ntop={counts['top']}\n")
    stream.write(f"total={total}\nrequired_total={required_total}\nrequired_each={required_each}\n")
    stream.write(f"unknown={len(unknown)}\n")
if duration <= 0 or total < required_total or any(value < required_each for value in counts.values()) or unknown:
    raise SystemExit(1)
PY
}

generate_reports()
{
    case "${MODE}" in
        perf-stat)
            if ! python3 - "${OUTPUT_DIR}/perf-stat.csv" \
                "${OUTPUT_DIR}/perf-stat-quality.txt" <<'PY'
import csv
import math
import re
import sys

stat_path, quality_path = sys.argv[1:]
expected_events = (
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
required_software_events = set(expected_events[:4])
unavailable_values = {"<not supported>", "<not counted>"}
rows = {}

with open(stat_path, newline="", encoding="utf-8", errors="replace") as stream:
    for row in csv.reader(stream):
        if len(row) < 5:
            continue
        event = row[2].strip()
        if event in expected_events:
            if event in rows:
                raise SystemExit(f"duplicate perf stat event: {event}")
            rows[event] = tuple(field.strip() for field in row)

missing = [event for event in expected_events if event not in rows]
if missing:
    raise SystemExit("missing perf stat events: " + ", ".join(missing))

quality = {}
for event in expected_events:
    row = rows[event]
    value_text = row[0]
    key = re.sub(r"[^a-z0-9]+", "_", event.lower()).strip("_")
    if value_text in unavailable_values:
        if event in required_software_events:
            raise SystemExit(f"required software event is unavailable: {event}")
        quality[f"{key}_status"] = value_text.strip("<>").replace(" ", "_")
        continue
    try:
        value = float(value_text)
        runtime_ns = float(row[3])
        running_percent = float(row[4])
    except ValueError as error:
        raise SystemExit(f"unparseable perf stat row for {event}: {row}") from error
    if not all(math.isfinite(item) for item in (value, runtime_ns, running_percent)):
        raise SystemExit(f"non-finite perf stat row for {event}: {row}")
    if value < 0 or runtime_ns <= 0 or not 0 < running_percent <= 100.0:
        raise SystemExit(f"invalid perf stat counters for {event}: {row}")
    if event == "task-clock" and value <= 0:
        raise SystemExit("task-clock must be positive")
    quality[f"{key}_status"] = "supported"
    quality[f"{key}_value"] = value_text
    quality[f"{key}_runtime_ns"] = row[3]
    quality[f"{key}_running_percent"] = row[4]

with open(quality_path, "w", encoding="utf-8") as stream:
    stream.write("parse_valid=true\n")
    for key, value in quality.items():
        stream.write(f"{key}={value}\n")
    stream.write("gate_pass=true\n")
PY
            then
                invalidate "perf stat report is incomplete or unparseable"
            else
                sed 's/^/perf_stat_/' "${OUTPUT_DIR}/perf-stat-quality.txt" \
                    >> "${OUTPUT_DIR}/run-manifest.txt"
            fi
            ;;
        perf-record)
            "${PERF_PATH}" report -i "${OUTPUT_DIR}/perf.data" --stdio --header \
                --no-children --sort comm,dso,symbol --percent-limit 0 \
                > "${OUTPUT_DIR}/perf-report.txt" 2> "${OUTPUT_DIR}/perf-report.stderr.log" || invalidate "perf report failed"
            "${PERF_PATH}" report -i "${OUTPUT_DIR}/perf.data" --stdio --header \
                --children --sort comm,dso,symbol --percent-limit 0 \
                > "${OUTPUT_DIR}/perf-callgraph-report.txt" 2> "${OUTPUT_DIR}/perf-callgraph-report.stderr.log" || invalidate "perf callgraph report failed"
            "${PERF_PATH}" buildid-list -i "${OUTPUT_DIR}/perf.data" \
                > "${OUTPUT_DIR}/perf-buildid-list.txt" 2>&1 || invalidate "perf buildid-list failed"
            if ! awk -v build_id="${TARGET_BUILD_ID}" \
                '$1 == build_id {found = 1} END {exit !found}' \
                "${OUTPUT_DIR}/perf-buildid-list.txt"; then
                invalidate "perf build-id list does not contain the target build ID"
            fi
            "${PERF_PATH}" report -i "${OUTPUT_DIR}/perf.data" --stdio \
                --no-children --call-graph none --sort overhead,comm,dso,symbol --percent-limit 0 \
                -t '|' -F overhead,sample,comm,dso,symbol \
                > "${OUTPUT_DIR}/perf-symbols.txt" 2> "${OUTPUT_DIR}/perf-symbols.stderr.log" \
                || invalidate "perf symbol summary failed"
            if ! python3 - "${OUTPUT_DIR}/perf-symbols.txt" \
                "${OUTPUT_DIR}/perf-report.txt" \
                "${OUTPUT_DIR}/perf-quality.txt" "${OUTPUT_DIR}/perf-top10.txt" "${DURATION}" <<'PY'
import re
import sys

symbols_path, report_path, quality_path, top_path, requested_duration = sys.argv[1:]
requested_duration = int(requested_duration)
with open(symbols_path, encoding="utf-8", errors="replace") as stream:
    lines = stream.readlines()
with open(report_path, encoding="utf-8", errors="replace") as stream:
    report_text = stream.read()

sample_match = next(
    (re.search(r"^# Samples:\s+(\S+)", line) for line in lines if line.startswith("# Samples:")),
    None,
)
header_samples = sample_match.group(1) if sample_match else "missing"
lost_match = re.search(r"^# Total Lost Samples:\s+(\d+)\s*$", report_text, re.MULTILINE)
lost_samples = int(lost_match.group(1)) if lost_match else -1
rows = []
for line in lines:
    parts = line.rstrip("\n").split("|")
    if len(parts) < 5 or not re.match(r"^\s*\d+(?:\.\d+)?%\s*$", parts[0]):
        continue
    try:
        row_samples = int(parts[1].strip())
    except ValueError:
        continue
    dso = parts[3].strip()
    symbol = parts[4].strip()
    normalized_symbol = re.sub(r"^\[[^]]+\]\s+", "", symbol).strip()
    is_unknown = (
        dso in {"[unknown]", "unknown"}
        or normalized_symbol in {"[unknown]", "unknown"}
        or normalized_symbol.startswith("0x")
        or normalized_symbol.lower().startswith("ffffffff")
    )
    is_workspace = (
        "PerceptionInputNode::" in normalized_symbol
        or "Perception::" in normalized_symbol
        or "PerceptionAdapters::" in normalized_symbol
        or "PerceptionCore::" in normalized_symbol
    )
    rows.append((line.rstrip(), row_samples, is_unknown, is_workspace))

parsed_samples = sum(row[1] for row in rows)
sample_count = parsed_samples
unknown_samples = sum(row[1] for row in rows if row[2])
workspace_samples = sum(row[1] for row in rows if row[3])
unknown_percent = 100.0 * unknown_samples / sample_count if sample_count else 100.0
gate_enforced = requested_duration >= 120
parse_valid = sample_match is not None and sample_count > 0 and lost_match is not None
gate_pass = (
    parse_valid
    and lost_samples == 0
    and (not gate_enforced or (
        sample_count >= 1000
        and unknown_percent <= 20.0
        and workspace_samples > 0
    ))
)

with open(quality_path, "w", encoding="utf-8") as stream:
    stream.write(f"requested_duration_s={requested_duration}\n")
    stream.write(f"gate_enforced={str(gate_enforced).lower()}\n")
    stream.write(f"header_samples={header_samples}\n")
    stream.write(f"samples={sample_count}\n")
    stream.write(f"parsed_samples={parsed_samples}\n")
    stream.write(f"unknown_samples={unknown_samples}\n")
    stream.write(f"unknown_percent={unknown_percent:.6f}\n")
    stream.write(f"workspace_samples={workspace_samples}\n")
    stream.write(f"lost_samples={lost_samples}\n")
    stream.write(f"parse_valid={str(parse_valid).lower()}\n")
    stream.write(f"gate_pass={str(gate_pass).lower()}\n")
with open(top_path, "w", encoding="utf-8") as stream:
    for row, *_ in rows[:10]:
        stream.write(row + "\n")

raise SystemExit(0 if gate_pass else 1)
PY
            then
                invalidate "perf sample count, symbolization, or unknown-ratio gate failed"
            fi
            ;;
        heaptrack)
            local heap_files=()
            mapfile -t heap_files < <(find "${OUTPUT_DIR}" -maxdepth 1 -type f -name 'heaptrack*.gz' | sort)
            if (( ${#heap_files[@]} != 1 )); then
                invalidate "Heaptrack must produce exactly one output"
            else
                local heap_file="${heap_files[0]}"
                heaptrack_print "${heap_file}" > "${OUTPUT_DIR}/heaptrack-report.txt" 2> "${OUTPUT_DIR}/heaptrack-report.stderr.log" \
                    || invalidate "heaptrack_print failed"
                if ! python3 - "${OUTPUT_DIR}/heaptrack-report.txt" \
                    "${OUTPUT_DIR}/heaptrack-quality.txt" "${DURATION}" <<'PY'
import math
import re
import sys

report_path, quality_path, requested_duration = sys.argv[1:]
requested_duration = int(requested_duration)
with open(report_path, encoding="utf-8", errors="replace") as stream:
    text = stream.read()

required_sections = (
    "MOST CALLS TO ALLOCATION FUNCTIONS",
    "PEAK MEMORY CONSUMERS",
    "MOST TEMPORARY ALLOCATIONS",
)
missing_sections = [section for section in required_sections if section not in text]
patterns = {
    "total_runtime_s": r"^total runtime:\s+([0-9.]+)s\.$",
    "allocation_calls": r"^calls to allocation functions:\s+(\d+)\s+\(",
    "temporary_allocations": r"^temporary memory allocations:\s+(\d+)\s+\(",
    "peak_heap": r"^peak heap memory consumption:\s+(\S+)\s*$",
    "peak_rss_with_overhead": r"^peak RSS \(including heaptrack overhead\):\s+(\S+)\s*$",
    "total_memory_leaked": r"^total memory leaked:\s+(\S+)\s*$",
}
values = {}
for key, pattern in patterns.items():
    match = re.search(pattern, text, re.MULTILINE)
    if match is None:
        raise SystemExit(f"missing Heaptrack summary: {key}")
    values[key] = match.group(1)
if missing_sections:
    raise SystemExit("missing Heaptrack sections: " + ", ".join(missing_sections))

runtime = float(values["total_runtime_s"])
if not math.isfinite(runtime) or runtime < requested_duration:
    raise SystemExit("Heaptrack runtime is shorter than the formal window")

def parse_bytes(value):
    match = re.fullmatch(r"([0-9]+(?:\.[0-9]+)?)([KMGTPE]?)(?:i?B)?", value)
    if match is None:
        raise SystemExit(f"unparseable Heaptrack byte quantity: {value}")
    scale = 1024 ** " KMGTPE".index(match.group(2))
    result = float(match.group(1)) * scale
    if not math.isfinite(result) or result < 0:
        raise SystemExit(f"invalid Heaptrack byte quantity: {value}")
    return round(result)

peak_heap_bytes = parse_bytes(values["peak_heap"])
peak_rss_bytes = parse_bytes(values["peak_rss_with_overhead"])
leaked_bytes = parse_bytes(values["total_memory_leaked"])
if int(values["allocation_calls"]) <= 0 or peak_heap_bytes <= 0 or peak_rss_bytes <= 0:
    raise SystemExit("Heaptrack allocation and peak summaries must be positive")

with open(quality_path, "w", encoding="utf-8") as stream:
    stream.write("parse_valid=true\n")
    for key, value in values.items():
        stream.write(f"{key}={value}\n")
    stream.write(f"peak_heap_bytes={peak_heap_bytes}\n")
    stream.write(f"peak_rss_with_overhead_bytes={peak_rss_bytes}\n")
    stream.write(f"total_memory_leaked_bytes={leaked_bytes}\n")
    stream.write("gate_pass=true\n")
PY
                then
                    invalidate "Heaptrack report is incomplete or unparseable"
                else
                    sed 's/^/heaptrack_/' "${OUTPUT_DIR}/heaptrack-quality.txt" \
                        >> "${OUTPUT_DIR}/run-manifest.txt"
                fi
            fi
            ;;
        valgrind-massif)
            local massif_files=()
            mapfile -t massif_files < <(find "${OUTPUT_DIR}" -maxdepth 1 -type f -name 'massif.*.out' | sort)
            if (( ${#massif_files[@]} != 1 )); then
                invalidate "Massif must produce exactly one output"
            else
                local massif_file="${massif_files[0]}"
                ms_print "${massif_file}" > "${OUTPUT_DIR}/massif-report.txt" 2> "${OUTPUT_DIR}/massif-report.stderr.log" \
                    || invalidate "ms_print failed"
                if ! python3 - "${massif_file}" "${OUTPUT_DIR}/massif-quality.txt" \
                    "${EXPECTED_TARGET_EXE}" "${DURATION}" <<'PY'
import sys

massif_path, quality_path, expected_target, requested_duration = sys.argv[1:]
requested_duration = int(requested_duration)
snapshots = []
current = None
header = {}
peak_tree_count = 0
with open(massif_path, encoding="utf-8", errors="replace") as stream:
    for raw_line in stream:
        line = raw_line.rstrip("\n")
        if line.startswith("cmd:"):
            header["cmd"] = line.split(":", 1)[1].strip()
        elif line.startswith("time_unit:"):
            header["time_unit"] = line.split(":", 1)[1].strip()
        elif line == "heap_tree=peak":
            peak_tree_count += 1
        if line.startswith("snapshot="):
            if current is not None:
                snapshots.append(current)
            current = {"snapshot": int(line.split("=", 1)[1])}
        elif current is not None and "=" in line:
            key, value = line.split("=", 1)
            if key in {"time", "mem_heap_B", "mem_heap_extra_B", "mem_stacks_B"}:
                current[key] = int(value)
if current is not None:
    snapshots.append(current)

required_keys = {"snapshot", "time", "mem_heap_B", "mem_heap_extra_B", "mem_stacks_B"}
if len(snapshots) < 20:
    raise SystemExit("Massif produced fewer than 20 snapshots")
if any(required_keys - snapshot.keys() for snapshot in snapshots):
    raise SystemExit("Massif snapshot is incomplete")
if expected_target not in header.get("cmd", ""):
    raise SystemExit("Massif command does not identify the target ELF")
if header.get("time_unit") != "ms":
    raise SystemExit("Massif time unit is not milliseconds")
if peak_tree_count == 0 or max(snapshot["mem_stacks_B"] for snapshot in snapshots) <= 0:
    raise SystemExit("Massif output does not contain stack-aware peak detail")

def total_bytes(snapshot):
    return snapshot["mem_heap_B"] + snapshot["mem_heap_extra_B"] + snapshot["mem_stacks_B"]

peak = max(snapshots, key=total_bytes)
final = snapshots[-1]
if final["time"] < requested_duration * 1000:
    raise SystemExit("Massif timeline is shorter than the formal window")
with open(quality_path, "w", encoding="utf-8") as stream:
    stream.write("parse_valid=true\n")
    stream.write(f"snapshot_count={len(snapshots)}\n")
    stream.write(f"peak_snapshot={peak['snapshot']}\n")
    stream.write(f"peak_time_ms={peak['time']}\n")
    stream.write(f"peak_total_bytes={total_bytes(peak)}\n")
    stream.write(f"final_snapshot={final['snapshot']}\n")
    stream.write(f"final_time_ms={final['time']}\n")
    stream.write(f"final_total_bytes={total_bytes(final)}\n")
    stream.write(f"peak_tree_count={peak_tree_count}\n")
    stream.write("target_verified=true\n")
    stream.write("gate_pass=true\n")
PY
                then
                    invalidate "Massif output is incomplete or unparseable"
                else
                    sed 's/^/massif_/' "${OUTPUT_DIR}/massif-quality.txt" \
                        >> "${OUTPUT_DIR}/run-manifest.txt"
                fi
            fi
            ;;
        valgrind-memcheck)
            local memcheck_files=()
            mapfile -t memcheck_files < <(find "${OUTPUT_DIR}" -maxdepth 1 -type f -name 'memcheck.*.log' | sort)
            if (( ${#memcheck_files[@]} != 1 )); then
                invalidate "Memcheck must produce exactly one log"
            elif ! python3 - "${memcheck_files[0]}" \
                "${OUTPUT_DIR}/memcheck-summary.txt" "${OUTPUT_DIR}/memcheck-quality.txt" \
                "${EXPECTED_TARGET_EXE}" <<'PY'
import re
import sys

log_path, summary_path, quality_path, expected_target = sys.argv[1:]
with open(log_path, encoding="utf-8", errors="replace") as stream:
    text = stream.read()

command_match = re.search(r"^==\d+== Command:\s+(.+)$", text, re.MULTILINE)
if command_match is None or expected_target not in command_match.group(1):
    raise SystemExit("Memcheck report does not identify the target ELF")

required_patterns = (
    r"HEAP SUMMARY:",
    r"in use at exit:",
    r"total heap usage:",
    r"LEAK SUMMARY:",
    r"definitely lost:\s+[\d,]+ bytes",
    r"indirectly lost:\s+[\d,]+ bytes",
    r"possibly lost:\s+[\d,]+ bytes",
    r"still reachable:\s+[\d,]+ bytes",
    r"ERROR SUMMARY:\s+\d+ errors? from \d+ contexts?",
)
missing = [pattern for pattern in required_patterns if re.search(pattern, text) is None]
if missing:
    print("missing Memcheck sections: " + ", ".join(missing), file=sys.stderr)
    raise SystemExit(1)

def bytes_for(label):
    match = re.search(rf"{label}:\s+([\d,]+) bytes", text)
    return int(match.group(1).replace(",", ""))

def matching_line(fragment):
    return next(line for line in text.splitlines() if fragment in line)

definite_bytes = bytes_for("definitely lost")
indirect_bytes = bytes_for("indirectly lost")
possible_bytes = bytes_for("possibly lost")
reachable_bytes = bytes_for("still reachable")
error_count = int(re.search(r"ERROR SUMMARY:\s+(\d+) errors?", text).group(1))
invalid_access = re.search(
    r"Invalid (?:read|write|free|delete)|Mismatched free|Source and destination overlap",
    text,
) is not None
other_error = error_count > 0 and not invalid_access and definite_bytes == 0 and indirect_bytes == 0
finding = error_count > 0 or definite_bytes > 0 or indirect_bytes > 0

summary_fragments = (
    "HEAP SUMMARY:",
    "in use at exit:",
    "total heap usage:",
    "LEAK SUMMARY:",
    "definitely lost:",
    "indirectly lost:",
    "possibly lost:",
    "still reachable:",
    "ERROR SUMMARY:",
)
with open(summary_path, "w", encoding="utf-8") as stream:
    for fragment in summary_fragments:
        stream.write(matching_line(fragment) + "\n")
with open(quality_path, "w", encoding="utf-8") as stream:
    stream.write(f"definite_lost_bytes={definite_bytes}\n")
    stream.write(f"indirect_lost_bytes={indirect_bytes}\n")
    stream.write(f"possibly_lost_bytes={possible_bytes}\n")
    stream.write(f"still_reachable_bytes={reachable_bytes}\n")
    stream.write(f"error_count={error_count}\n")
    stream.write(f"invalid_access={str(invalid_access).lower()}\n")
    stream.write(f"other_error={str(other_error).lower()}\n")
    stream.write(f"finding={str(finding).lower()}\n")
    stream.write("target_verified=true\n")
PY
            then
                invalidate "Memcheck report is incomplete or unparseable"
            else
                sed 's/^/memcheck_/' "${OUTPUT_DIR}/memcheck-quality.txt" \
                    >> "${OUTPUT_DIR}/run-manifest.txt"
                local memcheck_finding
                memcheck_finding="$(awk -F= '$1 == "finding" {print $2}' \
                    "${OUTPUT_DIR}/memcheck-quality.txt")"
                if [[ "${MEMCHECK_EXIT_CODE}" == 42 && "${memcheck_finding}" == true ]]; then
                    cp "${OUTPUT_DIR}/memcheck-quality.txt" "${OUTPUT_DIR}/memcheck-finding.txt"
                elif [[ "${MEMCHECK_EXIT_CODE}" == 0 && "${memcheck_finding}" == false ]]; then
                    :
                else
                    invalidate "Memcheck exit code and parsed finding classification disagree"
                fi
            fi
            ;;
        ros-trace)
            local trace_root
            trace_root="$(find "${OUTPUT_DIR}/trace" -mindepth 1 -maxdepth 2 -type d -name ust -printf '%h\n' | head -1)"
            if [[ -z "${trace_root}" ]]; then
                invalidate "LTTng trace data is missing"
            else
                babeltrace2 convert --clock-seconds --begin="${T0_REALTIME}" --end="${T1_REALTIME}" \
                    "${trace_root}" -w "${OUTPUT_DIR}/trace-window.txt" \
                    || invalidate "Babeltrace conversion failed"
                local callback_count take_count publish_count
                callback_count="$(grep -E "ros2:callback_(start|end).*vpid = ${TRACEE_PID}([, }]|$)" "${OUTPUT_DIR}/trace-window.txt" | wc -l)"
                take_count="$(grep -E "ros2:(rcl|rmw)_take.*vpid = ${TRACEE_PID}([, }]|$)" "${OUTPUT_DIR}/trace-window.txt" | wc -l)"
                publish_count="$(grep -E "ros2:(rclcpp|rcl)_publish.*vpid = ${TRACEE_PID}([, }]|$)" "${OUTPUT_DIR}/trace-window.txt" | wc -l)"
                {
                    echo "trace_callback=${callback_count}"
                    echo "trace_take=${take_count}"
                    echo "trace_publish=${publish_count}"
                } > "${OUTPUT_DIR}/trace-counts.txt"
                (( callback_count > 0 && take_count > 0 && publish_count > 0 )) \
                    || invalidate "target vpid is missing callback/take/publish trace evidence"
            fi
            ;;
    esac
}

start_fixture || exit 1
start_sink preflight "${OUTPUT_DIR}/preflight.csv" || exit 1
start_target || exit 1

if ! wait_for_nodes; then
    invalidate "fixture and input nodes did not become ready"
    exit 1
fi
if ! validate_parameters; then
    invalidate "runtime parameter dump does not match the fixed workload"
    exit 1
fi
if ! wait_for_graph_pair preflight "${PREFLIGHT_PID}"; then
    invalidate "preflight ROS graph is incomplete"
    exit 1
fi
if ! wait_for_sensor_ids "${OUTPUT_DIR}/preflight.csv" "${PREFLIGHT_PID}"; then
    invalidate "preflight sink did not receive front/rear/top"
    exit 1
fi
stop_group "${PREFLIGHT_PID}" "${PREFLIGHT_PGID}" preflight_sink "${PREFLIGHT_STARTTIME}"
PREFLIGHT_PID=""
PREFLIGHT_PGID=""
PREFLIGHT_STARTTIME=""

start_sink measurement "${OUTPUT_DIR}/measurement.csv" || exit 1
if ! wait_for_graph_pair measurement "${SINK_PID}"; then
    invalidate "measurement sink did not pair with the output publisher"
    exit 1
fi
if ! wait_for_sensor_ids "${OUTPUT_DIR}/measurement.csv" "${SINK_PID}"; then
    invalidate "measurement sink did not receive front/rear/top before profiling"
    exit 1
fi

case "${MODE}" in
    perf-stat|perf-record)
        start_perf || exit 1
        ;;
    ros-trace)
        start_trace || exit 1
        ;;
    plain-sample)
        start_plain_samplers || exit 1
        ;;
esac

T0_MONOTONIC_NS="$(monotonic_ns)"
T0_REALTIME="$(realtime_stamp)"
START_LINE="$(wc -l < "${OUTPUT_DIR}/measurement.csv")"
window_failure=""
if ! window_failure="$(monitor_measurement_window)"; then
    invalidate "${window_failure:-required process} exited before the measurement window ended"
    exit 1
fi
END_LINE="$(wc -l < "${OUTPUT_DIR}/measurement.csv")"
T1_REALTIME="$(realtime_stamp)"
T1_MONOTONIC_NS="$(monotonic_ns)"

{
    echo "t0_monotonic_ns=${T0_MONOTONIC_NS}"
    echo "t1_monotonic_ns=${T1_MONOTONIC_NS}"
    echo "t0_realtime=${T0_REALTIME}"
    echo "t1_realtime=${T1_REALTIME}"
} >> "${OUTPUT_DIR}/run-manifest.txt"

case "${MODE}" in
    perf-stat|perf-record)
        stop_perf
        if ! validate_perf_window_control; then
            ROLE_EXIT_FAILURE=true
            invalidate "perf control ACKs do not enclose the measurement window"
        else
            sed 's/^/perf_window_/' "${OUTPUT_DIR}/perf-window-quality.txt" \
                >> "${OUTPUT_DIR}/run-manifest.txt"
        fi
        ;;
    ros-trace)
        stop_trace
        ;;
    plain-sample)
        stop_group "${PIDSTAT_PID}" "${PIDSTAT_PGID}" pidstat "${PIDSTAT_STARTTIME}"
        PIDSTAT_PID=""
        PIDSTAT_PGID=""
        PIDSTAT_STARTTIME=""
        stop_group "${SMEM_PID}" "${SMEM_PGID}" smem_sampler "${SMEM_STARTTIME}" TERM
        SMEM_PID=""
        SMEM_PGID=""
        SMEM_STARTTIME=""
        ;;
esac

if ! parse_workload "${START_LINE}" "${END_LINE}" "${T0_MONOTONIC_NS}" "${T1_MONOTONIC_NS}"; then
    invalidate "measurement-window workload counts are below the validity gate"
fi
if [[ -f "${OUTPUT_DIR}/workload-counts.txt" ]]; then
    sed 's/^/workload_/' "${OUTPUT_DIR}/workload-counts.txt" \
        >> "${OUTPUT_DIR}/run-manifest.txt"
fi

stop_tracee
if [[ -n "${LAUNCHER_PID}" ]]; then
    if ! wait_for_dead "${LAUNCHER_PID}" 30; then
        FORCED_STOP=true
        invalidate "launcher/tool did not finalize after tracee exit"
        signal_process TERM "${LAUNCHER_PID}" "${LAUNCHER_PGID}" "${LAUNCHER_STARTTIME}" || true
        sleep 1
        if process_is_alive "${LAUNCHER_PID}"; then
            signal_process KILL "${LAUNCHER_PID}" "${LAUNCHER_PGID}" "${LAUNCHER_STARTTIME}" || true
        fi
    fi
    local_launcher_rc=0
    if wait_child "${LAUNCHER_PID}" launcher; then
        local_launcher_rc=0
    else
        local_launcher_rc=$?
    fi
    LAUNCHER_REAPED=true
    echo "tracee_exit_code=${local_launcher_rc}" >> "${OUTPUT_DIR}/exit-codes.txt"
    if [[ -n "${TOOL_PID}" && "${TOOL_PID}" == "${LAUNCHER_PID}" ]]; then
        TOOL_REAPED=true
        echo "tool_exit_code=${local_launcher_rc}" >> "${OUTPUT_DIR}/exit-codes.txt"
    fi
    if [[ "${MODE}" == heaptrack ]]; then
        echo "tracee_exit_code_source=heaptrack_controller" >> "${OUTPUT_DIR}/exit-codes.txt"
    else
        echo "tracee_exit_code_source=direct_process" >> "${OUTPUT_DIR}/exit-codes.txt"
    fi
    case "${MODE}" in
        valgrind-memcheck)
            MEMCHECK_EXIT_CODE="${local_launcher_rc}"
            if [[ "${local_launcher_rc}" -ne 0 && "${local_launcher_rc}" -ne 42 ]]; then
                ROLE_EXIT_FAILURE=true
                invalidate "Memcheck tool exited abnormally (${local_launcher_rc})"
            fi
            ;;
        asan-smoke|lsan-smoke|heaptrack|valgrind-massif)
            if [[ "${local_launcher_rc}" -ne 0 ]]; then
                ROLE_EXIT_FAILURE=true
                invalidate "instrumented target exited abnormally (${local_launcher_rc})"
            fi
            ;;
        *)
            if [[ "${local_launcher_rc}" -ne 0 ]]; then
                ROLE_EXIT_FAILURE=true
                invalidate "target exited abnormally (${local_launcher_rc})"
            fi
            ;;
    esac
fi

stop_group "${SINK_PID}" "${SINK_PGID}" measurement_sink "${SINK_STARTTIME}"
SINK_PID=""
SINK_PGID=""
SINK_STARTTIME=""
stop_group "${FIXTURE_PID}" "${FIXTURE_PGID}" fixture "${FIXTURE_STARTTIME}"
FIXTURE_PID=""
FIXTURE_PGID=""
FIXTURE_STARTTIME=""

generate_reports

if [[ "${MODE}" == asan-smoke || "${MODE}" == lsan-smoke ]]; then
    if grep -R -E 'ERROR: (AddressSanitizer|LeakSanitizer)|SUMMARY: (AddressSanitizer|LeakSanitizer)' \
        "${OUTPUT_DIR}"/asan* "${OUTPUT_DIR}"/lsan* "${OUTPUT_DIR}/target.log" 2>/dev/null; then
        invalidate "sanitizer error was reported"
    fi
fi

if [[ "${FORCED_STOP}" == false && "${ROLE_EXIT_FAILURE}" == false ]]; then
    NORMAL_COMPLETION=true
fi
if [[ "${VALID}" != true ]]; then
    exit 1
fi
exit 0
