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
PROFILE_RUNNER_COMMON="${SCRIPT_DIR}/lib/profile-runner-common.sh"
PROFILE_REPORT_PARSER="${SCRIPT_DIR}/lib/profile_report_parsers.py"
PROFILE_ROLE_MONITOR="${SCRIPT_DIR}/lib/profile_role_monitor.py"
PROFILE_ANALYSIS_COMMON="${SCRIPT_DIR}/lib/profile_analysis.py"
for common_file in "${PROFILE_RUNNER_COMMON}" "${PROFILE_REPORT_PARSER}" \
    "${PROFILE_ROLE_MONITOR}" "${PROFILE_ANALYSIS_COMMON}"; do
    if [[ ! -r "${common_file}" ]]; then
        echo "missing profiling common module: ${common_file}" >&2
        exit 2
    fi
done
source "${PROFILE_RUNNER_COMMON}"
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
TARGET_IDENTITY_RECORDED=false
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
    return 0
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
        if [[ "${TARGET_IDENTITY_RECORDED}" != true \
            && -n "${LAUNCHER_PID}" && "${LAUNCHER_REAPED}" != true ]]; then
            stop_partial_target_startup || true
        fi
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

PERF_PATH="$(find_perf || true)"

{
    echo "mode=${MODE}"
    echo "duration_requested_s=${DURATION}"
    echo "started_at=$(date --iso-8601=ns)"
    echo "source_revision=$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || echo unavailable)"
    echo "source_dirty_count=$(git -C "${REPO_ROOT}" status --short 2>/dev/null | wc -l)"
    echo "profile_script_sha256=$(sha256sum "${BASH_SOURCE[0]}" | awk '{print $1}')"
    echo "profile_runner_common_sha256=$(sha256sum "${PROFILE_RUNNER_COMMON}" | awk '{print $1}')"
    echo "profile_report_parsers_sha256=$(sha256sum "${PROFILE_REPORT_PARSER}" | awk '{print $1}')"
    echo "profile_role_monitor_sha256=$(sha256sum "${PROFILE_ROLE_MONITOR}" | awk '{print $1}')"
    echo "profile_analysis_common_sha256=$(sha256sum "${PROFILE_ANALYSIS_COMMON}" | awk '{print $1}')"
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
        process_identity_matches \
            "${root_pid}" "${LAUNCHER_STARTTIME}" "${LAUNCHER_PGID}" || return 3
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
    if [[ "${MODE}" == heaptrack && "${TRACEE_PGID}" != "${LAUNCHER_PGID}" ]]; then
        invalidate "Heaptrack target escaped the verified instrumentation process group"
        return 1
    fi
    if [[ -n "${TOOL_PID}" ]]; then
        TOOL_PGID="${LAUNCHER_PGID}"
        TOOL_STARTTIME="${LAUNCHER_STARTTIME}"
    fi
    tr '\0' ' ' < "/proc/${TRACEE_PID}/cmdline" > "${OUTPUT_DIR}/tracee-cmdline.txt"
    if [[ "${MODE}" == heaptrack ]]; then
        tr '\0' ' ' < "/proc/${LAUNCHER_PID}/cmdline" > "${OUTPUT_DIR}/launcher-cmdline.txt"
    fi

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
        if [[ "${MODE}" == heaptrack ]]; then
            echo "heaptrack_process_model=${heaptrack_process_model}"
            echo "launcher_exe=$(readlink -f "/proc/${LAUNCHER_PID}/exe")"
        fi
        echo "tracee_affinity=$(taskset -pc "${TRACEE_PID}" 2>&1 | tail -1)"
        echo "tracee_cgroup=$(tr '\n' ';' < "/proc/${TRACEE_PID}/cgroup")"
        awk '/^(Name|Pid|PPid|NSpid|Cpus_allowed_list|CapEff|NoNewPrivs|Seccomp|Seccomp_filters):/ {
            key = tolower($1)
            sub(/:$/, "", key)
            print "tracee_" key "=" $2
        }' "/proc/${TRACEE_PID}/status"
    } >> "${OUTPUT_DIR}/run-manifest.txt"
    TARGET_IDENTITY_RECORDED=true
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
    if list_ros_trace_events > "${OUTPUT_DIR}/trace-event-list.txt" 2>&1; then
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

    taskset -c 1 python3 "${PROFILE_ROLE_MONITOR}" "${DURATION}" "${trace_session}" \
        "${OUTPUT_DIR}/trace-session-monitor.txt" \
        "${OUTPUT_DIR}/role-monitor.txt" "${roles[@]}"
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
    local rc=0
    if wait_child "${TOOL_PID}" perf; then
        rc=0
    else
        rc=$?
    fi
    TOOL_REAPED=true
    if (( rc != 0 && rc != 130 )); then
        ROLE_EXIT_FAILURE=true
        invalidate "perf returned unexpected exit code ${rc}"
    fi
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
    python3 "${PROFILE_REPORT_PARSER}" perf-control \
        "${OUTPUT_DIR}/perf-control.txt" "${T0_MONOTONIC_NS}" \
        "${T1_MONOTONIC_NS}" "${OUTPUT_DIR}/perf-window-quality.txt"
}

parse_workload()
{
    local start_line="$1"
    local end_line="$2"
    local t0_ns="$3"
    local t1_ns="$4"
    python3 "${PROFILE_REPORT_PARSER}" c1-workload \
        "${OUTPUT_DIR}/measurement.csv" "${start_line}" "${end_line}" \
        "${t0_ns}" "${t1_ns}" "${OUTPUT_DIR}/workload-counts.txt"
}

generate_reports()
{
    case "${MODE}" in
        perf-stat)
            if ! python3 "${PROFILE_REPORT_PARSER}" perf-stat \
                "${OUTPUT_DIR}/perf-stat.csv" \
                "${OUTPUT_DIR}/perf-stat-quality.txt"
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
            if ! python3 "${PROFILE_REPORT_PARSER}" perf-record \
                "${OUTPUT_DIR}/perf-symbols.txt" \
                "${OUTPUT_DIR}/perf-report.txt" \
                "${OUTPUT_DIR}/perf-quality.txt" \
                "${OUTPUT_DIR}/perf-top10.txt" "${DURATION}"
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
                heaptrack_print -f "${heap_file}" -M "${OUTPUT_DIR}/heaptrack-massif.out" \
                    > "${OUTPUT_DIR}/heaptrack-massif.stdout.log" \
                    2> "${OUTPUT_DIR}/heaptrack-massif.stderr.log" \
                    || invalidate "heaptrack Massif timeline generation failed"
                python3 "${PROFILE_REPORT_PARSER}" heaptrack-massif \
                    "${OUTPUT_DIR}/heaptrack-massif.out" \
                    "${OUTPUT_DIR}/heaptrack-massif-quality.txt" \
                    "${EXPECTED_TARGET_EXE}" "${DURATION}" \
                    || invalidate "heaptrack Massif timeline gate failed"
                if ! python3 "${PROFILE_REPORT_PARSER}" heaptrack \
                    "${OUTPUT_DIR}/heaptrack-report.txt" \
                    "${OUTPUT_DIR}/heaptrack-quality.txt" "${DURATION}"
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
                if ! python3 "${PROFILE_REPORT_PARSER}" massif \
                    "${massif_file}" "${OUTPUT_DIR}/massif-quality.txt" \
                    "${EXPECTED_TARGET_EXE}" "${DURATION}"
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
            elif ! python3 "${PROFILE_REPORT_PARSER}" memcheck \
                "${memcheck_files[0]}" \
                "${OUTPUT_DIR}/memcheck-summary.txt" "${OUTPUT_DIR}/memcheck-quality.txt" \
                "${EXPECTED_TARGET_EXE}"
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
