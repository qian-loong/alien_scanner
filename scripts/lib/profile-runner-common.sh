#!/usr/bin/env bash

if [[ "${PROFILE_RUNNER_COMMON_LOADED:-false}" == true ]]; then
    return 0
fi
PROFILE_RUNNER_COMMON_LOADED=true

monotonic_ns()
{
    python3 -c 'import time; print(time.monotonic_ns())'
}

process_affinity_list()
{
    local pid="$1" affinity
    process_is_alive "${pid}" || return 1
    affinity="$(awk '$1 == "Cpus_allowed_list:" {print $2; exit}' \
        "/proc/${pid}/status" 2>/dev/null || true)"
    [[ "${affinity}" =~ ^[0-9]+(-[0-9]+)?(,[0-9]+(-[0-9]+)?)*$ ]] || return 1
    printf '%s\n' "${affinity}"
}

validate_pidstat_bracket()
{
    local start_ns="$1" t0_ns="$2" t1_ns="$3" stop_ns="$4" margin_ns="$5"
    local value
    for value in "${start_ns}" "${t0_ns}" "${t1_ns}" "${stop_ns}" "${margin_ns}"; do
        [[ "${value}" =~ ^[0-9]+$ ]] || return 2
    done
    (( start_ns <= t0_ns && t0_ns < t1_ns && t1_ns <= stop_ns \
        && t0_ns - start_ns <= margin_ns && stop_ns - t1_ns <= margin_ns ))
}

create_deterministic_source_archive()
{
    local source_root="$1" file_list="$2" output_path="$3"
    [[ -d "${source_root}" && -f "${file_list}" ]] || return 2
    tar -C "${source_root}" --sort=name --mtime=@0 --owner=0 --group=0 \
        --numeric-owner --format=gnu --verbatim-files-from \
        --use-compress-program='gzip -n' -cf "${output_path}" \
        --files-from "${file_list}"
}

realtime_stamp()
{
    python3 -c 'import time; value=time.time_ns(); print(f"{value // 1000000000}.{value % 1000000000:09d}")'
}

list_ros_trace_events()
{
    python3 - <<'PY'
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

print("\n".join(DEFAULT_EVENTS_ROS))
PY
}

lttng_ust_shm_headroom()
{
    local cpu_count="$1"
    local subbuf_size_bytes="$2"
    local num_subbuf="$3"
    local margin_bytes="$4"
    local available_bytes="${5:-}"
    local value required_bytes gate_pass
    for value in "${cpu_count}" "${subbuf_size_bytes}" "${num_subbuf}"; do
        [[ "${value}" =~ ^[1-9][0-9]*$ ]] || return 2
    done
    [[ "${margin_bytes}" =~ ^[0-9]+$ ]] || return 2
    if [[ -z "${available_bytes}" ]]; then
        available_bytes="$(df --output=avail -B1 /dev/shm 2>/dev/null \
            | tail -1 | tr -d '[:space:]')"
    fi
    [[ "${available_bytes}" =~ ^[0-9]+$ ]] || return 2

    required_bytes=$((cpu_count * subbuf_size_bytes * num_subbuf + margin_bytes))
    gate_pass=false
    (( available_bytes >= required_bytes )) && gate_pass=true
    printf 'stage_trace_cpu_count=%s\n' "${cpu_count}"
    printf 'stage_trace_subbuf_size_bytes=%s\n' "${subbuf_size_bytes}"
    printf 'stage_trace_num_subbuf=%s\n' "${num_subbuf}"
    printf 'stage_trace_shm_margin_bytes=%s\n' "${margin_bytes}"
    printf 'stage_trace_shm_required_bytes=%s\n' "${required_bytes}"
    printf 'stage_trace_shm_available_bytes=%s\n' "${available_bytes}"
    printf 'stage_trace_shm_gate_pass=%s\n' "${gate_pass}"
    [[ "${gate_pass}" == true ]]
}

wait_for_dead()
{
    local pid="$1"
    local timeout_seconds="$2"
    local deadline=$((SECONDS + timeout_seconds))
    while kill -0 "${pid}" 2>/dev/null; do
        if [[ -r "/proc/${pid}/stat" ]] \
            && [[ "$(awk '{print $3}' "/proc/${pid}/stat" 2>/dev/null || true)" == Z ]]; then
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

process_cmdline_has_exact_arguments()
{
    local pid="$1"
    shift
    python3 - "${pid}" "$@" <<'PY'
import os
import sys

pid = sys.argv[1]
expected = [os.fsencode(value) for value in sys.argv[2:]]
with open(f"/proc/{pid}/cmdline", "rb") as stream:
    arguments = [value for value in stream.read().split(b"\0") if value]
raise SystemExit(0 if expected and all(value in arguments for value in expected) else 1)
PY
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
