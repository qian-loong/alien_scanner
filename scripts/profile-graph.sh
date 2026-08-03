#!/usr/bin/env bash
# Orchestrate one graph-wide CPU/memory measurement over the real perception
# chain, either driven live by the simulated scene or replayed from a bag.
#
# Three field-observed facts shape the orchestration and are enforced as hard
# gates rather than left to convention:
#
#   1. The scene carries useful load for only about the trajectory duration.
#      `cave_laser_scan` is odometry driven and latches `scanning_stopped_` for
#      good once the trajectory finishes, after which nothing downstream has any
#      work at all. The formal window is therefore split into segments so that
#      the moving phase and the idle phase are never averaged together.
#   2. `ros2 launch` does not forward signals to the nodes it spawned. Scene
#      teardown is done by process group, and a residual sweep proves the group
#      is empty afterwards.
#   3. A recorder that is cleaned up before it exits on its own never writes
#      `metadata.yaml`, and such a bag is entirely unreadable. Teardown waits for
#      the recorder to exit and then proves the bag readable with `ros2 bag info`.
#
# Anything failing those gates sets `valid=false`. The raw evidence is kept, but
# the run does not count and the gate is not to be relaxed and the run repeated.
set -uo pipefail

usage()
{
    echo "usage: $0 <mode> <install-prefix> <new-output-dir> <duration-seconds>" >&2
    echo "modes: scene-record replay replay-loop" >&2
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
    scene-record|replay|replay-loop) ;;
    *) usage; exit 2 ;;
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
PROFILE_RUNNER_COMMON="${SCRIPT_DIR}/lib/profile-runner-common.sh"
GRAPH_SAMPLER="${SCRIPT_DIR}/lib/graph_sampler.py"
GRAPH_GATES="${SCRIPT_DIR}/lib/graph_run_gates.py"
GRAPH_EQUIVALENCE="${SCRIPT_DIR}/lib/graph_replay_equivalence.py"
GRAPH_TERMINAL_STATE="${SCRIPT_DIR}/lib/graph_terminal_state.py"
GRAPH_RESOLVER="${SCRIPT_DIR}/resolve-graph-pids.py"
GRAPH_ANALYZER="${SCRIPT_DIR}/analyze-graph-profile.py"
GRAPH_RECONCILER="${SCRIPT_DIR}/reconcile-graph-bags.py"
for required_file in "${PROFILE_RUNNER_COMMON}" "${GRAPH_SAMPLER}" "${GRAPH_GATES}" \
    "${GRAPH_EQUIVALENCE}" "${GRAPH_TERMINAL_STATE}" "${GRAPH_RESOLVER}" "${GRAPH_ANALYZER}" "${GRAPH_RECONCILER}"; do
    if [[ ! -r "${required_file}" ]]; then
        echo "missing profiling asset: ${required_file}" >&2
        exit 2
    fi
done
source "${PROFILE_RUNNER_COMMON}"

OUTPUT_DIR="$(readlink -m "${OUTPUT_DIR}")"
mkdir -p "${OUTPUT_DIR}"

SAMPLE_INTERVAL_S="${ALIEN_GRAPH_SAMPLE_INTERVAL_S:-1.0}"
PIDSTAT_INTERVAL_S="${ALIEN_GRAPH_PIDSTAT_INTERVAL_S:-1}"
READY_TIMEOUT_S="${ALIEN_GRAPH_READY_TIMEOUT_S:-90}"
RECORDER_EXIT_TIMEOUT_S="${ALIEN_GRAPH_RECORDER_EXIT_TIMEOUT_S:-90}"
SCENE_EXIT_TIMEOUT_S="${ALIEN_GRAPH_SCENE_EXIT_TIMEOUT_S:-60}"
PIDSTAT_BRACKET_MARGIN_NS=2000000000
PRODUCT_NODES="${ALIEN_GRAPH_PRODUCT_NODES:-cave_full_ray_perception_input cave_full_ray_local_map}"
FIXTURE_NODES="${ALIEN_GRAPH_FIXTURE_NODES:-cave_full_ray_truth cave_full_ray_odom cave_full_ray_scanner cave_full_ray_pose_gate}"
UNMEASURED_SCENE_NODES="${ALIEN_GRAPH_UNMEASURED_SCENE_NODES:-cave_full_ray_body_to_scan}"
MOTION_LATCH_PATTERN="${ALIEN_GRAPH_MOTION_LATCH_PATTERN:-raw LaserScan acquisition stopped}"
# Default 5: the mapper's recovery stability gate consumes 0-3 observations at
# startup depending on the health/pose race (measured), so direct and replay
# terminal revisions may legitimately differ by a few commits.
EQUIVALENCE_COUNT_TOLERANCE="${ALIEN_GRAPH_EQUIVALENCE_COUNT_TOLERANCE:-5}"

VALID=true
NORMAL_COMPLETION=false
FORCED_STOP=false
ROLE_EXIT_FAILURE=false
SCRIPT_PGID="$(ps -o pgid= -p "$$" | tr -d ' ')"
T0_MONOTONIC_NS=""
T1_MONOTONIC_NS=""
MOTION_LATCH_MONOTONIC_NS=""
PIDSTAT_START_MONOTONIC_NS=""
PIDSTAT_STOP_MONOTONIC_NS=""

# Parallel arrays: one entry per launched process, in launch order.
ROLE_NAMES=()
ROLE_PIDS=()
ROLE_PGIDS=()
ROLE_STARTTIMES=()
ROLE_REAPED=()
RECORDER_NAMES=()
RECORDER_TOPICS=()
RECORDER_DIRS=()
RECORDER_PAUSED=()
RECORDER_ROLE_INDEXES=()

invalidate()
{
    local reason="$1"
    VALID=false
    echo "INVALID: ${reason}" | tee -a "${OUTPUT_DIR}/invalid-reasons.txt" >&2
}

role_index()
{
    local name="$1" index
    for index in "${!ROLE_NAMES[@]}"; do
        if [[ "${ROLE_NAMES[index]}" == "${name}" ]]; then
            echo "${index}"
            return 0
        fi
    done
    return 1
}

record_role()
{
    local role="$1" pid="$2" pgid="$3" starttime="$4"
    {
        echo "${role}_pid=${pid}"
        echo "${role}_pgid=${pgid}"
        echo "${role}_starttime=${starttime}"
    } >> "${OUTPUT_DIR}/run-manifest.txt"
}

# Launch one role in its own session so it can always be torn down as a group.
# `ros2 launch` in particular does not pass signals on to the nodes it spawned,
# so a group is the only handle that reaches every descendant.
launch_role()
{
    local name="$1" logfile="$2"
    shift 2
    setsid "$@" > "${logfile}" 2>&1 &
    local pid=$! pgid starttime
    if ! pgid="$(wait_for_isolated_pgid "${pid}")"; then
        invalidate "${name} did not obtain an isolated process group"
        return 1
    fi
    starttime="$(process_starttime "${pid}" 2>/dev/null || true)"
    if [[ -z "${starttime}" ]]; then
        invalidate "${name} identity did not stabilize"
        return 1
    fi
    ROLE_NAMES+=("${name}")
    ROLE_PIDS+=("${pid}")
    ROLE_PGIDS+=("${pgid}")
    ROLE_STARTTIMES+=("${starttime}")
    ROLE_REAPED+=(false)
    record_role "${name}" "${pid}" "${pgid}" "${starttime}"
    return 0
}

# Signal a role's whole process group and wait for it to go away on its own.
# Escalation past the first signal is recorded as a forced stop: a role that had
# to be killed may have left its artifacts truncated.
stop_role()
{
    local index="$1" signal="${2:-INT}" timeout="${3:-20}"
    if [[ ! "${index}" =~ ^[0-9]+$ || "${index}" -ge "${#ROLE_NAMES[@]}" ]]; then
        invalidate "refusing to stop an unknown role index '${index}'"
        return 1
    fi
    local name="${ROLE_NAMES[index]}" pid="${ROLE_PIDS[index]}"
    local pgid="${ROLE_PGIDS[index]}" starttime="${ROLE_STARTTIMES[index]}" rc=0
    [[ "${ROLE_REAPED[index]}" == true ]] && return 0
    if ! process_is_alive "${pid}"; then
        wait_child "${pid}" "${name}" || rc=$?
        ROLE_REAPED[index]=true
        ROLE_EXIT_FAILURE=true
        invalidate "${name} exited before the requested normal stop (${rc})"
        return 0
    fi
    signal_process "${signal}" "${pid}" "${pgid}" "${starttime}" || return 1
    if ! wait_for_dead "${pid}" "${timeout}"; then
        FORCED_STOP=true
        invalidate "${name} did not exit after SIG${signal} within ${timeout}s"
        signal_process TERM "${pid}" "${pgid}" "${starttime}" || true
        if ! wait_for_dead "${pid}" 5; then
            signal_process KILL "${pid}" "${pgid}" "${starttime}" || true
            wait_for_dead "${pid}" 5 || true
        fi
    fi
    wait_child "${pid}" "${name}" || rc=$?
    ROLE_REAPED[index]=true
    if (( rc != 0 )) \
        && ! [[ "${signal}" == INT && "${rc}" -eq 130 ]] \
        && ! [[ "${signal}" == TERM && "${rc}" -eq 143 ]]; then
        ROLE_EXIT_FAILURE=true
        invalidate "${name} exited unexpectedly (${rc})"
    fi
    return 0
}

signal_role()
{
    local index="$1" signal="$2"
    signal_process "${signal}" "${ROLE_PIDS[index]}" "${ROLE_PGIDS[index]}" \
        "${ROLE_STARTTIMES[index]}"
}

# Measured on this image: `ros2 bag record` does not react to SIGINT at all. It
# reacts to SIGTERM by stopping the recording, writing metadata.yaml and exiting
# with status 0 - that is its graceful stop. SIGTERM is therefore the normal stop
# for these roles, not a forced one; only a KILL means the artifacts may be
# truncated.
stop_ros2_cli_role()
{
    local index="$1" int_timeout="${2:-5}" term_timeout="${3:-30}"
    if [[ ! "${index}" =~ ^[0-9]+$ || "${index}" -ge "${#ROLE_NAMES[@]}" ]]; then
        invalidate "refusing to stop an unknown role index '${index}'"
        return 1
    fi
    local name="${ROLE_NAMES[index]}" pid="${ROLE_PIDS[index]}" rc=0
    [[ "${ROLE_REAPED[index]}" == true ]] && return 0
    if ! process_is_alive "${pid}"; then
        wait_child "${pid}" "${name}" || rc=$?
        ROLE_REAPED[index]=true
        ROLE_EXIT_FAILURE=true
        invalidate "${name} exited before the requested normal stop (${rc})"
        return 0
    fi
    signal_role "${index}" INT || return 1
    if ! wait_for_dead "${pid}" "${int_timeout}"; then
        signal_role "${index}" TERM || return 1
        if ! wait_for_dead "${pid}" "${term_timeout}"; then
            FORCED_STOP=true
            invalidate "${name} did not exit after SIGINT then SIGTERM"
            signal_role "${index}" KILL || true
            wait_for_dead "${pid}" 5 || true
        fi
    fi
    wait_child "${pid}" "${name}" || rc=$?
    ROLE_REAPED[index]=true
    if (( rc != 0 && rc != 130 && rc != 143 )); then
        ROLE_EXIT_FAILURE=true
        invalidate "${name} exited unexpectedly (${rc})"
    fi
    return 0
}

# Prove a process group is empty. `ros2 launch` leaving descendants behind was
# observed running for 38 minutes and silently polluting later measurements.
assert_group_is_empty()
{
    local name="$1"
    local pgid="$2"
    # Declared separately: bash expands every word of a `local` statement before
    # the builtin assigns any of them, so `name` would still be unset here.
    local evidence="${OUTPUT_DIR}/${name}-residual-ps.txt"
    ps -o pid= -g "${pgid}" > "${evidence}" 2>/dev/null || true
    if ! python3 "${GRAPH_GATES}" --report "${OUTPUT_DIR}/${name}-residual-gate.txt" \
        residual "${evidence}" >> "${OUTPUT_DIR}/gate.log" 2>&1; then
        invalidate "${name} left residual processes in its process group"
        return 1
    fi
    return 0
}

cleanup()
{
    local original_rc=$? index
    trap - EXIT INT TERM
    for (( index=${#ROLE_NAMES[@]}-1; index>=0; index-- )); do
        stop_role "${index}" TERM 10 || true
    done
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
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" || SETUP_OK=false
source "${INSTALL_PREFIX}/setup.bash" || SETUP_OK=false
set -u
if [[ "${SETUP_OK}" != true ]]; then
    invalidate "ROS or workspace install setup failed"
    exit 1
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export RCUTILS_COLORIZED_OUTPUT=0
export LC_ALL=C
if [[ -n "${ALIEN_PROFILE_ROS_DOMAIN_ID:-}" ]]; then
    export ROS_DOMAIN_ID="${ALIEN_PROFILE_ROS_DOMAIN_ID}"
else
    for candidate in $(seq 100 229 | shuf); do
        if eval "exec 7>/tmp/alien-graph-profile-domain-${candidate}.lock" && flock -n 7; then
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

LOCAL_MAP_PREFIX="$(ros2 pkg prefix perception_local_map 2>/dev/null || true)"
LOCAL_MAP_PREFIX="$(readlink -f "${LOCAL_MAP_PREFIX}" 2>/dev/null || true)"
if [[ -z "${LOCAL_MAP_PREFIX}" || ( "${LOCAL_MAP_PREFIX}" != "${INSTALL_PREFIX}" \
    && "${LOCAL_MAP_PREFIX}" != "${INSTALL_PREFIX}/"* ) ]]; then
    invalidate "perception_local_map does not resolve from the requested install prefix"
    exit 1
fi
SCENE_CONFIG="${ALIEN_GRAPH_SCENE_CONFIG:-${LOCAL_MAP_PREFIX}/share/perception_local_map/config/cave_full_ray_scene.yaml}"
if [[ ! -r "${SCENE_CONFIG}" ]]; then
    invalidate "scene configuration is unreadable: ${SCENE_CONFIG}"
    exit 1
fi

# Every topic name and the trajectory duration come from the scene config. The
# runner hard-codes neither, so a config change cannot silently desynchronize
# the recorded topic set from the graph that is actually running.
SCENE_READER="$(cat <<'PY'
import sys

import yaml

with open(sys.argv[1], encoding="utf-8") as stream:
    config = yaml.safe_load(stream)
for key, value in config["topics"].items():
    print(f"topic_{key}={value}")
print(f"trajectory_duration_s={config['trajectory']['duration_s']}")
print(f"transport_drain_s={config['timing']['transport_drain_s']}")
print(f"odometry_rate_hz={config['trajectory']['odometry_rate_hz']}")
print(f"scan_rate_hz={config['scan']['rate_hz']}")
PY
)"
if ! python3 -c "${SCENE_READER}" "${SCENE_CONFIG}" \
    > "${OUTPUT_DIR}/scene-values.txt" 2> "${OUTPUT_DIR}/scene-values.err"; then
    invalidate "could not read topics and timing from the scene configuration"
    exit 1
fi

scene_value()
{
    awk -F= -v key="$1" '$1 == key {print substr($0, index($0, "=") + 1); exit}' \
        "${OUTPUT_DIR}/scene-values.txt"
}

cat > "${OUTPUT_DIR}/qos-overrides.yaml" << 'QOS'
/tf_static:
  history: keep_all
  reliability: reliable
  durability: transient_local
QOS

TOPIC_ODOMETRY="$(scene_value topic_odometry)"
TOPIC_RAW_SCAN="$(scene_value topic_raw_scan)"
TOPIC_RELEASED_SCAN="$(scene_value topic_released_scan)"
TOPIC_OBSERVATIONS="$(scene_value topic_observations)"
TOPIC_POSE="$(scene_value topic_pose)"
TOPIC_HEALTH="$(scene_value topic_health)"
TOPIC_STATE="$(scene_value topic_local_map_state)"
TOPIC_OCTOMAP="$(scene_value topic_local_map_octomap)"
TRAJECTORY_DURATION_S="$(scene_value trajectory_duration_s)"
TRANSPORT_DRAIN_S="$(scene_value transport_drain_s)"
for value in "${TOPIC_ODOMETRY}" "${TOPIC_RAW_SCAN}" "${TOPIC_RELEASED_SCAN}" \
    "${TOPIC_OBSERVATIONS}" "${TOPIC_POSE}" "${TOPIC_HEALTH}" "${TOPIC_STATE}" \
    "${TOPIC_OCTOMAP}" "${TRAJECTORY_DURATION_S}" "${TRANSPORT_DRAIN_S}"; do
    if [[ -z "${value}" ]]; then
        invalidate "scene configuration is missing a required topic or timing value"
        exit 1
    fi
done

git -C "${REPO_ROOT}" status --short > "${OUTPUT_DIR}/git-status.txt" 2>&1 || true
SOURCE_INPUT_PATHS=(.devcontainer scripts ws/src CMakeLists.txt)
git -C "${REPO_ROOT}" diff --binary HEAD -- "${SOURCE_INPUT_PATHS[@]}" \
    > "${OUTPUT_DIR}/source-diff.patch" 2>&1 || true
git -C "${REPO_ROOT}" ls-files --others --exclude-standard -- "${SOURCE_INPUT_PATHS[@]}" \
    > "${OUTPUT_DIR}/source-untracked.txt" 2>&1 || true
create_deterministic_source_archive "${REPO_ROOT}" \
    "${OUTPUT_DIR}/source-untracked.txt" "${OUTPUT_DIR}/source-untracked.tar.gz" \
    || { invalidate "deterministic source archive creation failed"; exit 1; }
SOURCE_REVISION="$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || echo unavailable)"

# Build provenance for every measured product ELF, so the numbers can be tied
# back to the exact binary that produced them.
: > "${OUTPUT_DIR}/measured-build-ids.txt"
for package_executable in perception_input_node:perception_input_node \
    perception_local_map:perception_local_map_node; do
    package="${package_executable%%:*}"
    executable="${package_executable##*:}"
    prefix="$(readlink -f "$(ros2 pkg prefix "${package}" 2>/dev/null || true)" 2>/dev/null || true)"
    path="${prefix}/lib/${package}/${executable}"
    if [[ ! -x "${path}" ]]; then
        invalidate "measured product executable is missing: ${path}"
        exit 1
    fi
    {
        echo "measured_exe[${executable}]=${path}"
        echo "measured_exe_sha256[${executable}]=$(sha256sum "${path}" | awk '{print $1}')"
        echo "measured_exe_build_id[${executable}]=$(readelf -n "${path}" 2>/dev/null \
            | awk '/Build ID/ {print $3; exit}')"
    } >> "${OUTPUT_DIR}/measured-build-ids.txt"
done

{
    echo "mode=graph-${MODE}"
    echo "duration_requested_s=${DURATION}"
    echo "started_at=$(date --iso-8601=ns)"
    echo "source_revision=${SOURCE_REVISION}"
    echo "source_dirty_count=$(wc -l < "${OUTPUT_DIR}/git-status.txt")"
    echo "source_diff_sha256=$(sha256sum "${OUTPUT_DIR}/source-diff.patch" | awk '{print $1}')"
    echo "source_untracked_archive_sha256=$(sha256sum "${OUTPUT_DIR}/source-untracked.tar.gz" | awk '{print $1}')"
    echo "profile_script_sha256=$(sha256sum "${BASH_SOURCE[0]}" | awk '{print $1}')"
    echo "graph_sampler_sha256=$(sha256sum "${GRAPH_SAMPLER}" | awk '{print $1}')"
    echo "graph_run_gates_sha256=$(sha256sum "${GRAPH_GATES}" | awk '{print $1}')"
    echo "graph_replay_equivalence_sha256=$(sha256sum "${GRAPH_EQUIVALENCE}" | awk '{print $1}')"
    echo "graph_terminal_state_sha256=$(sha256sum "${GRAPH_TERMINAL_STATE}" | awk '{print $1}')"
    echo "graph_reconciler_sha256=$(sha256sum "${GRAPH_RECONCILER}" | awk '{print $1}')"
    echo "graph_resolver_sha256=$(sha256sum "${GRAPH_RESOLVER}" | awk '{print $1}')"
    echo "analysis_script_sha256=$(sha256sum "${GRAPH_ANALYZER}" | awk '{print $1}')"
    echo "scene_config=${SCENE_CONFIG}"
    echo "scene_config_sha256=$(sha256sum "${SCENE_CONFIG}" | awk '{print $1}')"
    echo "container_id=$(hostname)"
    echo "image_id=${PROFILE_IMAGE_ID}"
    echo "kernel=$(uname -srvm)"
    echo "rmw=${RMW_IMPLEMENTATION}"
    echo "ros_domain_id=${ROS_DOMAIN_ID}"
    echo "nproc=$(nproc)"
    echo "cpu_max=$(cat /sys/fs/cgroup/cpu.max 2>/dev/null || echo unavailable)"
    echo "memory_max=$(cat /sys/fs/cgroup/memory.max 2>/dev/null || echo unavailable)"
    echo "product_nodes=${PRODUCT_NODES}"
    echo "fixture_nodes=${FIXTURE_NODES}"
    echo "unmeasured_scene_nodes=${UNMEASURED_SCENE_NODES}"
    echo "sample_interval_s=${SAMPLE_INTERVAL_S}"
    cat "${OUTPUT_DIR}/measured-build-ids.txt"
} > "${OUTPUT_DIR}/run-manifest.txt"
{
    echo "pidstat=$(pidstat -V 2>&1 | head -1)"
    echo "ros2_bag=$(ros2 bag --help >/dev/null 2>&1 && echo available || echo unavailable)"
} > "${OUTPUT_DIR}/tool-versions.txt"

start_scene()
{
    launch_role scene "${OUTPUT_DIR}/scene.log" \
        ros2 launch perception_local_map cave_full_ray_scene.launch.py \
        show_rviz:=false "scene_config:=${SCENE_CONFIG}" || return 1
    echo "scene_launched_monotonic_ns=$(monotonic_ns)" >> "${OUTPUT_DIR}/run-manifest.txt"
}

# The recorders start BEFORE the scene and are never paused.
#
# `ros2 bag record` keeps discovery on by default and polls at
# --polling-interval until every requested topic exists; the "Stopping
# discovery" log line fires only once ALL requested topics are subscribed. A
# recorder started before its publishers therefore waits for them rather than
# missing them. Measured: recorder up 3 s ahead of the scene, 9/9 topics
# subscribed, raw_scan/scan/observations 199/199/199, C2 state revision 0..198
# contiguous, first revision 0 - zero loss.
#
# --start-paused must NOT be used. /tf_static is published exactly once with
# transient-local durability, so its only copy lands during the paused period
# and is dropped, producing a bag with zero /tf_static that cannot replay
# anything needing a transform. Every recorder therefore carries /tf_static
# itself, which makes each bag self-contained.
start_recorder()
{
    local name="$1"
    local topics_csv="$2"
    local paused="${3:-false}"
    local bag_dir="${OUTPUT_DIR}/bag-${name}"
    local -a topics=()
    IFS=',' read -r -a topics <<< "${topics_csv}"
    local -a options=()
    [[ "${paused}" == true ]] && options+=(--start-paused)
    if [[ "${topics_csv}" == *"/tf_static"* ]]; then
        options+=(--qos-profile-overrides-path "${OUTPUT_DIR}/qos-overrides.yaml")
    fi
    launch_role "recorder_${name}" "${OUTPUT_DIR}/record-${name}.log" \
        "${helper_env[@]}" ros2 bag record --node-name "graph_recorder_${name}" \
        "${options[@]}" -o "${bag_dir}" --topics "${topics[@]}" || return 1
    RECORDER_NAMES+=("${name}")
    RECORDER_TOPICS+=("${topics_csv}")
    RECORDER_DIRS+=("${bag_dir}")
    RECORDER_PAUSED+=("${paused}")
    RECORDER_ROLE_INDEXES+=("$(( ${#ROLE_NAMES[@]} - 1 ))")
}

# Hard readiness gate: `ros2 bag record` stops topic discovery once it has
# subscribed to whatever existed at that moment, and then records nothing for a
# topic that appeared later without ever complaining.
wait_for_recorder_subscriptions()
{
    local deadline=$((SECONDS + READY_TIMEOUT_S)) index ready
    while (( SECONDS < deadline )); do
        ready=true
        for index in "${!RECORDER_NAMES[@]}"; do
            local -a topics=()
            IFS=',' read -r -a topics <<< "${RECORDER_TOPICS[index]}"
            if ! python3 "${GRAPH_GATES}" \
                --report "${OUTPUT_DIR}/recorder-${RECORDER_NAMES[index]}-subscription-gate.txt" \
                subscriptions "${OUTPUT_DIR}/record-${RECORDER_NAMES[index]}.log" \
                "${topics[@]}" >> "${OUTPUT_DIR}/gate.log" 2>&1; then
                ready=false
            fi
            process_is_alive "${ROLE_PIDS[RECORDER_ROLE_INDEXES[index]]}" || return 1
        done
        [[ "${ready}" == true ]] && return 0
        sleep 0.25
    done
    return 1
}

wait_for_rosbag_resume_service()
{
    local node_name="$1" deadline=$((SECONDS + READY_TIMEOUT_S))
    while (( SECONDS < deadline )); do
        if ros2 service list 2>/dev/null | grep -Fxq "/${node_name}/resume"; then
            return 0
        fi
        sleep 0.25
    done
    return 1
}

resume_rosbag_node()
{
    local node_name="$1"
    wait_for_rosbag_resume_service "${node_name}" || return 1
    ros2 service call "/${node_name}/resume" rosbag2_interfaces/srv/Resume \
        >> "${OUTPUT_DIR}/rosbag-resume.log" 2>&1
}

# Recorders now start unpaused, so this is a no-op for them. It is kept because
# a paused recorder remains expressible via start_recorder's third argument, and
# skipping the unpaused ones keeps the pre-window cost at zero - every second
# spent here would be taken out of a moving phase that only lasts as long as the
# trajectory.
resume_recorders()
{
    local index failed=false
    local -a paused_indexes=()
    for index in "${!RECORDER_NAMES[@]}"; do
        [[ "${RECORDER_PAUSED[index]}" == true ]] && paused_indexes+=("${index}")
    done
    if (( ${#paused_indexes[@]} == 0 )); then
        echo "recorders_resumed_monotonic_ns=$(monotonic_ns)" >> "${OUTPUT_DIR}/run-manifest.txt"
        echo "recorders_started_unpaused=true" >> "${OUTPUT_DIR}/run-manifest.txt"
        return 0
    fi
    for index in "${paused_indexes[@]}"; do
        wait_for_rosbag_resume_service "graph_recorder_${RECORDER_NAMES[index]}" \
            || { invalidate "recorder ${RECORDER_NAMES[index]} never offered a resume service"; return 1; }
    done
    local -a resume_pids=()
    for index in "${paused_indexes[@]}"; do
        ros2 service call "/graph_recorder_${RECORDER_NAMES[index]}/resume" \
            rosbag2_interfaces/srv/Resume \
            >> "${OUTPUT_DIR}/rosbag-resume.log" 2>&1 &
        resume_pids+=("$!")
    done
    for index in "${!resume_pids[@]}"; do
        wait "${resume_pids[index]}" || failed=true
    done
    if [[ "${failed}" == true ]]; then
        invalidate "a recorder did not resume"
        return 1
    fi
    echo "recorders_resumed_monotonic_ns=$(monotonic_ns)" >> "${OUTPUT_DIR}/run-manifest.txt"
}

# Teardown gate. The observable proof that a recorder finished is metadata.yaml
# appearing - not the process exiting, and certainly not the signal having been
# sent. Cleaning up before that file exists leaves a bag that cannot be opened at
# all, so the wait is on the file itself.
wait_for_recorder_finalization()
{
    local deadline=$((SECONDS + RECORDER_EXIT_TIMEOUT_S)) index ready missing attempts=0
    missing=""
    while (( SECONDS < deadline )); do
        ready=true
        missing=""
        for index in "${!RECORDER_DIRS[@]}"; do
            if [[ ! -s "${RECORDER_DIRS[index]}/metadata.yaml" ]]; then
                ready=false
                missing="${missing} ${RECORDER_NAMES[index]}"
                # Repeating the stop request is harmless for a recorder that has
                # already stopped, so a missed signal is retried, never escalated.
                signal_role "${RECORDER_ROLE_INDEXES[index]}" TERM || true
            fi
        done
        if [[ "${ready}" == true ]]; then
            echo "recorder_finalization_attempts=${attempts}" >> "${OUTPUT_DIR}/run-manifest.txt"
            return 0
        fi
        attempts=$((attempts + 1))
        sleep 1
    done
    invalidate "recorders did not finalize their bags:${missing}"
    return 1
}

# Collect a role that was already asked to stop. Its exit is the expected
# outcome here, so unlike stop_role this is not the place that decides a role
# died prematurely; that judgement was made when the stop was requested.
reap_requested_role()
{
    local index="$1" timeout="${2:-30}"
    local name="${ROLE_NAMES[index]}" pid="${ROLE_PIDS[index]}" rc=0
    [[ "${ROLE_REAPED[index]}" == true ]] && return 0
    if ! wait_for_dead "${pid}" "${timeout}"; then
        FORCED_STOP=true
        invalidate "${name} did not exit after its stop was requested"
        signal_role "${index}" KILL || true
        wait_for_dead "${pid}" 5 || true
    fi
    wait_child "${pid}" "${name}" || rc=$?
    ROLE_REAPED[index]=true
    if (( rc != 0 && rc != 130 && rc != 143 )); then
        ROLE_EXIT_FAILURE=true
        invalidate "${name} exited unexpectedly (${rc})"
    fi
    return 0
}

stop_recorders_and_verify_bags()
{
    local index role_index failed=false
    echo "recorder_stop_requested_monotonic_ns=$(monotonic_ns)" \
        >> "${OUTPUT_DIR}/run-manifest.txt"
    for index in "${!RECORDER_NAMES[@]}"; do
        signal_role "${RECORDER_ROLE_INDEXES[index]}" TERM || failed=true
    done
    wait_for_recorder_finalization || failed=true
    echo "recorder_finalized_monotonic_ns=$(monotonic_ns)" \
        >> "${OUTPUT_DIR}/run-manifest.txt"
    for index in "${!RECORDER_NAMES[@]}"; do
        role_index="${RECORDER_ROLE_INDEXES[index]}"
        reap_requested_role "${role_index}" "${RECORDER_EXIT_TIMEOUT_S}" || failed=true
    done
    for index in "${!RECORDER_NAMES[@]}"; do
        local name="${RECORDER_NAMES[index]}" bag_dir="${RECORDER_DIRS[index]}"
        local -a topics=()
        IFS=',' read -r -a topics <<< "${RECORDER_TOPICS[index]}"
        ros2 bag info "${bag_dir}" > "${OUTPUT_DIR}/bag-${name}-info.txt" 2>&1 || true
        if ! python3 "${GRAPH_GATES}" --report "${OUTPUT_DIR}/bag-${name}-gate.txt" \
            bag "${bag_dir}" "${OUTPUT_DIR}/bag-${name}-info.txt" "${topics[@]}" \
            >> "${OUTPUT_DIR}/gate.log" 2>&1; then
            invalidate "bag ${name} is not readable evidence"
            failed=true
        fi
    done
    # Loss reconciliation. Equal counts across stages is NOT the property to
    # check - the pose gate and C1 both reject by contract, so upstream
    # legitimately exceeds downstream, and upstream production itself varies
    # with load (measured: raw_scan 193..199 across three runs). What proves the
    # recording is complete is C2's own revision ledger, which the recorder
    # cannot influence: contiguous revisions, and as many observations captured
    # as were committed.
    if [[ "${MODE}" == scene-record ]]; then
        if ! python3 "${GRAPH_RECONCILER}" \
            --report "${OUTPUT_DIR}/bag-reconciliation.txt" \
            --json "${OUTPUT_DIR}/bag-reconciliation.json" \
            --stage "raw_scan=${OUTPUT_DIR}/bag-all:${TOPIC_RAW_SCAN}" \
            --stage "released_scan=${OUTPUT_DIR}/bag-all:${TOPIC_RELEASED_SCAN}" \
            --stage "observations=${OUTPUT_DIR}/bag-all:${TOPIC_OBSERVATIONS}" \
            --revisions "${OUTPUT_DIR}/bag-all:${TOPIC_STATE}" \
            >> "${OUTPUT_DIR}/gate.log" 2>&1; then
            invalidate "recorded evidence is incomplete - see bag-reconciliation.txt"
            failed=true
        fi
    fi
    [[ "${failed}" == false ]]
}

# Terminal identity is captured AFTER t1, never during. A subscriber living
# through the window would perturb exactly the CPU numbers the window exists to
# measure, and only the last value is needed - so the cost is moved outside.
capture_terminal_state()
{
    "${helper_env[@]}" python3 "${GRAPH_TERMINAL_STATE}" "${OUTPUT_DIR}/terminal-state.txt" \
        --state-topic "${TOPIC_STATE}" \
        >> "${OUTPUT_DIR}/terminal-state.log" 2>&1
}

# Freeze every expected node's identity before the window opens. A node that is
# missing or that matches two processes makes the whole graph unattributable, so
# either is fatal rather than something to work around.
freeze_identities()
{
    local -a arguments=()
    local node index
    for node in ${PRODUCT_NODES}; do
        arguments+=(--product "${node}")
    done
    for node in ${FIXTURE_NODES}; do
        arguments+=(--fixture "${node}")
    done
    for index in "${!ROLE_NAMES[@]}"; do
        case "${ROLE_NAMES[index]}" in
            recorder_*|bag_play)
                arguments+=(--support "${ROLE_NAMES[index]}=${ROLE_PIDS[index]}")
                ;;
        esac
    done
    local deadline=$((SECONDS + READY_TIMEOUT_S))
    while (( SECONDS < deadline )); do
        if python3 "${GRAPH_RESOLVER}" "${arguments[@]}" \
            --output "${OUTPUT_DIR}/graph-identities.json" \
            > "${OUTPUT_DIR}/resolve-graph-pids.log" 2>&1; then
            return 0
        fi
        sleep 0.5
    done
    return 1
}

frozen_pids()
{
    python3 -c '
import json
import sys
from pathlib import Path

payload = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
print(",".join(str(record["pid"]) for record in payload["nodes"]))
' "${OUTPUT_DIR}/graph-identities.json"
}

start_pidstat()
{
    local pids="$1"
    launch_role pidstat "${OUTPUT_DIR}/pidstat.txt" \
        stdbuf -oL -eL pidstat -h -u -r -p "${pids}" "${PIDSTAT_INTERVAL_S}" || return 1
    PIDSTAT_START_MONOTONIC_NS="$(monotonic_ns)"
    {
        echo "pidstat_start_monotonic_ns=${PIDSTAT_START_MONOTONIC_NS}"
        echo "pidstat_interval_s=${PIDSTAT_INTERVAL_S}"
        echo "pidstat_pids=${pids}"
    } >> "${OUTPUT_DIR}/run-manifest.txt"
}

start_graph_sampler()
{
    launch_role graph_sampler "${OUTPUT_DIR}/graph-sampler.log" \
        env PYTHONPATH="${SCRIPT_DIR}" python3 -m lib.graph_sampler \
        "${OUTPUT_DIR}/graph-identities.json" "${OUTPUT_DIR}/graph-samples.csv" \
        "${OUTPUT_DIR}/graph-headroom.csv" "${OUTPUT_DIR}/graph-identity-violation.txt" \
        --interval-s "${SAMPLE_INTERVAL_S}"
}

# Watch for the scanner's own report that the trajectory finished. Using the
# observed latch rather than an assumed duration is what keeps the moving phase
# and the idle phase from bleeding into one another.
monitor_window()
{
    local deadline=$((SECONDS + DURATION)) index
    while (( SECONDS < deadline )); do
        for index in "${!ROLE_NAMES[@]}"; do
            if ! process_identity_matches "${ROLE_PIDS[index]}" \
                "${ROLE_STARTTIMES[index]}" "${ROLE_PGIDS[index]}"; then
                invalidate "${ROLE_NAMES[index]} exited or changed identity during the formal window"
                return 1
            fi
        done
        if [[ -z "${MOTION_LATCH_MONOTONIC_NS}" && "${MODE}" == scene-record ]] \
            && grep -Fq "${MOTION_LATCH_PATTERN}" "${OUTPUT_DIR}/scene.log" 2>/dev/null; then
            MOTION_LATCH_MONOTONIC_NS="$(monotonic_ns)"
            echo "motion_latch_monotonic_ns=${MOTION_LATCH_MONOTONIC_NS}" \
                >> "${OUTPUT_DIR}/run-manifest.txt"
        fi
        if [[ -f "${OUTPUT_DIR}/graph-identity-violation.txt" ]]; then
            invalidate "graph sampler reported a PID identity violation"
            return 1
        fi
        sleep 0.25
    done
    return 0
}

start_replay_nodes()
{
    local specification package executable node_name
    local -a arguments
    for specification in ${REPLAY_NODES}; do
        package="${specification%%:*}"
        executable="${specification#*:}"
        executable="${executable%%:*}"
        node_name="${specification##*:}"
        local prefix path
        prefix="$(readlink -f "$(ros2 pkg prefix "${package}" 2>/dev/null || true)" 2>/dev/null || true)"
        path="${prefix}/lib/${package}/${executable}"
        if [[ ! -x "${path}" ]]; then
            invalidate "replay node executable is missing: ${path}"
            return 1
        fi
        arguments=("${path}" --ros-args -r "__node:=${node_name}")
        if [[ -n "${REPLAY_PARAMS}" ]]; then
            arguments+=(--params-file "${REPLAY_PARAMS}")
        fi
        launch_role "node_${node_name}" "${OUTPUT_DIR}/node-${node_name}.log" \
            "${arguments[@]}" || return 1
    done
}

# The player is a support role: it is sampled so its cost is quantified, and its
# resource use is never attributed to a node under test. It starts paused for the
# same reason the recorder does - so that playback begins at a moment the runner
# chooses, after every identity is frozen and every sampler is running.
start_bag_play()
{
    local -a arguments=("${helper_env[@]}" ros2 bag play --start-paused)
    local bag
    # Repeated -i lets the static-transform bag be played alongside the data bag.
    # A node replayed without /tf_static cannot resolve its sensor extrinsic and
    # silently drops every observation, which looks like a replay that lost data.
    for bag in ${REPLAY_BAG}; do
        arguments+=(-i "${bag}")
    done
    # bag-all carries every stage's topics. Replay MUST select only the cut
    # point feeding the node under test - playing the node's own recorded
    # outputs back alongside its inputs would double-publish them and corrupt
    # the run.
    if [[ -n "${REPLAY_TOPICS}" ]]; then
        arguments+=(--topics)
        for topic in ${REPLAY_TOPICS}; do
            arguments+=("${topic}")
        done
    fi
    # replay-loop plays a pre-concatenated stamp-shifted bag exactly once.
    # `--loop` is NOT used: the mapper's stamp high-water gates reject every
    # observation and pose from the second pass of a looped bag (measured
    # 2026-08-02: revision frozen, 100% "observation replay or rollback").
    # Build the long bag with scripts/restamp-concat-bag.py instead.
    [[ -n "${REPLAY_RATE}" ]] && arguments+=(--rate "${REPLAY_RATE}")
    launch_role bag_play "${OUTPUT_DIR}/bag-play.log" "${arguments[@]}" || return 1
    {
        echo "replay_bag=${REPLAY_BAG}"
        echo "replay_cycle_mode=$([[ "${MODE}" == replay-loop ]] && echo true || echo false)"
        echo "replay_rate=${REPLAY_RATE:-1.0}"
    } >> "${OUTPUT_DIR}/run-manifest.txt"
}

resume_bag_play()
{
    resume_rosbag_node rosbag2_player || { invalidate "bag play did not resume"; return 1; }
    echo "bag_play_resumed_monotonic_ns=$(monotonic_ns)" >> "${OUTPUT_DIR}/run-manifest.txt"
}

wait_for_expected_nodes()
{
    local deadline=$((SECONDS + READY_TIMEOUT_S)) node ready
    while (( SECONDS < deadline )); do
        ros2 node list > "${OUTPUT_DIR}/node-list.txt" 2>/dev/null || true
        ready=true
        for node in ${PRODUCT_NODES} ${FIXTURE_NODES}; do
            grep -Fxq "/${node}" "${OUTPUT_DIR}/node-list.txt" || ready=false
        done
        [[ "${ready}" == true ]] && return 0
        sleep 0.25
    done
    return 1
}

# Recorder processes must be up as ROS nodes before the scene starts publishing.
# Forking the process is not enough: rclpy import and DDS participant creation
# take a moment, and a recorder that is still initialising cannot subscribe.
wait_for_recorder_nodes()
{
    local deadline=$((SECONDS + READY_TIMEOUT_S)) index ready
    while (( SECONDS < deadline )); do
        ros2 node list > "${OUTPUT_DIR}/recorder-node-list.txt" 2>/dev/null || true
        ready=true
        for index in "${!RECORDER_NAMES[@]}"; do
            grep -Fxq "/graph_recorder_${RECORDER_NAMES[index]}" \
                "${OUTPUT_DIR}/recorder-node-list.txt" || ready=false
            process_is_alive "${ROLE_PIDS[RECORDER_ROLE_INDEXES[index]]}" || return 1
        done
        [[ "${ready}" == true ]] && {
            echo "recorder_nodes_ready_monotonic_ns=$(monotonic_ns)" \
                >> "${OUTPUT_DIR}/run-manifest.txt"
            # No settle delay here. One was tried, on the hypothesis that the
            # recorders needed to warm up before the scene started; measurement
            # over three runs then showed the shortfalls it was meant to fix
            # were not recording loss at all. Every run's revision ledger was
            # contiguous - what varied was upstream production (raw_scan
            # 193..199, the scanner being odometry-driven off a wall-clock
            # timer) plus the pose gate and C1 rejecting by contract.
            #
            # Detection replaces prevention: the reconciliation gate reads C2's
            # revision ledger, which the recorder cannot influence, so actual
            # loss is measured rather than guessed at and mitigated blindly.
            return 0
        }
        sleep 0.25
    done
    return 1
}

capture_node_parameters()
{
    local node
    for node in ${PRODUCT_NODES} ${FIXTURE_NODES}; do
        ros2 param dump "/${node}" > "${OUTPUT_DIR}/params-${node}.yaml" 2>&1 \
            || { invalidate "could not dump parameters of ${node}"; return 1; }
    done
}

declare_segments()
{
    local latch_argument=()
    [[ -n "${MOTION_LATCH_MONOTONIC_NS}" ]] \
        && latch_argument=(--latch-monotonic-ns "${MOTION_LATCH_MONOTONIC_NS}")
    local minimum_s
    minimum_s="$(python3 -c 'import sys; print(max(float(sys.argv[1]), float(sys.argv[2])) * 3)' \
        "${SAMPLE_INTERVAL_S}" "${PIDSTAT_INTERVAL_S}")"
    python3 "${GRAPH_GATES}" --report "${OUTPUT_DIR}/segments.txt" segments \
        "${T0_MONOTONIC_NS}" "${T1_MONOTONIC_NS}" "${latch_argument[@]}" \
        --drain-s "${TRANSPORT_DRAIN_S}" --minimum-duration-s "${minimum_s}" \
        --boundary-source observed_scanner_latch \
        >> "${OUTPUT_DIR}/gate.log" 2>&1 \
        || { invalidate "segment boundary derivation failed"; return 1; }
    cat "${OUTPUT_DIR}/segments.txt" >> "${OUTPUT_DIR}/run-manifest.txt"
    {
        echo "motion_latch_observed=$([[ -n "${MOTION_LATCH_MONOTONIC_NS}" ]] && echo true || echo false)"
        echo "trajectory_duration_s=${TRAJECTORY_DURATION_S}"
        echo "segment_drain_s=${TRANSPORT_DRAIN_S}"
    } >> "${OUTPUT_DIR}/run-manifest.txt"
}

# Count-based equivalence for a node under test that has no aggregate state
# topic (C1): the support recorder captured its outputs in-window, and the bag's
# own metadata carries the per-topic message count. In replay-loop the expected
# count and tolerance both scale by the number of cycles.
compare_recorded_output_count()
{
    if [[ ! "${REPLAY_OUTPUT_BASELINE_COUNT}" =~ ^[0-9]+$ ]]; then
        invalidate "count equivalence requires numeric ALIEN_GRAPH_REPLAY_OUTPUT_BASELINE_COUNT"
        return 1
    fi
    local recorded expected tolerance delta
    recorded="$(python3 - "${OUTPUT_DIR}/bag-replay_out/metadata.yaml" "${REPLAY_OUTPUT_TOPIC}" <<'PY'
import sys
import yaml
with open(sys.argv[1], encoding="utf-8") as stream:
    metadata = yaml.safe_load(stream)["rosbag2_bagfile_information"]
for entry in metadata["topics_with_message_count"]:
    if entry["topic_metadata"]["name"] == sys.argv[2]:
        print(entry["message_count"])
        break
PY
)"
    if [[ ! "${recorded}" =~ ^[0-9]+$ ]]; then
        invalidate "recorded output count for ${REPLAY_OUTPUT_TOPIC} is unreadable"
        return 1
    fi
    expected="${REPLAY_OUTPUT_BASELINE_COUNT}"
    tolerance="${EQUIVALENCE_COUNT_TOLERANCE}"
    if [[ "${MODE}" == replay-loop ]]; then
        if [[ ! "${CYCLE_COPIES}" =~ ^[0-9]+$ ]]; then
            invalidate "replay-loop count equivalence requires ALIEN_GRAPH_CYCLE_COPIES"
            return 1
        fi
        expected=$(( expected * CYCLE_COPIES ))
        tolerance=$(( CYCLE_REVISION_TOLERANCE * CYCLE_COPIES ))
    fi
    {
        echo "output_count_topic=${REPLAY_OUTPUT_TOPIC}"
        echo "output_count_recorded=${recorded}"
        echo "output_count_expected=${expected}"
        echo "output_count_tolerance=${tolerance}"
    } > "${OUTPUT_DIR}/replay-equivalence.txt"
    delta=$(( recorded - expected ))
    [[ ${delta} -lt 0 ]] && delta=$(( -delta ))
    if [[ ${delta} -gt ${tolerance} ]]; then
        echo "failure=recorded output count ${recorded} deviates from expected ${expected} by ${delta}" \
            >> "${OUTPUT_DIR}/replay-equivalence.txt"
        invalidate "recorded output count ${recorded} deviates from expected ${expected} by ${delta} (> ${tolerance})"
        return 1
    fi
    # Key name is the analyzer's contract (analyze-graph-profile.py).
    echo "replay_equivalent=true" >> "${OUTPUT_DIR}/replay-equivalence.txt"
}

compare_replay_equivalence()
{
    if [[ -n "${REPLAY_OUTPUT_TOPIC}" ]]; then
        compare_recorded_output_count
        return $?
    fi
    if [[ "${MODE}" == replay-loop ]]; then
        # Cycle equivalence for the leak long run. Each copy boundary teleports
        # the pose back to the start, which correctly triggers a map epoch
        # reset, so exact single-pass equivalence does not apply. Two checks
        # cover it instead:
        #   1. terminal_map_epoch == copies - every boundary produced exactly
        #      one reset, so every cycle actually ran;
        #   2. terminal_revision ~= single-pass baseline +/- tolerance - the
        #      last cycle did a full cycle's work (the recovery stability gate
        #      eats 1-4 frames at each boundary, measured 2026-08-02).
        if [[ -z "${CYCLE_COPIES}" || -z "${CYCLE_BASELINE_REVISION}" ]]; then
            invalidate "replay-loop requires ALIEN_GRAPH_CYCLE_COPIES and ALIEN_GRAPH_CYCLE_BASELINE_REVISION"
            return 1
        fi
        local terminal_epoch terminal_revision delta
        terminal_epoch="$(sed -n 's/^terminal_map_epoch=//p' "${OUTPUT_DIR}/terminal-state.txt")"
        terminal_revision="$(sed -n 's/^terminal_revision=//p' "${OUTPUT_DIR}/terminal-state.txt")"
        if [[ ! "${terminal_epoch}" =~ ^[0-9]+$ || ! "${terminal_revision}" =~ ^[0-9]+$ ]]; then
            invalidate "terminal state lacks numeric map_epoch/revision"
            return 1
        fi
        {
            echo "cycle_copies=${CYCLE_COPIES}"
            echo "cycle_baseline_revision=${CYCLE_BASELINE_REVISION}"
            echo "cycle_revision_tolerance=${CYCLE_REVISION_TOLERANCE}"
            echo "cycle_terminal_epoch=${terminal_epoch}"
            echo "cycle_terminal_revision=${terminal_revision}"
        } >> "${OUTPUT_DIR}/run-manifest.txt"
        if [[ "${terminal_epoch}" -ne "${CYCLE_COPIES}" ]]; then
            invalidate "terminal map_epoch ${terminal_epoch} != cycle count ${CYCLE_COPIES}: not every cycle ran and reset"
            return 1
        fi
        delta=$(( terminal_revision - CYCLE_BASELINE_REVISION ))
        [[ ${delta} -lt 0 ]] && delta=$(( -delta ))
        if [[ ${delta} -gt ${CYCLE_REVISION_TOLERANCE} ]]; then
            invalidate "last-cycle revision ${terminal_revision} deviates from baseline ${CYCLE_BASELINE_REVISION} by ${delta} (> ${CYCLE_REVISION_TOLERANCE})"
            return 1
        fi
        return 0
    fi
    if [[ -z "${EQUIVALENCE_BASELINE}" ]]; then
        invalidate "replay requires ALIEN_GRAPH_EQUIVALENCE_BASELINE from the direct run"
        return 1
    fi
    cp "${EQUIVALENCE_BASELINE}" "${OUTPUT_DIR}/equivalence-baseline.txt" \
        || { invalidate "equivalence baseline is unreadable"; return 1; }
    # v3 scope: counted equivalence, not oracle equivalence. The task PRD
    # explicitly replaces exact ground-truth migration with count gates, so the
    # comparison is terminal map_epoch (exact) plus terminal revision (within a
    # declared tolerance - the mapper's recovery stability gate consumes 0-3
    # observations at startup depending on the health/pose race, so replay and
    # direct runs may legitimately differ by a few commits).
    local base_epoch base_revision replay_epoch replay_revision delta
    base_epoch="$(sed -n 's/^terminal_map_epoch=//p' "${OUTPUT_DIR}/equivalence-baseline.txt")"
    base_revision="$(sed -n 's/^terminal_revision=//p' "${OUTPUT_DIR}/equivalence-baseline.txt")"
    replay_epoch="$(sed -n 's/^terminal_map_epoch=//p' "${OUTPUT_DIR}/terminal-state.txt")"
    replay_revision="$(sed -n 's/^terminal_revision=//p' "${OUTPUT_DIR}/terminal-state.txt")"
    if [[ ! "${base_epoch}" =~ ^[0-9]+$ || ! "${base_revision}" =~ ^[0-9]+$ \
        || ! "${replay_epoch}" =~ ^[0-9]+$ || ! "${replay_revision}" =~ ^[0-9]+$ ]]; then
        invalidate "terminal state or baseline lacks numeric map_epoch/revision"
        return 1
    fi
    {
        echo "equivalence_baseline_epoch=${base_epoch}"
        echo "equivalence_baseline_revision=${base_revision}"
        echo "equivalence_replay_epoch=${replay_epoch}"
        echo "equivalence_replay_revision=${replay_revision}"
        echo "equivalence_revision_tolerance=${EQUIVALENCE_COUNT_TOLERANCE}"
    } > "${OUTPUT_DIR}/replay-equivalence.txt"
    if [[ "${replay_epoch}" -ne "${base_epoch}" ]]; then
        echo "failure=replay terminal map_epoch ${replay_epoch} != direct ${base_epoch}" \
            >> "${OUTPUT_DIR}/replay-equivalence.txt"
        invalidate "replay terminal map_epoch ${replay_epoch} != direct ${base_epoch}"
        return 1
    fi
    delta=$(( replay_revision - base_revision ))
    [[ ${delta} -lt 0 ]] && delta=$(( -delta ))
    if [[ ${delta} -gt ${EQUIVALENCE_COUNT_TOLERANCE} ]]; then
        echo "failure=replay terminal revision ${replay_revision} deviates from direct ${base_revision} by ${delta}" \
            >> "${OUTPUT_DIR}/replay-equivalence.txt"
        invalidate "replay terminal revision ${replay_revision} deviates from direct ${base_revision} by ${delta} (> ${EQUIVALENCE_COUNT_TOLERANCE})"
        return 1
    fi
    # Key name is the analyzer's contract (analyze-graph-profile.py).
    echo "replay_equivalent=true" >> "${OUTPUT_DIR}/replay-equivalence.txt"
}

REPLAY_BAG="${ALIEN_GRAPH_REPLAY_BAG:-}"
REPLAY_TOPICS="${ALIEN_GRAPH_REPLAY_TOPICS:-}"
# For a node with no aggregate state topic (C1), equivalence is a count of its
# recorded outputs instead of a terminal state query:
#   REPLAY_RECORD_TOPICS - csv of topics a support recorder captures in-window
#   REPLAY_OUTPUT_TOPIC  - the recorded topic whose message count is compared
#   REPLAY_OUTPUT_BASELINE_COUNT - expected count (per cycle in replay-loop)
#   TERMINAL_STATE_MODE=none - skip the C2 state snapshot entirely
REPLAY_RECORD_TOPICS="${ALIEN_GRAPH_REPLAY_RECORD_TOPICS:-}"
REPLAY_OUTPUT_TOPIC="${ALIEN_GRAPH_REPLAY_OUTPUT_TOPIC:-}"
REPLAY_OUTPUT_BASELINE_COUNT="${ALIEN_GRAPH_REPLAY_OUTPUT_BASELINE_COUNT:-}"
TERMINAL_STATE_MODE="${ALIEN_GRAPH_TERMINAL_STATE:-state}"
# When the install under test is ASan-built, helper processes that load its
# shared libraries (bag play / recorders load typesupport, the terminal-state
# snapshot imports message modules) abort with "ASan runtime does not come
# first" unless the runtime is preloaded. Preloading must be TARGETED: a global
# LD_PRELOAD leaks LSan noise into every command substitution (measured: tar
# received "==NNN==Processing thread" lines as file names and provenance
# archiving failed). Helpers are not under test, so their leak detection is off.
HELPER_PRELOAD="${ALIEN_GRAPH_HELPER_PRELOAD:-}"
helper_env=()
if [[ -n "${HELPER_PRELOAD}" ]]; then
    helper_env=(env "LD_PRELOAD=${HELPER_PRELOAD}" "ASAN_OPTIONS=detect_leaks=0")
fi
REPLAY_NODES="${ALIEN_GRAPH_REPLAY_NODES:-}"
REPLAY_PARAMS="${ALIEN_GRAPH_REPLAY_PARAMS:-}"
REPLAY_RATE="${ALIEN_GRAPH_REPLAY_RATE:-}"
EQUIVALENCE_BASELINE="${ALIEN_GRAPH_EQUIVALENCE_BASELINE:-}"
CYCLE_COPIES="${ALIEN_GRAPH_CYCLE_COPIES:-}"
CYCLE_BASELINE_REVISION="${ALIEN_GRAPH_CYCLE_BASELINE_REVISION:-}"
CYCLE_REVISION_TOLERANCE="${ALIEN_GRAPH_CYCLE_REVISION_TOLERANCE:-5}"

if [[ "${MODE}" == replay || "${MODE}" == replay-loop ]]; then
    if [[ -z "${REPLAY_BAG}" ]]; then
        invalidate "ALIEN_GRAPH_REPLAY_BAG must name one or more existing bag directories"
        exit 1
    fi
    for specification in ${REPLAY_BAG}; do
        if [[ ! -d "${specification}" ]]; then
            invalidate "replay bag directory does not exist: ${specification}"
            exit 1
        fi
    done
    if [[ -z "${REPLAY_NODES}" ]]; then
        invalidate "ALIEN_GRAPH_REPLAY_NODES must list the nodes under test"
        exit 1
    fi
    if [[ -n "${REPLAY_PARAMS}" && ! -r "${REPLAY_PARAMS}" ]]; then
        invalidate "replay parameter file is unreadable: ${REPLAY_PARAMS}"
        exit 1
    fi
    PRODUCT_NODES=""
    for specification in ${REPLAY_NODES}; do
        PRODUCT_NODES="${PRODUCT_NODES} ${specification##*:}"
    done
    FIXTURE_NODES=""
    {
        echo "replay_nodes=${REPLAY_NODES}"
        echo "replay_params=${REPLAY_PARAMS}"
        echo "measured_nodes=${PRODUCT_NODES}"
    } >> "${OUTPUT_DIR}/run-manifest.txt"
fi

case "${MODE}" in
    scene-record)
        # Recorders first: they poll for topics that do not exist yet, so
        # starting them ahead of the scene is what guarantees full coverage
        # from the graph's first message. Each carries /tf_static so every bag
        # is self-contained for replay.
        # ONE recorder, not three. Three independent processes race each other
        # through DDS initialisation, and whichever loses drops the head of its
        # stream: measured as bag-s2 missing raw_scan indexes 0..23 (the first
        # 2.30 s) while bag-s1 and bag-s3 were complete - proven to be recording
        # loss rather than upstream variation because 18 of those scans appeared
        # downstream as observations, so C1 had received what s2 never wrote.
        #
        # A single recorder has no one to race. The manual nine-topic recording
        # that established this orchestration used one recorder and lost
        # nothing. Replay selects what it needs with `ros2 bag play --topics`,
        # so one bag serves every cut point.
        start_recorder all \
            "${TOPIC_RAW_SCAN},${TOPIC_RELEASED_SCAN},${TOPIC_ODOMETRY},${TOPIC_OBSERVATIONS},${TOPIC_POSE},${TOPIC_HEALTH},${TOPIC_STATE},/tf,/tf_static" \
            || { invalidate "recorder did not start"; exit 1; }
        wait_for_recorder_nodes \
            || { invalidate "the recorder did not appear as a ROS node before the scene"; exit 1; }
        start_scene || { invalidate "scene did not start"; exit 1; }
        ;;
    replay|replay-loop)
        # The output recorder (if any) starts first for the same reason the
        # scene recorders do - discovery keeps polling until the topic exists,
        # so an early recorder waits rather than misses.
        if [[ -n "${REPLAY_RECORD_TOPICS}" ]]; then
            # Underscore, not hyphen: the name becomes part of a ROS node name,
            # which rejects anything beyond alphanumerics and '_'.
            start_recorder replay_out "${REPLAY_RECORD_TOPICS}" \
                || { invalidate "replay output recorder did not start"; exit 1; }
        fi
        start_replay_nodes || { invalidate "nodes under test did not start"; exit 1; }
        start_bag_play || { invalidate "bag play did not start"; exit 1; }
        ;;
esac

wait_for_expected_nodes \
    || { invalidate "expected nodes did not all appear in the ROS graph"; exit 1; }
if (( ${#RECORDER_NAMES[@]} > 0 )); then
    wait_for_recorder_subscriptions \
        || { invalidate "a recorder did not subscribe to every requested topic"; exit 1; }
fi
freeze_identities || { invalidate "could not freeze one PID per expected node"; exit 1; }

FROZEN_PIDS="$(frozen_pids)"
if [[ -z "${FROZEN_PIDS}" ]]; then
    invalidate "frozen identity file yielded no PIDs"
    exit 1
fi

# Load starts only now: everything above is setup whose cost must not land in
# the window. Anything slow that is not needed before the window - dumping node
# parameters in particular - is deliberately deferred, because the scene's useful
# load lasts only as long as the trajectory and every setup second eats into it.
case "${MODE}" in
    scene-record) resume_recorders || exit 1 ;;
    replay|replay-loop) resume_bag_play || exit 1 ;;
esac
start_pidstat "${FROZEN_PIDS}" || { invalidate "pidstat did not start"; exit 1; }
start_graph_sampler || { invalidate "graph sampler did not start"; exit 1; }

T0_MONOTONIC_NS="$(monotonic_ns)"
if (( PIDSTAT_START_MONOTONIC_NS > T0_MONOTONIC_NS \
    || T0_MONOTONIC_NS - PIDSTAT_START_MONOTONIC_NS > PIDSTAT_BRACKET_MARGIN_NS )); then
    invalidate "pidstat did not start within two seconds before the formal window"
    exit 1
fi
WINDOW_OK=true
monitor_window || WINDOW_OK=false
T1_MONOTONIC_NS="$(monotonic_ns)"
PIDSTAT_STOP_MONOTONIC_NS="${T1_MONOTONIC_NS}"
{
    echo "t0_monotonic_ns=${T0_MONOTONIC_NS}"
    echo "t1_monotonic_ns=${T1_MONOTONIC_NS}"
    echo "pidstat_start_margin_ns=$((T0_MONOTONIC_NS - PIDSTAT_START_MONOTONIC_NS))"
} >> "${OUTPUT_DIR}/run-manifest.txt"
[[ "${WINDOW_OK}" == true ]] || exit 1

stop_role "$(role_index graph_sampler)" TERM 15
PIDSTAT_STOP_MONOTONIC_NS="$(monotonic_ns)"
stop_role "$(role_index pidstat)" TERM 15
echo "pidstat_stop_margin_ns=$((PIDSTAT_STOP_MONOTONIC_NS - T1_MONOTONIC_NS))" \
    >> "${OUTPUT_DIR}/run-manifest.txt"
if ! validate_pidstat_bracket "${PIDSTAT_START_MONOTONIC_NS}" "${T0_MONOTONIC_NS}" \
    "${T1_MONOTONIC_NS}" "${PIDSTAT_STOP_MONOTONIC_NS}" "${PIDSTAT_BRACKET_MARGIN_NS}"; then
    invalidate "pidstat did not tightly bracket the formal window"
fi

# Node parameters are constant for the life of a node, so they are captured here
# rather than before the window, where the six dumps cost more than the moving
# phase can spare.
capture_node_parameters || true

if [[ "${MODE}" == scene-record ]]; then
    stop_recorders_and_verify_bags || true
else
    stop_ros2_cli_role "$(role_index bag_play)" 10 30
fi
if [[ "${TERMINAL_STATE_MODE}" == none ]]; then
    # A node without an aggregate state topic (C1) has nothing to snapshot;
    # equivalence for such runs is count-based via the output recorder.
    echo "terminal_state_skipped=no_state_topic_for_node_under_test" \
        >> "${OUTPUT_DIR}/run-manifest.txt"
else
    capture_terminal_state || invalidate "terminal map state was not captured after the window"
    if [[ ! -s "${OUTPUT_DIR}/terminal-state.txt" ]]; then
        invalidate "terminal map state file is empty"
    fi
fi

if [[ "${MODE}" == scene-record ]]; then
    SCENE_INDEX="$(role_index scene)"
    SCENE_PGID="${ROLE_PGIDS[SCENE_INDEX]}"
    stop_ros2_cli_role "${SCENE_INDEX}" "${SCENE_EXIT_TIMEOUT_S}" 30
    assert_group_is_empty scene "${SCENE_PGID}" || true
else
    for specification in ${REPLAY_NODES}; do
        stop_role "$(role_index "node_${specification##*:}")" INT 30
    done
    # The output recorder outlives the nodes under test so trailing outputs are
    # captured; it then gets the same graceful-exit-and-verify treatment as the
    # scene recorders.
    if (( ${#RECORDER_NAMES[@]} > 0 )); then
        stop_recorders_and_verify_bags || true
    fi
fi

declare_segments || true
if [[ "${MODE}" == replay || "${MODE}" == replay-loop ]]; then
    compare_replay_equivalence || true
fi

if [[ "${FORCED_STOP}" == false && "${ROLE_EXIT_FAILURE}" == false ]]; then
    NORMAL_COMPLETION=true
fi
{
    echo "valid=${VALID}"
    echo "normal_completion=${NORMAL_COMPLETION}"
} >> "${OUTPUT_DIR}/run-manifest.txt"
if [[ "${VALID}" == true && "${NORMAL_COMPLETION}" == true ]]; then
    python3 "${GRAPH_ANALYZER}" "${OUTPUT_DIR}" \
        --output "${OUTPUT_DIR}/analysis-summary.json" \
        > "${OUTPUT_DIR}/analysis-summary.stdout.json" \
        2> "${OUTPUT_DIR}/analysis-summary.stderr.log" \
        || invalidate "graph analyzer rejected the completed raw evidence"
fi

exit 0
