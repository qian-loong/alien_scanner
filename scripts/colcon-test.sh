#!/bin/bash
# colcon test --packages-select <package_name> --ctest-args tests
set -eo pipefail

PKG="${1:?usage: colcon-test.sh <package_name>}"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "${ROOT}/ws"

if [[ -f install/setup.bash ]]; then
  source install/setup.bash
fi

if [[ -n "${ROS2_COLCON_TEST_VERBOSE:-}" ]]; then
  colcon test --packages-select "${PKG}" --ctest-args tests --event-handlers console_direct+
  exit $?
fi

colcon test --packages-select "${PKG}" --ctest-args tests >/dev/null 2>&1
_exit=$?

_log="log/latest_test/${PKG}/stdout_stderr.log"
if [[ ! -f "${_log}" ]]; then
  echo "(no test log: ${_log})"
  exit "${_exit}"
fi

echo ""

while IFS= read -r _line; do
  [[ "${_line}" =~ ^([0-9]+/[0-9]+)\ Test\ #[0-9]+:\ ([^[:space:]]+) ]] || continue
  _n="${BASH_REMATCH[1]}"
  _name="${BASH_REMATCH[2]}"
  _status=""
  _sec=""

  if [[ "${_line}" =~ Passed[[:space:]]+([0-9.]+)[[:space:]]+sec ]]; then
    _status="Passed"
    _sec="${BASH_REMATCH[1]}"
  elif [[ "${_line}" =~ \*\*\*Failed[[:space:]]+([0-9.]+)[[:space:]]+sec ]]; then
    _status="Failed"
    _sec="${BASH_REMATCH[1]}"
  elif [[ "${_line}" =~ [[:space:]]Failed[[:space:]]+([0-9.]+)[[:space:]]+sec ]]; then
    _status="Failed"
    _sec="${BASH_REMATCH[1]}"
  elif [[ "${_line}" == *"***Failed"* ]] || [[ "${_line}" == *" Failed "* ]]; then
    _status="Failed"
    _sec="?"
  else
    continue
  fi

  printf '%s Test: %s\nTime = %6s sec\tTest %s.\n\n' "${_n}" "${_name}" "${_sec}" "${_status}"
done < <(grep -E '^[0-9]+/[0-9]+ Test #[0-9]+:' "${_log}" || true)

for _summary_log in \
  "${_log}" \
  "build/${PKG}/Testing/Temporary/LastTest.log"; do
  [[ -f "${_summary_log}" ]] || continue
  _summary="$(grep -E '[0-9]+% tests passed|[0-9]+ tests failed out of' "${_summary_log}" 2>/dev/null | tail -1 || true)"
  if [[ -n "${_summary}" ]]; then
    echo "${_summary}"
    break
  fi
done

exit "${_exit}"
