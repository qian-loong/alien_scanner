#!/bin/bash
# colcon build --packages-select <package_name>
# CMake target 调用时设 ROS2_COLCON_QUIET=1：适配 CLion 构建窗口（非真实 TTY）
set -eo pipefail

PKG="${1:?usage: colcon-build.sh <package_name>}"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "${ROOT}/ws"

if [[ -f install/setup.bash ]]; then
  source install/setup.bash
fi

_ros2_print_env_banner() {
  echo "=== colcon build: ${PKG} ==="
  echo "ROS_DISTRO=${ROS_DISTRO:-<unset>}"
  echo "ROS_VERSION=${ROS_VERSION:-<unset>}"
  echo "ROS_PYTHON_VERSION=${ROS_PYTHON_VERSION:-<unset>}"
  echo "AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-<unset>}"
  echo "CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH:-<unset>}"
  echo "COLCON_CURRENT_PREFIX=${COLCON_CURRENT_PREFIX:-<unset>}"
  echo "Workspace=${ROOT}/ws"
  echo "=================================="
}

# ROS2_COLCON_SHOW_ENV=1 或 IDE quiet 模式：打印已 source 的环境（便于核对 CLion 构建环境）
if [[ -n "${ROS2_COLCON_SHOW_ENV:-}" ]] || [[ -n "${ROS2_COLCON_QUIET:-}" ]]; then
  _ros2_print_env_banner
fi

COLCON_ARGS=(
  build
  --packages-select "${PKG}"
  --symlink-install
)

if [[ -n "${ROS2_COLCON_QUIET:-}" ]] || [[ ! -t 1 ]]; then
  export TERM=dumb
fi


if [[ -n "${ROS2_COLCON_QUIET:-}" ]]; then
  colcon "${COLCON_ARGS[@]}" 2>&1
  _exit="${PIPESTATUS[0]}"
  echo ""
  exit "${_exit}"
else
  exec colcon "${COLCON_ARGS[@]}"
fi
