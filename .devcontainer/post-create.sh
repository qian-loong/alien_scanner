#!/bin/bash
set -eo pipefail

grep -q 'source /opt/ros/jazzy/setup.bash' /root/.bashrc 2>/dev/null || \
  echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc

grep -q 'source /workspaces/alien-scanner/ws/install/setup.bash' /root/.bashrc 2>/dev/null || \
  echo '[[ -f /workspaces/alien-scanner/ws/install/setup.bash ]] && source /workspaces/alien-scanner/ws/install/setup.bash' >> /root/.bashrc

grep -q 'RCUTILS_CONSOLE_OUTPUT_FORMAT' /root/.bashrc 2>/dev/null || \
  echo 'export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {date_time_with_ms}] [{name}]: {message}"' >> /root/.bashrc

mkdir -p /workspaces/alien-scanner/ws/src

# CLion colcon 目标 / VcXsrv 无关：给仓库脚本加执行位
chmod +x /workspaces/alien-scanner/scripts/*.sh 2>/dev/null || true

source /opt/ros/jazzy/setup.bash
if [[ -f /workspaces/alien-scanner/ws/install/setup.bash ]]; then
  source /workspaces/alien-scanner/ws/install/setup.bash || true
fi

# 生成 CLion Python Run/Debug 用的 scripts/ros2-clion.env（overlay 变化后可手动重跑）
if [[ -f /workspaces/alien-scanner/scripts/update-ros2-clion-env.sh ]]; then
  bash /workspaces/alien-scanner/scripts/update-ros2-clion-env.sh || true
fi

echo "Dev Container ready: ROS 2 Jazzy, workspace=/workspaces/alien-scanner, colcon ws=/workspaces/alien-scanner/ws"
