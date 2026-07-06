# alien-scanner

多无人机进入地下洞穴扫描、实时构建三维地形的 ROS 2 Jazzy 仿真项目（无真实硬件，脚本仿真）。

## 环境

- Docker Dev Container：镜像 `alien-scanner-jazzy:latest`，容器 `alien-scanner-dev`，ROS 2 Jazzy。
- Windows `D:\WorkDir\alien-scanner` ←→ 容器 `/workspaces/alien-scanner`（bind mount）。
- colcon 工作区：`ws/`，源码包在 `ws/src/`。
- GUI（RViz2）经 VcXsrv 转发，`DISPLAY=host.docker.internal:0.0`。

## 构建

```bash
cd /workspaces/alien-scanner/ws
colcon build --symlink-install
source install/setup.bash
```

## 文档

- 工程约定（分层 / 接口 / 测试边界）：见 `AGENTS.md`
- 分阶段实施计划：见 `docs/xenomorph-scanner-plan.md`

## Phase 1 快速启动（cave_world）

```bash
cd /workspaces/alien-scanner/ws && source install/setup.bash
ros2 launch cave_world cave_world_launch.py
```

- **基础地图**（默认）：`cave_mode:=tree`，`seed:=42`，`tree.loop_bulge:=12`，`tree.loop_direct_length:=16`
- **发布话题**：`/cave/points`（`sensor_msgs/PointCloud2`，`frame_id=map`，`TRANSIENT_LOCAL`）
- **参数写法**：`ros2 launch cave_world cave_world_launch.py 参数名:=值`（勿用 `--ros-args -p`）
- 完整参数表、拓扑说明、验收命令：见 `docs/xenomorph-scanner-plan.md` §6 Phase 1
