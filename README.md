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

- 工程约定（分层 / 接口 / 测试边界）：[`AGENTS.md`](AGENTS.md)
- 总体规划（目标 / 架构 / 跨 Phase 契约）：[`docs/xenomorph-scanner-plan.md`](docs/xenomorph-scanner-plan.md)
- **分步实施细节：**
  - Phase 1：[`docs/phases/phase-01-cave-world.md`](docs/phases/phase-01-cave-world.md)
  - Phase 2：[`docs/phases/phase-02-drone-scanner.md`](docs/phases/phase-02-drone-scanner.md)
  - Phase 3：[`docs/phases/phase-03-swarm.md`](docs/phases/phase-03-swarm.md)

## Phase 1 快速启动（cave_world）

```bash
cd /workspaces/alien-scanner/ws && source install/setup.bash
ros2 launch cave_world cave_world_launch.py
```

- **基础地图**（默认）：`cave_mode:=tree`，`seed:=42`，`tree.loop_bulge:=12`，`tree.loop_direct_length:=16`
- **发布话题**：`/cave/points`（`sensor_msgs/PointCloud2`，`frame_id=map`，`TRANSIENT_LOCAL`）
- **参数写法**：`ros2 launch cave_world cave_world_launch.py 参数名:=值`（勿用 `--ros-args -p`）
- 完整参数表、拓扑说明、验收命令：见 [`docs/phases/phase-01-cave-world.md`](docs/phases/phase-01-cave-world.md)

## Phase 2 快速启动（drone_scanner，已完成）

```bash
cd /workspaces/alien-scanner/ws && source install/setup.bash
ros2 launch drone_scanner fake_lidar_launch.py

# 纯探索视角（关闭洞穴真值）
ros2 launch drone_scanner fake_lidar_launch.py show_cave:=false
```

- **感知：** 3D **垂直 360° 环**（非 2D 水平扫描）；机头沿 **map +X**，环在 **YZ 平面**（`ring_pitch=0`）
- **关键话题：** `/drone_0/odom`、`/drone_0/points`、`/drone_0/cloud_map`
- 完整分步、坐标约定、验收：见 [`docs/phases/phase-02-drone-scanner.md`](docs/phases/phase-02-drone-scanner.md)

## Phase 3（进行中：3-1～3-7 已完成）

- **未知探索**（规划不读洞穴真值）；**俯仰垂直环**；**OctoMap**；多机任务调度；`/global_map`
- 当前入口：`ros2 launch swarm_controller multi_drone_exploration.launch.py`
- 当前下一步：3-8 `/global_map` 融合
- 分步与验收：[`docs/phases/phase-03-swarm.md`](docs/phases/phase-03-swarm.md)
