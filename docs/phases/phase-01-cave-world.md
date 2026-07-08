# Phase 1：洞穴点云生成器（cave_world）

> **状态：** ✅ 已完成（`main`）  
> **上级摘要：** [`docs/xenomorph-scanner-plan.md`](../xenomorph-scanner-plan.md) §6 Phase 1  
> **工程约定：** [`AGENTS.md`](../../AGENTS.md)

---

## 目标与产出

**目标：** 生成并可视化静态地下洞穴点云，作为后续 LiDAR 采样的「真实环境」体积函数。

**产出：**

- RViz2 中看到网状分叉洞穴（默认）或经典 Y 字隧道（回退模式）的 3D 点云
- 话题 `/cave/points`（`sensor_msgs/PointCloud2`），`frame_id = map`，`TRANSIENT_LOCAL`（latched）发布
- `ICaveField` 接口 + `isSolid` / `raycast` / `sampleSurface`，供 Phase 2 `fake_lidar` 同进程链接复用

**语言约定：** 主体 C++（`rclcpp` + `ament_cmake`）；launch 使用 Python；运行期节点是纯 C++。

---

## 1.1 包结构

```
ws/src/cave_world/
├── CMakeLists.txt              # cave_geometry 静态库 + cave_publisher 可执行
├── package.xml
├── include/cave_world/
│   ├── ICaveField.hpp          # 抽象洞穴场（ROS-free）
│   ├── ProceduralCaveField.hpp # 经典对称 Y 字（cave_mode:=y）
│   ├── TreeCaveField.hpp       # 草图网状拓扑（cave_mode:=tree，默认）
│   └── CavePublisherNode.hpp
├── src/
│   ├── ProceduralCaveField.cpp # 中轴线 / 半径剖面 / 外表面过滤 / raycast
│   ├── TreeCaveField.cpp       # 7 条臂 + Bezier 外环 + 体积并集
│   ├── CavePublisherNode.cpp   # 薄节点：参数 → ICaveField → PointCloud2
│   └── CavePublisherMain.cpp
├── launch/
│   ├── cave_publisher_launch.py  # 仅 cave_publisher（可被 include）
│   └── cave_world_launch.py      # include + rviz2
├── config/
│   └── cave_world.rviz         # Fixed Frame=map，订阅 /cave/points，Z 轴着色
└── test/
    ├── TestProceduralCaveField.cpp
    └── TestTreeCaveField.cpp
```

**分层（遵循 `AGENTS.md`）：**

- 算法库 `cave_geometry`（`ProceduralCaveField` + `TreeCaveField`）**不含 rclcpp**。
- 节点 `cave_publisher` 持有 `std::unique_ptr<ICaveField>`，启动时 `sampleSurface()` 一次并缓存，定时重发。
- Phase 2 `fake_lidar` 将**链接 `cave_geometry` 并注入 `std::shared_ptr<ICaveField>`**，调用 `raycast()`。

> `CaveFieldFactory` 已编入 `cave_geometry` 并随 `ament_export_targets` 导出。

---

## 1.2 几何模式

| `cave_mode` | 实现类 | 说明 |
|-------------|--------|------|
| `tree`（**默认**） | `TreeCaveField` | 项目基础地图：1 入口、3 出口、1 外环 |
| `y` | `ProceduralCaveField` | 经典对称 Y 字，保留作回退/对照 |

**基础地图拓扑（`TreeCaveField`，已确认参数）：**

```text
入口 ──approach──► A ─┬─ 外环（Bezier 弧，相对 A→B 直连向左鼓出）──┐
                      ├─ 直连 A→B ───────────────────────────────► B ──► 出口1
                      └─ 右廊 A→C ──┬─► 出口2
                                    └─► 出口3
```

**进洞方向：** 接入段沿 **`map` +X**（`TreeCaveField` 接入臂 `(1,0,0)`）；默认轨迹机头沿 map +X（`yaw=0`）。

合并算法（两套实现共用思路）：

- `isSolid`：各臂折线管体**体积并集**（任一条臂内即空腔）
- `sampleSurface`：各臂独立环采样 + **外表面过滤**（沿外法向微移后仍非 solid 则丢弃内部假墙）

---

## 1.3 `ICaveField` 接口

```cpp
// include/cave_world/ICaveField.hpp
namespace CaveWorld {
struct Point3 { float x, y, z; };

class ICaveField {
public:
  virtual ~ICaveField() = default;
  virtual bool isSolid(float x, float y, float z) const = 0;
  virtual bool raycast(const Point3& origin, const Point3& dir,
                       float max_range, float& out_dist) const = 0;
  virtual std::vector<Point3> sampleSurface() const = 0;
};
}  // namespace CaveWorld
```

---

## 1.4 地图参数指定方式

**推荐：通过 launch 文件传参**

```bash
cd /workspaces/alien-scanner/ws && source install/setup.bash

# 基础地图（默认即此配置，无需额外参数）
ros2 launch cave_world cave_world_launch.py

# 显式写出基础地图三要素（与默认等价）
ros2 launch cave_world cave_world_launch.py \
  seed:=42 tree.loop_bulge:=12 tree.loop_direct_length:=16

# 微调示例
ros2 launch cave_world cave_world_launch.py \
  seed:=42 tree.loop_bulge:=14 density:=500

# 回退经典 Y 字
ros2 launch cave_world cave_world_launch.py cave_mode:=y seed:=7 length:=40
```

**语法规则：**

| 方式 | 写法 | 说明 |
|------|------|------|
| ✅ Launch 覆盖 | `ros2 launch ... param:=value` | **首选**；`tree.*` 带点号的参数名原样书写 |
| ✅ 多参数 | 空格分隔多个 `name:=value` | 布尔：`branch:=false`；整数：`seed:=42` |
| ❌ 避免 | `ros2 launch ... --ros-args -p` | 对 launch 文件**无效**，参数不会传到节点 |
| 备选 | `ros2 run cave_world cave_publisher --ros-args -p ...` | 仅单独跑节点时用；launch 未暴露的 `frame_id` / `topic` / `publish_rate` 只能走此路径 |

**参数生效范围：**

- `cave_mode:=tree` 时：`tree.*` 与下方「通用参数」生效；`length` / `branch_*` / `chamber_*`（Y 专用）被忽略。
- `cave_mode:=y` 时：Y 专用参数 + 通用参数生效；`tree.*` 被忽略。
- 修改几何参数后需**重启 launch**（点云在节点启动时生成并缓存）。

**Launch 拆分：**

| 文件 | 用途 |
|------|------|
| `cave_publisher_launch.py` | 仅启动 `cave_publisher`；Phase 2 通过 `IncludeLaunchDescription` 复用 |
| `cave_world_launch.py` | include `cave_publisher_launch.py` + 启动 RViz2 |

---

## 1.5 参数表

**基础地图默认值（`cave_mode:=tree`）— 后续 Phase 2/3 以此为基准：**

| 参数 | 默认 | 说明 |
|------|------|------|
| `seed` | `42` | 形状确定性种子（`asymmetry` 扰动） |
| `tree.loop_direct_length` | `16.0` | A→B 直连长度 (m) |
| `tree.loop_bulge` | `12.0` | 外环相对 A→B 中点左侧鼓出 (m) |

**通用参数（两种模式共用）：**

| 参数 | 默认 | 说明 |
|------|------|------|
| `cave_mode` | `tree` | `tree` \| `y` |
| `seed` | `42` | 确定性种子 |
| `base_radius` | `2.5` | 管体基础半径 (m) |
| `n_segments` | `200` | 中轴线分段数（tree 接入段；Y 每条臂） |
| `density` | `400` | 表面采样密度（越大点越多） |
| `noise_scale` | `0.4` | 洞壁噪声强度 |

**`tree.*` 参数（仅 `cave_mode:=tree`）：**

| 参数 | 默认 | 说明 |
|------|------|------|
| `tree.approach_length` | `12.0` | 入口 → 分叉点 A (m) |
| `tree.loop_yaw` | `0.50` | A→B 方向相对入口切线偏角 (rad) |
| `tree.loop_direct_length` | `16.0` | A→B 直连长度 (m) |
| `tree.loop_bulge` | `12.0` | 外环左侧鼓出 (m) |
| `tree.exit1_length` | `14.0` | B → 出口1 (m) |
| `tree.right_yaw` | `-0.12` | A→C 右廊偏角 (rad) |
| `tree.right_corridor_length` | `10.0` | A→C 长度 (m) |
| `tree.exit_yaw_spread` | `0.35` | 出口2/3 展开半角 (rad) |
| `tree.exit_arm_length` | `14.0` | 出口2/3 长度 (m) |
| `tree.vertical_step` | `-0.10` | 各臂 pitch，负=下沉 (rad) |
| `tree.asymmetry` | `0.22` | seed 驱动的角度/长度扰动 [0,1] |
| `tree.chamber_on_approach` | `false` | 接入段溶洞（默认关，避免入口膨大） |
| `tree.chamber_at` | `0.55` | 溶洞轴向位置（仅上项为 true 时） |
| `tree.chamber_scale` | `2.2` | 溶洞半径放大倍数 |

**Y 模式参数（仅 `cave_mode:=y`）：**

| 参数 | 默认 | 说明 |
|------|------|------|
| `length` | `40.0` | 入口 → junction 主干长度 (m) |
| `branch_length` | `20.0` | 每条岔臂长度 (m) |
| `branch` | `true` | 是否生成 Y 分叉 |
| `branch_angle` | `0.55` | 两岔臂半角 (rad) |
| `chamber_at` | `0.5` | 溶洞在主干上的比例 [0,1] |
| `chamber_scale` | `3.0` | 溶洞放大倍数 |

**发布相关（节点 `declare_parameter`，launch 未暴露，改默认值或 `ros2 run --ros-args -p`）：**

| 参数 | 默认 | 说明 |
|------|------|------|
| `frame_id` | `map` | 点云坐标系 |
| `topic` | `/cave/points` | 发布话题 |
| `publish_rate` | `1.0` | 重发频率 (Hz)；几何不变，仅刷新 stamp |

---

## 1.6 发布话题与 QoS

| 项目 | 值 |
|------|-----|
| **话题** | `/cave/points` |
| **消息类型** | `sensor_msgs/msg/PointCloud2` |
| **发布节点** | `cave_publisher`（`cave_world` 包） |
| **frame_id** | `map` |
| **字段** | `x`, `y`, `z`（float32，`is_dense=true`） |
| **QoS** | 深度 1 + **`TRANSIENT_LOCAL`**（latched）；晚订阅的 RViz2 也能立即收到 |
| **生成时机** | 节点启动时 `sampleSurface()` 一次；定时器按 `publish_rate` 重发同一缓存 |
| **典型点数** | 与 `density`、臂长度、拓扑相关，约 10⁵ 量级（视参数而定） |

**验证命令：**

```bash
ros2 topic list | grep cave
ros2 topic info /cave/points -v
ros2 topic hz /cave/points
ros2 topic echo /cave/points --once | head -20
```

**RViz2：** launch 自动加载 `config/cave_world.rviz`；Fixed Frame 须为 `map`，PointCloud2 显示订阅 `/cave/points`。

---

## 1.7 构建、测试与验收

```bash
cd /workspaces/alien-scanner/ws
colcon build --symlink-install --packages-select cave_world
source install/setup.bash

# 单元测试（算法库，无 ROS）
colcon test --packages-select cave_world --event-handlers console_direct+
colcon test-result --verbose

# 集成测试（launch_testing）：/cave/points 收发、frame_id=map、TRANSIENT_LOCAL
# 含于上述 colcon test，标签 launch_test

# 可视化验收
ros2 launch cave_world cave_world_launch.py
```

**通过标准（已满足）：**

- RViz2 可见网状分叉洞穴，可旋转/缩放
- `/cave/points` 持续发布，`frame_id=map`，新开 RViz 能立即显示
- 修改 `density` / `tree.loop_bulge` 等参数后重启，点云随之变化
- `seed` 固定时形状可复现（单测覆盖）

**任务清单：**

- [x] 创建 `cave_world` C++ 包（`ament_cmake`）
- [x] `ICaveField.hpp` 抽象接口
- [x] `ProceduralCaveField` — 经典 Y + `isSolid` / `raycast` / `sampleSurface`
- [x] `TreeCaveField` — 草图拓扑 + 外环 + 外表面过滤
- [x] `CavePublisherNode` — 参数、`TRANSIENT_LOCAL`、缓存点云、定时发布
- [x] `cave_publisher_launch.py` + `cave_world_launch.py` + `cave_world.rviz`
- [x] gtest：`TestProceduralCaveField`、`TestTreeCaveField`
- [x] 基础地图默认值：`seed=42`, `tree.loop_bulge=12`, `tree.loop_direct_length=16`
- [x] `launch_testing`：`test_cave_publisher_integration.py`（`/cave/points` 收发、`frame_id=map`）
- [ ] （可选）`scripts/gen_cave_ply.py` 离线导出 `.ply`

---

## Phase 2 衔接

`fake_lidar` 链接 `cave_geometry`，构造注入与 `cave_publisher` 相同配置的 `ICaveField` 实例，按位姿调用 `raycast()`；静态 `/cave/points` 仍可用于 RViz 对照真值。感知为 **3D 垂直环扫描**（非 2D 水平），详见 [`phase-02-drone-scanner.md`](phase-02-drone-scanner.md)。
