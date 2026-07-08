# AGENTS.md — 本仓库 AI 协作与工程约定

> 面向在本仓库执行任务的 AI 与开发者。**动手写 C++ / 建包 / 写测试前，先读本文件。**
> 项目背景与分阶段计划见 `docs/xenomorph-scanner-plan.md`；**各 Phase 分步细节**见 `docs/phases/`（如 [`phase-01-cave-world.md`](docs/phases/phase-01-cave-world.md)、[`phase-02-drone-scanner.md`](docs/phases/phase-02-drone-scanner.md)）。

---

## 0. 环境与工作区

- 开发在 **Docker 容器 `alien-scanner-dev`（镜像 `alien-scanner-jazzy:latest`，ROS 2 Jazzy）** 内进行。
- Windows `D:\WorkDir\alien-scanner` ←→ 容器 `/workspaces/alien-scanner`（bind mount）；colcon `build/install/log` 用命名卷（`alien_build` / `alien_install` / `alien_log`）。
- colcon 工作区：`/workspaces/alien-scanner/ws`，源码包在 `ws/src/`。
- 构建：`cd /workspaces/alien-scanner/ws && colcon build --symlink-install && source install/setup.bash`
- GUI（RViz2）经 VcXsrv 转发，`DISPLAY=host.docker.internal:0.0`。

### 0.1 CMake 与 CLion（建包 / 改 C++ 前）

| 场景 | 文档 |
|------|------|
| 新建或修改 `ament_cmake` 包（C++ 标准、库 export、链接） | [`docs/ament-cmake-conventions.md`](docs/ament-cmake-conventions.md) |
| CLion 索引、`colcon_build_*`、CMake 缓存选项 | [`docs/clion-cpp-cmake.md`](docs/clion-cpp-cmake.md) |

---

## 1. 语言约定

- **主体语言：C++（`rclcpp` + `ament_cmake`）。** 运行期节点一律 C++。
- **Python 仅用于「特殊 / 外部数据资产」例外**：如离线预生成/导入复杂点云或网格（`.ply`/`.obj`）、或某算法只有成熟 Python 生态（如 Open3D 重建）时。默认路径不引入 Python 运行期节点。
- **launch 文件优先用 XML（`.launch.xml`）**，保持非 Python 主体；确需复杂逻辑再用 Python launch。

---

## 2. 分层：算法库 + 薄节点（最重要）

**每个包 = 一个「无 ROS 依赖的算法库」+ 一个「薄 rclcpp 节点」。**

```
<pkg>/
├── include/<pkg>/…     # 纯 C++ 算法 / 接口，禁止 include rclcpp
├── src/…               # 算法实现（编成 add_library）
└── src/<pkg>_node.cpp  # 薄节点：仅 参数读取 / 消息转换 / 收发；调用算法库
```

- 算法库**不依赖 ROS** → 可单元测试、可被其他包直接链接复用。
- 节点是**唯一**接触 rclcpp / 消息类型的地方。
- 复用优先靠**链接算法库**（同进程、C++），而非拆成额外话题/服务。

---

## 3. 接口（抽象）使用约定

### 3.1 何时该建接口（满足任一即可）

1. 会实验 / 切换**多种算法或数据源**；
2. 需要在无 ROS / 无真实数据时 **mock 依赖**做单测（且该依赖昂贵）；
3. 需要**运行时或配置**切换实现。

三者皆否（只有一种实现、可预见无第二种、也不需 mock）→ **写具体类，不要抽象**（YAGNI）。

### 3.2 铁律

- **不要为了测试给「被测单元自己」套接口**；直接调用它即可。
- 接口只加在**难以在测试中使用的「依赖」**上（依赖倒置），且该依赖确实昂贵；依赖便宜就直接喂真数据。
- **接口的正当性来自「会有第二实现 / 运行时切换」**；测试便利是附赠，不能作为唯一动机。
- **先具体后抽象**：不确定是否有第二实现时先写具体类，出现第二实现再提取接口。

### 3.3 C++ 抽象接口 vs ROS 接口

| 场景 | 用什么 |
|------|--------|
| 同进程、同语言（C++）复用/替换算法 | **C++ 抽象接口**（纯虚类） |
| 跨进程 / 跨语言 / 跨机通信 | **ROS 接口，优先标准消息**（PointCloud2 / Odometry / OccupancyGrid / Octomap / Marker …） |
| 请求-响应（一问一答） | ROS **srv**（仅在确有 RPC 边界时） |
| 长任务 + 反馈 + 可取消 | ROS **action** |

自定义 ROS 接口（`.msg/.srv/.action`）要克制；确需时放独立 `*_interfaces`（`rosidl`）包。

### 3.4 命名与写法

- **C++ 命名空间：PascalCase**（如 `namespace CaveWorld { ... }`）；**ROS 包名 / include 路径**仍用 snake_case（如包 `cave_world`、`#include "cave_world/ICaveField.hpp"`）。
- **类名 / 文件名：PascalCase**（如 `ProceduralCaveField.hpp`、`ProceduralCaveField.cpp`）。
- **接口类 + 头文件：`IXxx`**，头文件与类同名（如 `ICaveField.hpp` 定义 `class ICaveField`）；纯虚 + `virtual ~IXxx() = default;`。
- **实现类**：`ProceduralXxx` / `ConcreteXxx`（如 `ProceduralCaveField : public ICaveField`）。
- 接口放 `include/<pkg>/`，实现放 `src/`；接口与算法库**保持 ROS-free**。
- **依赖注入**：实现通过**构造函数**接收依赖接口（如 `FakeLidar(std::shared_ptr<ICaveField>)`）。

### 3.5 本项目接口决策（快速索引）

| 组件 | 接口？ | 形式 | 说明 |
|------|--------|------|------|
| 洞穴几何（P1） | ✅ | C++ `ICaveField` + `ProceduralCaveField` / `TreeCaveField` | 数据源会变；P2 射线采样复用 |
| 飞行轨迹（P2） | ✅ | C++ `ITrajectory` | 会试多种路径 |
| fake_lidar（P2） | ⬜ 自身不抽象 | 具体类，注入 `ICaveField` | **YZ 垂直 360° 环**；进洞 **map +X**；见 plan §6 Phase 2 §2.0 |
| SLAM / 建图 | ❌ | 不用 2D slam_toolbox 作主路径 | Phase 2 **3D 扫描累积**；Phase 3 **OctoMap 融合** |
| 探索策略（P3） | ✅ | C++ `IExplorationStrategy` | 典型策略切换点 |
| 地图融合（P3） | 🔸 | `IMapMerger`，先具体留边界 | 有第二实现可能 |
| 网格重建（P4） | 🔸 | `IMeshReconstructor`，先具体留边界 | 多重建法时再抽 |
| 消息转换 / 发布 | ❌ | 写在节点里 | 纯管道，抽象无收益 |

图例：✅现在就做 / 🔸留边界需要时再抽 / ⬜靠注入不必自身抽象 / ❌不要。

---

## 4. 测试边界

### 4.1 做单元测试（`ament_cmake_gtest`，追求高覆盖）

- **只测无 ROS 的算法库**：几何 / 轨迹 / 探索策略 / 融合 / 重建等纯逻辑。
- 依赖仅是**数据**时，直接构造输入调用；**无需 mock**。
- 被测单元依赖**昂贵外部协作者**时，才注入该依赖的接口的假实现（如给 `fake_lidar` 注入解析可算的假 `ICaveField`）。

### 4.2 不做单元测试（用集成/手动覆盖）

- rclcpp 节点管道、收发、QoS、TF 广播接线；
- 第三方节点（`slam_toolbox` / `octomap_server` / `rviz2`）与第三方库（`FastNoiseLite`）；
- 可视化外观。

### 4.3 集成测试与目检（单独一层）

- ROS 接线用 **`launch_testing`**（`ament_add_pytest_test`）验证：话题在发、TF 树合法、频率达标。
- 可视化效果靠**手动 RViz2 目检**。

### 4.4 可测性杠杆

- **优先「纯函数 + 固定 `seed`」获得确定性**，其次才考虑接口/mock。含随机的生成器必须接受 `seed` 参数以便断言。

---

## 5. 提交与协作

### 5.0 铁律：无明确指令不得提交

- **仅在用户明确要求时**才执行 `git add` / `git commit` / `git push`；完成实现、测试通过、或改动已就绪，**均不构成**提交许可。
- 用户若说「先做 A 再提交文档」「文档提交一次后实施 B」，**只对字面提到的对象提交**（例：只提交文档，**不得**顺带提交刚完成的代码）。
- Phase 内分步 commit（§5.1）是**用户发起提交时**应遵循的 message/粒度约定，**不是** AI 在每步完成后自动 commit 的授权。
- 改动完成后：汇报 diff 与验证结果，**询问或等待**用户是否提交；用户未说「提交 / commit」前，保持工作区改动未 commit 即可。
- 不修改 `git config`；不使用交互式 git 命令。
- 保持改动聚焦；文档类改动优先用定向编辑，避免整文件重写。

### 5.1 Git：按 Phase 提交

- **`main` 上每个 Phase 最终仅 1 个合并提交**（Phase 验收通过后对阶段分支做 squash merge）。
- **阶段开发在独立分支**上进行，命名：`phase/N-简短描述`（例：`phase/2-drone-scanner`）。
- **用户明确要求提交时**，Phase 内每完成计划中的一步，创建 1 个 commit；message 格式：`phaseN(stepK): 简短说明`（例：`phase2(step1): add LineTrajectory`）。
- Phase 验收完成前**不在 `main` 上堆多个阶段内 commit**；步骤历史保留在阶段分支，便于逐步 diff / revert。
- 需要回到某步时，在阶段分支上用 commit hash 或可选 tag（例：`phase2-step1`），**不要**依赖长期 `commit --amend` 链代替步骤记录。
- 推送：阶段分支可推送备份；**合并进 `main` 在用户确认 Phase 验收后进行**。
