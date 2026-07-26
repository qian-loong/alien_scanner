# CMake 拓扑与源码索引实施计划

## 实施清单

1. 修改根 `CMakeLists.txt`：用 `colcon list --topological-order --paths-only` 获取有序包目录；为 `ROS2_PACKAGES_SELECT` 使用 `--packages-up-to`；增加 `ROS2_PACKAGE_ROOTS` 目录递归选择，并以无限层级 glob 监听 `package.xml` 变更；保留包过滤、custom target 和日志。
2. 修改 `perception_core/CMakeLists.txt`：添加 build-tree alias；保留 build/install target include requirements；删除手写 Config 生成与安装段；保留标准 ament export。
3. 修改 `perception_adapters/CMakeLists.txt`：添加 build-tree alias；删除 `${perception_core_INCLUDE_DIRS}`；为 Core 增加本地 target 优先、安装包回退；删除手写 Config 生成与安装段。
4. 修改 `perception_input_node/CMakeLists.txt`：对 Core、Adapter 和 ROSIDL typesupport 增加本地 target 优先回退逻辑，避免 build/install 接口 target 混用。
5. 修改 `swarm_controller/CMakeLists.txt`：对 `swarm_controller_interfaces` 使用本地 ROSIDL target 优先，消除全工作区拓扑配置产生的循环 RPATH 警告。
6. 删除 Core、Adapter 不再使用的 `cmake/*Config.cmake.in` 模板，更新 `docs/clion-cpp-cmake.md` 对选择依赖闭包和拓扑加载的说明。
7. 使用 `target_sources(... PRIVATE ...)` 将 Core、Adapter、Session Manager 和 Fixture Scene 的手写公共头文件注册到真实实现 target。
8. 重建 CLion CMake profile，确认配置日志顺序、编译数据库 include path 和 CMake File API target source 列表。

## 验证命令

在 `alien-scanner-dev` 容器内：

```bash
colcon list --base-paths /workspaces/alien-scanner/ws/src --topological-order
cmake -S /workspaces/alien-scanner -B /workspaces/alien-scanner/cmake-build-debug -DROS2_PACKAGES_SELECT=perception_adapters
cmake -S /workspaces/alien-scanner -B /tmp/alien-perception-roots -DROS2_PACKAGE_ROOTS=alien_perception
grep -nE 'perception_core/include|perception_adapters/.+laser_scan_adapter.cpp' /workspaces/alien-scanner/cmake-build-debug/compile_commands.json
colcon build --symlink-install --packages-up-to perception_input_node
colcon test --packages-select perception_core perception_adapters perception_input_node --ctest-args tests
```

验收重点：

- 配置日志在 Adapter 前加载 Core。
- 目录选择自动发现 `alien_perception` 下的全部包，并继续按拓扑顺序加载。
- Adapter 编译命令包含源码 Core include path。
- Input Node 使用当前 CMake build 目录中的 ROSIDL 生成 include，不产生本地/install typesupport 循环 RPATH 警告。
- 安装配置不再出现本包手写 `lib/cmake/<pkg>` Config。
- Core、Adapter、Input Node 构建和相关测试成功。

## 目录选择验证记录

- `ROS2_PACKAGE_ROOTS=alien_perception` 发现 5 个包，并按 Core、Interfaces、Adapter、Input Node、Fixtures 的拓扑顺序配置。
- `ROS2_PACKAGE_ROOTS=swarm_controller` 支持以单包目录作为根，并自动加载 Cave World、接口和 Drone Scanner 等工作区依赖。
- 原有 `ROS2_PACKAGES_SELECT=perception_adapters` 仍先加载 Core，再加载 Adapter。
- 空选择的全工作区配置通过；目录模式下 `perception_input_node` target 编译通过。
- 目录模式下 5 个感知包完整构建通过；Core、Adapters、Session Manager、Fixture Scene 四个 C++ 测试均通过。
- 不存在、越出 `ws/src`、不包含 ROS 包的目录均在配置阶段明确失败。
- CMake 生成的 `VerifyGlobs.cmake` 使用递归表达式并列出当前全部 `package.xml`，不再保留三层路径清单。
- 根 CMake build tree 不提供 colcon install 环境，Fixture launch 测试仍按既有质量门在干净 colcon 工作区运行；不以根级 `ctest` 的空测试或缺少已安装 Python 接口作为通过结果。

## 头文件 target 归属验证记录

- Debug profile 的 CMake File API 中，`perception_core` 列出 8 个公共头文件和实现源文件。
- `perception_adapters` 列出 3 个公共头文件和 3 个实现源文件。
- `sensor_session_manager` 列出 `SensorSessionManager.hpp` 和对应实现源文件。
- `fixture_scene` 列出 `FixtureScene.hpp` 和对应实现源文件；`PerceptionInputNode.cpp`、`FixturePublisher.cpp` 继续属于各自 executable target。
- 独立临时目录完整构建通过；Core、Adapters、Session Manager、Fixture Scene 四个 C++ 测试均通过。

## 风险文件与回滚点

- 根包扫描：`CMakeLists.txt`
- 感知库导出：`ws/src/alien_perception/perception_core/CMakeLists.txt`、`perception_adapters/CMakeLists.txt`
- 下游消费：`ws/src/alien_perception/perception_input_node/CMakeLists.txt`
- IDE 文档：`docs/clion-cpp-cmake.md`

实施前保存当前 diff；不执行 `git reset`、`checkout` 或提交。配置失败时先定位根扫描与包级导出哪一层违约，不回滚用户无关改动。

## 质量门

- [x] PRD、设计和实施计划获用户确认。
- [x] 根 CMake 拓扑配置验证通过。
- [x] 编译数据库源码 include 验证通过。
- [x] 相关 colcon build/test 通过。
- [x] `trellis-check` 复核通过，lint 按用户要求不启用。

## 收口记录（2026-07-24）

复验环境：`alien-scanner-dev` 容器，ROS 2 Jazzy。

1. `colcon list --base-paths ws/src/alien_perception --topological-order --names-only`
   顺序：`perception_core` → `perception_interfaces` → `perception_adapters` → `perception_input_node` → `perception_fixtures`。
2. `cmake -S . -B /tmp/alien-perception-cmake-closeout -DROS2_PACKAGE_ROOTS=alien_perception -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` 配置成功。
3. 配置日志按上述拓扑顺序 `Add ament package`。
4. `compile_commands.json` 中 `laser_scan_adapter.cpp` / `mapper_health_gate.cpp` 均包含
   `-I.../ws/src/alien_perception/perception_core/include`，**不**包含 `install/perception_core/include`。
5. 手写 `cmake/*Config.cmake.in` 已删除；空 `cmake/` 目录已清理。
6. 文档：`docs/clion-cpp-cmake.md`、`docs/ament-cmake-conventions.md` 已同步拓扑选择与源码 target 约定。

**结论**：本子任务验收通过，可归档。
**范围边界**：不提交 C1 业务代码；提交/合入由父任务 `07-22-c1-perception-input` 统一处理。
