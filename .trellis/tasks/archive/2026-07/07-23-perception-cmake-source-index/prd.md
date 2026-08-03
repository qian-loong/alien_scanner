# 修复感知包 CMake 拓扑与源码索引

## Goal

使 CLion 根 CMake 与 ROS 2 工作区的依赖关系一致：包按 colcon 拓扑顺序加载，感知包优先消费源码构建 target，`SensorDescriptor` 等手写头文件在 CLion 中跳转到 `ws/src`。

## Background

- 根 [CMakeLists.txt](../../../CMakeLists.txt:59) 目前扫描 `package.xml` 后按路径字典序排序，并直接 `add_subdirectory()`，不是依赖拓扑顺序。
- 当前缓存选择顺序为 `perception_adapters;perception_interfaces;perception_core`，而 colcon 拓扑顺序为 `perception_core;perception_interfaces;perception_adapters`。
- 根 CMake 将 `ws/install` 放入 `CMAKE_PREFIX_PATH`。由于 Adapter 先于 Core 配置，`find_package(perception_core)` 命中安装配置，Adapter 编译命令只得到 `ws/install/perception_core/include`。
- [perception_adapters/CMakeLists.txt](../../../ws/src/alien_perception/perception_adapters/CMakeLists.txt:27) 显式消费 `${perception_core_INCLUDE_DIRS}`；该旧式变量不是源码 target 的可靠来源。
- install 下的 `sensor_descriptor.hpp` 是指向源码的符号链接，但 IDE 仍按编译数据库中的 install include 解析。
- Core 与 Adapter 同时手动安装 `lib/cmake/<pkg>` Config 和调用 `ament_package()`，存在两套 package config。

## Requirements

### R1：拓扑加载

- 根 CMake 使用 colcon 已解析的工作区包拓扑顺序，不再以 `list(SORT _package_xmls)` 决定 `add_subdirectory()` 顺序。
- `ROS2_PACKAGES_SELECT` 为空时加载全部可用 `ament_cmake` 包；非空时加载选中包及其递归工作区构建依赖。
- `ROS2_PACKAGE_ROOTS` 可按 `ws/src` 下的目录递归选择全部 ROS 包，并与显式包名合并后展开工作区依赖。
- `package.xml` 的 CMake 变更监听不限制目录深度；实际发现、ignore 处理和拓扑排序仍由 colcon 完成。
- 保留 ignore/非 ament 包过滤以及现有 `colcon_build_*`、`colcon_test_*` target 注册。
- colcon 不可执行或拓扑解析失败时给出明确配置错误，不静默恢复为字典序。

### R2：源码 target 优先

- `perception_core` 提供构建树别名 `perception_core::perception_core`。
- `perception_adapters` 提供构建树别名 `perception_adapters::perception_adapters`。
- 感知包之间通过 target 传递 include/link usage requirements，不显式消费 `${perception_core_INCLUDE_DIRS}`。
- 独立 colcon 包构建仍可通过 `find_package()` 使用已安装依赖；根 CMake superbuild 在本地 target 已存在时不强制解析 install target。

### R3：统一包导出

- Core、Adapter 使用标准 `ament_export_targets()` / `ament_package()` 生成安装配置。
- 移除重复的手写 `configure_package_config_file()`、Config 安装段及对应模板。
- target 命名保持包级形式：`perception_core::perception_core`、`perception_adapters::perception_adapters`。

### R4：索引边界

- `SensorDescriptor` 等手写头文件的编译数据库包含 `ws/src/alien_perception/perception_core/include`。
- 根 superbuild 已加载 `perception_interfaces` 时，Input Node 优先使用本地 ROSIDL typesupport target，生成头来自当前 build 目录；独立 colcon 构建回退安装包。
- 根拓扑加载不得使既有 `swarm_controller_interfaces` 同时混入本地与 install typesupport；Swarm 消费者采用相同的本地 target 优先规则。
- `perception_interfaces` 的 `.msg` 生成头文件属于生成产物，不要求跳转到不存在的源码 `.hpp`。
- Core、Adapter、Session Manager 和 Fixture Scene 的手写公共头文件通过 `target_sources(... PRIVATE ...)` 显式属于其实现 target，CLion 不依赖 include 图猜测 header context。
- 不修改运行时节点逻辑、ROS 消息定义、launch 行为或测试断言。

## Acceptance Criteria

- [x] 根 CMake 配置日志显示包按 colcon 拓扑顺序加入；选择 `perception_adapters` 时 Core 自动先加入。
- [x] 指定 `ROS2_PACKAGE_ROOTS=alien_perception` 时自动发现目录下全部包，并按依赖拓扑加载。
- [x] 任意深度新增或删除 `package.xml` 后，CMake 能触发重新配置，不受原三层 glob 限制。
- [x] Adapter 编译命令包含 `ws/src/alien_perception/perception_core/include`，不依赖 install include 解析 `SensorDescriptor`。
- [x] Core、Adapter、Input Node 在根 CMake superbuild 和独立 colcon 构建中都能解析正确 target。
- [x] Input Node 的根 CMake 配置不同时混入本地和 install 版 `perception_interfaces` typesupport，不产生循环 RPATH 警告。
- [x] 加载全部工作区包时不产生 `swarm_controller_interfaces` 本地/install typesupport 循环 RPATH 警告。
- [x] 安装消费命中标准 ament 配置，不再命中本包手写的 `lib/cmake/<pkg>` Config。
- [x] 相关感知 build/test 通过；不启用 lint 检测。
- [x] 清理并重载 CLion CMake 缓存后，`SensorDescriptor` 跳转目标为 `ws/src/.../sensor_descriptor.hpp`。
- [x] CMake File API 中，四个感知库 target 均列出其手写公共头文件和实现源文件。

## Out Of Scope

- 不将 ROSIDL 生成头文件重定向到源码目录。
- 不修改 C++ 算法、ROS 节点管道、消息接口或 launch 文件。
- 不改动 git 分支、提交或未授权的无关工作区变更。
