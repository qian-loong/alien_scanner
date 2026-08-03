# CMake 拓扑与源码索引修复设计

## 1. 设计目标

让根 CMake 的包加载顺序与 colcon 一致，并在根 superbuild 中优先复用源码包 target。安装包仍保留标准 ament 消费路径，保证单包 colcon 构建不依赖根 CMake。

## 2. 根包发现与拓扑

根 CMake 使用 `execute_process()` 调用：

```text
colcon list --base-paths <ROS2_WS_DIR>/src --topological-order --paths-only
```

当 `ROS2_PACKAGES_SELECT` 非空时追加：

```text
--packages-up-to <selected packages>
```

colcon 负责 package.xml 解析、依赖条件、ignore 标记和拓扑排序；根 CMake 读取有序包目录并解析包名，随后按返回顺序注册 custom target 与 `add_subdirectory()`。不保留路径排序回退，命令失败直接 `FATAL_ERROR`。

选择语义从“只加载列出的包”调整为“加载列出的包及其工作区递归构建依赖”。外部 ROS 包仍由安装前缀中的 `find_package()` 提供。

目录选择通过独立缓存变量 `ROS2_PACKAGE_ROOTS` 提供。每个路径默认相对
`<ROS2_WS_DIR>/src`，并限制在源码根目录内。根 CMake 先对每个目录执行
`colcon list --base-paths <root> --names-only`，得到目录内的包名；再将这些
包名与 `ROS2_PACKAGES_SELECT` 合并，交给全工作区 `--packages-up-to` 展开
依赖并输出拓扑路径。这样目录外的工作区依赖不会遗漏，ignore 语义仍由
colcon 负责。

`package.xml` 的新增和删除继续通过 `CONFIGURE_DEPENDS` 触发重新配置，但
固定三层 glob 改为 `GLOB_RECURSE`。该 glob 不参与包解析或排序。

## 3. 感知库 target 契约

Core 和 Adapter 的实际 target 名称继续使用包名 snake_case，并创建 build-tree alias：

```cmake
add_library(perception_core ...)
add_library(perception_core::perception_core ALIAS perception_core)

add_library(perception_adapters ...)
add_library(perception_adapters::perception_adapters ALIAS perception_adapters)
```

Core 的 `PUBLIC` include directories 保留 build/install generator expression。Adapter 只声明自己的 include 目录，通过 `target_link_libraries(... PUBLIC perception_core::perception_core)` 继承 Core include，删除 `${perception_core_INCLUDE_DIRS}`。

为保持包级 `cmake_minimum_required(VERSION 3.8)` 兼容，手写公共头文件使用
`target_sources(<target> PRIVATE ...)` 显式注册到真实实现 target。`PRIVATE`
避免将源码绝对路径传播为 `INTERFACE_SOURCES`；公共 API 仍由 PUBLIC include
目录、安装规则和 ament export 表达。该注册只补充 CMake/CLion 文件归属，
不改变编译、链接或安装产物。

## 4. 内部依赖解析

Adapter 和 Input Node 对工作区内部库采用 target 优先策略：

```cmake
if(NOT TARGET perception_core::perception_core)
  find_package(perception_core REQUIRED)
endif()
```

Adapter 对 Core、Input Node 对 Core/Adapter 均遵循此模式。ROSIDL 接口也使用相同边界：根 superbuild 中若存在本地 `<pkg>__rosidl_typesupport_cpp` target，则直接链接本地 target；独立 colcon 构建时才 `find_package(<pkg>)` 并交给 `ament_target_dependencies()`。该规则同时覆盖 `perception_interfaces` 和根拓扑加载会提前定义的 `swarm_controller_interfaces`。

## 5. 安装配置

Core、Adapter 保留 `install(TARGETS ... EXPORT ...)` 与 `ament_export_targets(... HAS_LIBRARY_TARGET)`，由 `ament_package()` 生成唯一标准 package config。删除手写 `CMakePackageConfigHelpers` 配置和 `cmake/*Config.cmake.in` 模板，避免 `lib/cmake/<pkg>` 配置优先于 ament 的 `share/<pkg>/cmake` 配置。

## 6. 兼容性与风险

| 风险 | 缓解 |
|---|---|
| CLion 本机环境没有 colcon | 项目约定在 ROS 2 Dev Container 中配置；命令缺失时给出可操作错误 |
| 选中下游包但依赖未加载 | 使用 `--packages-up-to` 自动包含工作区递归依赖 |
| CMakeCache 保留旧 `*_DIR` | 实施后重建 CLion CMake profile，并检查 `compile_commands.json` |
| 安装配置与 build-tree alias 名称不一致 | 两者统一使用包级 target 名称 |
| 生成接口同时混入 build/install | superbuild 直接链接本地 ROSIDL typesupport；独立包构建才使用 install target |

## 7. 回滚

若根 superbuild 对现有包产生未预期配置错误，可独立回滚根 CMake 的 colcon 拓扑段。包级 alias、标准 ament 导出和旧式 include 变量清理作为同一修改集验证，不改变运行时源代码。
