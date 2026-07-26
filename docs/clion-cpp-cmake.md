# CLion + Dev Container：C++ 包 CMake 索引与 colcon 构建 Target

本文说明仓库根目录 **CMake 工程**如何为 `ament_cmake` 包做 **CLion 索引** 与 **`colcon_build_*` target**。适用于 **CLion + Dev Container + ROS 2 Jazzy**。

> **各包的 CMake 写法**（C++ 标准、库 export、下游链接）见 **[ament-cmake-conventions.md](ament-cmake-conventions.md)** — 建包前必读。

---

## 设计目标

| 目标 | 实现方式 |
|------|----------|
| CLion 跳转 / 补全 | 根 `CMakeLists.txt` 按 colcon 拓扑对包 `add_subdirectory` |
| 一键 colcon 安装 | 每包 `colcon_build_<pkg>` custom target |
| 工作区分离 | 源码在 `ws/src/`，根目录保留 Docker / Dev Container |

**两套构建语义（不要混用）：**

| 操作 | 产物位置 | 用途 |
|------|----------|------|
| Build `colcon_build_*` | `ws/build/` + `ws/install/` | 正式安装、`ros2 run` |
| Build 包内 executable target | `cmake-build-debug/` | 快速编译 / 断点调试 |

---

## 涉及文件

| 文件 | 作用 |
|------|------|
| [`CMakeLists.txt`](../CMakeLists.txt) | 扫描 `ws/src`、`add_subdirectory` |
| [`cmake/ColconBuildTargets.cmake`](../cmake/ColconBuildTargets.cmake) | `colcon_build_*` / `colcon_test_*` |
| [`scripts/colcon-build.sh`](../scripts/colcon-build.sh) | colcon build 脚本 |

---

## CMake 缓存选项

在 **Settings → CMake → CMake options** 或 **CMakePresets** 中设置。

### `ROS2_WS_DIR`

默认 `${CMAKE_CURRENT_SOURCE_DIR}/ws`；容器内为 `/workspaces/alien-scanner/ws`。

### `ROS2_PACKAGES_SELECT`

| 默认 | 与 `ROS2_PACKAGE_ROOTS` 均为空时，加载全部 `ament_cmake` 包 |
|------|--------------------------------|
| 示例 | `cave_world` 或 `cave_world;drone_scanner` |

参数是 `package.xml` 的 `<name>`，不是目录名。非空时按
`colcon list --packages-up-to` 语义加载选中包及其工作区递归依赖，随后按
`--topological-order` 顺序执行 `add_subdirectory`。

### `ROS2_PACKAGE_ROOTS`

按目录选择包。路径默认相对 `${ROS2_WS_DIR}/src`，也接受 `ws/src` 内的
绝对路径。根 CMake 使用 colcon 递归发现每个目录下的 ROS 包，再加载这些
包及其工作区递归依赖。

例如，以下选项会自动发现 `ws/src/alien_perception/` 下的 Core、Adapter、
Interfaces、Input Node 和 Fixtures，无需逐个列出包名：

```text
-DROS2_PACKAGE_ROOTS=alien_perception
```

多个目录使用分号分隔：

```text
-DROS2_PACKAGE_ROOTS=alien_perception;swarm_controller
```

`ROS2_PACKAGE_ROOTS` 与 `ROS2_PACKAGES_SELECT` 可以同时使用，二者的包集合
合并后再执行 `--packages-up-to`。目录不存在、超出 `ws/src`，或目录内没有
可发现的 ROS 包时，CMake 配置会明确失败。

从包名模式切换到目录模式时，应在 CLion profile 中清空旧的
`ROS2_PACKAGES_SELECT`；否则旧包名会按上表的合并语义继续生效。

| `ROS2_PACKAGE_ROOTS` | `ROS2_PACKAGES_SELECT` | 加载结果 |
|----------------------|------------------------|----------|
| 空 | 空 | 全部工作区 `ament_cmake` 包 |
| 非空 | 空 | 目录下的包及其工作区依赖 |
| 空 | 非空 | 指定包及其工作区依赖 |
| 非空 | 非空 | 两者合并后的包及其工作区依赖 |

包目录增删由无限层级的 `package.xml` 变更监听触发 CMake 重新配置；实际包
发现、ignore 处理和拓扑排序仍以 colcon 输出为准。

### `ROS2_COLCON_BUILD_TARGETS` / `ROS2_COLCON_TEST_TARGETS`

默认 `ON`；设为 `OFF` 则只做 CLion 索引、不注册 colcon target。

---

## 自动生成的 Target

- **`colcon_build_<包名>`** — 等价 `colcon build --packages-select <包名> --symlink-install`
- **`colcon_build_all`** — 构建当前 CMake 已加载的全部包
- **`colcon_test_<包名>`** — 依赖对应 build target 后跑 `colcon test`

示例：`colcon_build_cave_world`、`colcon_build_drone_scanner`。

---

## Reload CMake 后

1. 确认 CMake 日志中依赖包先于消费包出现，例如 `perception_core` 先于 `perception_adapters`
2. Build `colcon_build_<pkg>` 安装到 `ws/install`
3. 终端：`source ws/install/setup.bash` 后 `ros2 run …`

---

## 与 colcon 终端的关系

| 任务 | 推荐方式 |
|------|----------|
| 日常开发 / CI | 终端 `colcon build` |
| CLion 索引 / 单文件调试 | 根 CMake + `cmake-build-debug` |
| 安装后运行节点 | **`colcon_build_*`** 或终端 colcon，再用 `ws/install` |

包级 CMake 规范不受 CLion/ colcon 入口影响，见 **[ament-cmake-conventions.md](ament-cmake-conventions.md)**。
