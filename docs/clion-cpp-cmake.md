# CLion + Dev Container：C++ 包 CMake 索引与 colcon 构建 Target

本文说明仓库根目录 **CMake 工程**如何为 `ament_cmake` 包做 **CLion 索引** 与 **`colcon_build_*` target**。适用于 **CLion + Dev Container + ROS 2 Jazzy**。

> **各包的 CMake 写法**（C++ 标准、库 export、下游链接）见 **[ament-cmake-conventions.md](ament-cmake-conventions.md)** — 建包前必读。

---

## 设计目标

| 目标 | 实现方式 |
|------|----------|
| CLion 跳转 / 补全 | 根 `CMakeLists.txt` 对包 `add_subdirectory` |
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

| 默认 | 空 = 加载全部 `ament_cmake` 包 |
|------|--------------------------------|
| 示例 | `cave_world` 或 `cave_world;drone_scanner` |

过滤的是 `package.xml` 的 `<name>`，不是目录名。

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

1. 确认 CMake 日志中有 `Add ament package: cave_world` 等
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
