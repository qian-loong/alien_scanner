# ament_cmake 包 CMake 约定

面向 `ws/src/` 下新建或修改的 **C++ ROS 2 包**。适用于 **colcon build** 与 **CLion 根 CMake `add_subdirectory`** 两条路径。

> CLion 索引、`colcon_build_*` target、CMakePresets 等 IDE 专用说明见 [clion-cpp-cmake.md](clion-cpp-cmake.md)。

---

## 1. 两套 CMake 入口（必读）

| 入口 | 触发 | 读哪个 CMakeLists | 产物 |
|------|------|-------------------|------|
| **colcon** | 终端 / `colcon_build_*` | 各包 `ws/src/<pkg>/CMakeLists.txt` | `ws/build/`、`ws/install/` |
| **CLion** | Reload CMake | 仓库根 `CMakeLists.txt` → `add_subdirectory` | `cmake-build-debug/` |

**结论：** 根目录 `set(CMAKE_CXX_STANDARD …)` **只管 CLion 索引**；**每个 ament 包必须在自身 CMakeLists 里声明 C++ 标准**，否则 colcon 构建不受根目录约束。

---

## 2. C++ 标准

### 2.1 包级（默认做法）

在 `project(...)` 之后、`find_package` 之前或之后均可：

```cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
```

仓库根 `CMakeLists.txt` 同样设为 **17**（与 Jazzy / 实际代码一致），仅用于 CLion 子目录继承。

### 2.2 何时还要写 `target_compile_features`

| 场景 | 是否需要 |
|------|----------|
| 包内 node / 测试 executable | ❌ 包级 17 即可 |
| **会被其他包链接的库** | 可选 `target_compile_features(<lib> PUBLIC cxx_std_17)`，向 downstream 声明最低标准 |

**不要**在每个 executable 上重复 `target_compile_features(cxx_std_17)`（模板复制粘贴遗留）。

---

## 3. 库分类

| 类型 | 示例 | install | ament export |
|------|------|---------|--------------|
| **包内算法库** | `drone_trajectory`、`drone_lidar` | 可选 `install(TARGETS … ARCHIVE/LIBRARY)` | ❌ 不 export |
| **对外算法库** | `cave_geometry` | `install(TARGETS … EXPORT export_${PROJECT_NAME})` | ✅ 见 §4 |
| **节点 executable** | `fake_lidar`、`cave_publisher` | `install(TARGETS … DESTINATION lib/${PROJECT_NAME})` | ❌ |

包内库仅本包链接时用 `target_link_libraries(node my_lib)`，不必 export。

---

## 4. 导出静态/动态库（推荐写法）

### 4.1 创建与 install

库类型在 **`add_library` 时确定**，不在 `install` 里写 STATIC/SHARED：

```cmake
add_library(cave_geometry STATIC
  src/ProceduralCaveField.cpp
  ...
)
target_include_directories(cave_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS cave_geometry
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib   # .a
        LIBRARY DESTINATION lib)  # .so（STATIC 时无产物）
install(DIRECTORY include/ DESTINATION include)  # 或 install(FILES ...)
```

### 4.2 export

在 `ament_package()` 之前、**包 CMakeLists 顶层**直接写（不要包进 function）：

```cmake
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
```

**不要**对静态库使用 `ament_export_libraries(cave_geometry)`：生成的 import 描述可能指向过期的 `.so`，而实际产物是 `.a`（曾导致 `createCaveField` 链接失败）。

### 4.3 下游链接

```cmake
find_package(cave_world REQUIRED)
target_link_libraries(fake_lidar drone_lidar cave_world::cave_geometry)
```

**不要**使用 `${cave_world_LIBRARIES}` 等旧式变量（除非依赖的第三方包尚未 export target）。

### 4.4 根 CMake superbuild 的构建树别名

如果同一个仓库的根 CMake 通过 `add_subdirectory` 同时加载依赖包和消费包，
对外库必须让构建树别名与安装导出 target 同名：

```cmake
add_library(perception_core ...)
add_library(perception_core::perception_core ALIAS perception_core)
```

消费包先检查本地 target；只有独立 colcon 构建没有该 target 时才调用
`find_package`：

```cmake
if(NOT TARGET perception_core::perception_core)
  find_package(perception_core REQUIRED)
endif()
target_link_libraries(perception_adapters PUBLIC perception_core::perception_core)
```

这样 `BUILD_INTERFACE` 会沿 target usage requirements 传播源码 include 目录，
不需要显式拼接 `${perception_core_INCLUDE_DIRS}`。ROSIDL 接口包同理：根
superbuild 中优先链接本地 `<pkg>__rosidl_typesupport_cpp`，独立包构建才消费
安装 target。

### 4.5 公共头文件的 target 归属（CMake 3.8 兼容）

公共头文件必须显式属于其实现库，避免 CLion 只能通过 include 图猜测 header
context。保持当前最低 CMake 版本时，在 `add_library()` 后使用
`target_sources(... PRIVATE ...)`：

```cmake
set(cave_geometry_header_files
  include/cave_world/ICaveField.hpp
  include/cave_world/ProceduralCaveField.hpp
)

add_library(cave_geometry
  src/ProceduralCaveField.cpp
)
target_sources(cave_geometry
  PRIVATE
  ${cave_geometry_header_files}
)
```

这里的 `PRIVATE` 只表示不把源码路径传播为 downstream 的
`INTERFACE_SOURCES`，不表示这些头文件是私有 API。公共 include 契约仍由
`target_include_directories(... PUBLIC ...)`、`install(DIRECTORY include/ ...)`
和 `ament_export_*` 表达。

不要用 `file(GLOB_RECURSE ...)` 收集 C++ 头文件，也不要创建只为 IDE 展示的
`add_custom_target(... SOURCES ...)`；两者都不能稳定表达真实库的文件所有权。

---

## 5. 新建包 Checklist

```
ws/src/<pkg>/
├── CMakeLists.txt      # 包级 C++17；分层 add_library + 薄 node
├── package.xml
├── include/<pkg>/…     # 算法头文件（无 rclcpp）
├── src/…               # 算法实现
├── src/<pkg>_node.cpp  # 可选：薄节点
├── launch/             # 优先 .launch.xml；复杂时用 .py
└── test/               # gtest 只测无 ROS 算法库
```

**CMakeLists 顺序建议：**

1. `cmake_minimum_required` / `project`
2. `set(CMAKE_CXX_STANDARD 17)` 等
3. 编译告警选项
4. `find_package(ament_cmake …)` 及依赖
5. `add_library` 算法库 → `add_executable` 节点
6. `install` / `ament_export_*`（若对外库）
7. `BUILD_TESTING` → gtest
8. `ament_package()`

---

## 6. 反模式

| 反模式 | 原因 |
|--------|------|
| 只在根 CMake 设 C++14/17，包内不写 | colcon 不读根 CMake |
| 每个 target 都 `cxx_std_17` | 冗余；包级一次即可 |
| `ament_export_libraries` 导 STATIC 库 | import 路径易错 |
| `ament_export_*` 写在 function 内 | package hook 不注册 |
| install 目录残留旧 `.so` + 新 `.a` | 链接到无符号的旧 so；用 `ament_export_targets` |
| 公共头文件不属于任何真实 target | CLion 只能猜测 header context，直接打开时跳转可能失效 |
| 用 glob/custom target 维护公共头文件 | 文件所有权不明确，或归入错误的 IDE target |

---

## 7. 参考实现

| 包 | 要点 |
|----|------|
| `cave_world` | 对外 `cave_geometry` STATIC + `ament_export_targets` |
| `drone_scanner` | 包内 `drone_lidar` / `drone_trajectory`；节点链 `cave_world::cave_geometry` |
