cmake_minimum_required(VERSION 3.16)
project(alien_scanner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# colcon 工作空间在仓库 ws/ 子目录（Dev Container 内为 /workspaces/alien-scanner/ws）
set(ROS2_WS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ws" CACHE PATH "ROS 2 colcon workspace root")

# CLion Dev Container / 本地：ROS 2 前缀
if(DEFINED ENV{ROS_DISTRO})
  list(PREPEND CMAKE_PREFIX_PATH "/opt/ros/$ENV{ROS_DISTRO}")
else()
  list(PREPEND CMAKE_PREFIX_PATH "/opt/ros/jazzy")
endif()

if(EXISTS "${ROS2_WS_DIR}/install/setup.bash")
  list(PREPEND CMAKE_PREFIX_PATH "${ROS2_WS_DIR}/install")
endif()

# 类似 colcon --packages-select：空=加载全部 ament_cmake 包；非空=只加载列出的包名
# 例：turtlesim 或 turtlesim;roscpp_tutorials（CLion CMake 选项 / -DROS2_PACKAGES_SELECT=...）
set(ROS2_PACKAGES_SELECT "" CACHE STRING
    "Semicolon-separated ROS 2 package names to load in CLion; empty = all ament_cmake packages")

# 每个 ament 包自动 add_custom_target(colcon_build_<pkg> / colcon_test_<pkg>)，CLion 运行配置一键 colcon
option(ROS2_COLCON_BUILD_TARGETS "Register colcon_build_<pkg> custom targets" ON)
option(ROS2_COLCON_TEST_TARGETS "Register colcon_test_<pkg> custom targets" ON)
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/ColconBuildTargets.cmake")

# ---------------------------------------------------------------------------
# 扫描 ws/src 下的 ament_cmake 包
# Docker / compose / Dockerfile 等保留在仓库根目录，不参与 add_subdirectory
# ---------------------------------------------------------------------------
function(_ros2_package_name_from_xml _xml_out _xml)
  if("${_xml}" MATCHES "<name>[ \t]*([^< \t\r\n]+)[ \t]*</name>")
    set("${_xml_out}" "${CMAKE_MATCH_1}" PARENT_SCOPE)
  else()
    set("${_xml_out}" "" PARENT_SCOPE)
  endif()
endfunction()

function(_ros2_add_ament_packages)
  set_property(GLOBAL PROPERTY _ROS2_COLCON_BUILD_TARGETS "")
  set_property(GLOBAL PROPERTY _ROS2_COLCON_TEST_TARGETS "")

  if(ROS2_PACKAGES_SELECT)
    message(STATUS "ROS2_PACKAGES_SELECT: ${ROS2_PACKAGES_SELECT}")
  else()
    message(STATUS "ROS2_PACKAGES_SELECT: (empty, loading all ament_cmake packages)")
  endif()

  set(_search_depths
    "${ROS2_WS_DIR}/src/*/package.xml"
    "${ROS2_WS_DIR}/src/*/*/package.xml"
    "${ROS2_WS_DIR}/src/*/*/*/package.xml"
  )
  file(GLOB _package_xmls CONFIGURE_DEPENDS ${_search_depths})
  list(SORT _package_xmls)

  foreach(_pkg_xml IN LISTS _package_xmls)
    get_filename_component(_pkg_dir "${_pkg_xml}" DIRECTORY)
    file(RELATIVE_PATH _rel "${CMAKE_CURRENT_SOURCE_DIR}" "${_pkg_dir}")

    if(EXISTS "${_pkg_dir}/COLCON_IGNORE" OR EXISTS "${_pkg_dir}/AMENT_IGNORE")
      message(STATUS "Skip ${_rel}: COLCON_IGNORE/AMENT_IGNORE")
      continue()
    endif()

    file(READ "${_pkg_xml}" _xml)
    if(NOT _xml MATCHES "<build_type>ament_cmake</build_type>")
      message(STATUS "Skip ${_rel}: not ament_cmake (ROS1 catkin or other)")
      continue()
    endif()

    _ros2_package_name_from_xml(_pkg_name "${_xml}")
    if(_pkg_name STREQUAL "")
      message(WARNING "Skip ${_rel}: cannot parse <name> from package.xml")
      continue()
    endif()

    if(ROS2_PACKAGES_SELECT)
      list(FIND ROS2_PACKAGES_SELECT "${_pkg_name}" _select_index)
      if(_select_index LESS 0)
        message(STATUS "Skip ${_rel} (${_pkg_name}): not in ROS2_PACKAGES_SELECT")
        continue()
      endif()
    endif()

    message(STATUS "Add ament package: ${_pkg_name} (${_rel})")
    ros2_add_colcon_build_target("${_pkg_name}")
    ros2_add_colcon_test_target("${_pkg_name}")
    add_subdirectory("${_rel}")
  endforeach()

  ros2_finalize_colcon_build_targets()
  ros2_finalize_colcon_test_targets()
endfunction()

_ros2_add_ament_packages()
