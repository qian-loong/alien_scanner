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

# 空=不按包名筛选；非空=加载列出的包及其工作区递归依赖
# 例：perception_input_node 或 cave_world;drone_scanner
set(ROS2_PACKAGES_SELECT "" CACHE STRING
    "Semicolon-separated ROS 2 package names to load with workspace dependencies")

# 相对 ws/src 的目录（也接受 ws/src 内的绝对路径）；递归加载目录下所有包及其工作区依赖
# 例：alien_perception 或 alien_perception;swarm_controller
set(ROS2_PACKAGE_ROOTS "" CACHE STRING
    "Semicolon-separated directories under ws/src whose ROS 2 packages should be loaded recursively")

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

function(_ros2_lines_to_list _list_out _lines)
  string(REPLACE "\r\n" "\n" _normalized_lines "${_lines}")
  string(REPLACE "\r" "\n" _normalized_lines "${_normalized_lines}")
  string(REPLACE "\n" ";" _items "${_normalized_lines}")
  list(FILTER _items EXCLUDE REGEX "^$")
  set("${_list_out}" "${_items}" PARENT_SCOPE)
endfunction()

function(_ros2_colcon_package_paths _paths_out _selection_seeds_out)
  find_program(_colcon_executable NAMES colcon)
  if(NOT _colcon_executable)
    message(FATAL_ERROR
      "colcon was not found. Configure this project inside the ROS 2 Dev Container.")
  endif()

  set(_selection_seeds ${ROS2_PACKAGES_SELECT})
  if(ROS2_PACKAGE_ROOTS)
    get_filename_component(_source_root "${ROS2_WS_DIR}/src" REALPATH)

    foreach(_root IN LISTS ROS2_PACKAGE_ROOTS)
      if(IS_ABSOLUTE "${_root}")
        set(_root_candidate "${_root}")
      else()
        set(_root_candidate "${_source_root}/${_root}")
      endif()

      if(NOT IS_DIRECTORY "${_root_candidate}")
        message(FATAL_ERROR
          "ROS2_PACKAGE_ROOTS entry is not a directory: ${_root}")
      endif()

      get_filename_component(_root_path "${_root_candidate}" REALPATH)
      file(RELATIVE_PATH _root_relative "${_source_root}" "${_root_path}")
      file(TO_CMAKE_PATH "${_root_relative}" _root_relative)
      if(_root_relative MATCHES "^\\.\\.(/|$)")
        message(FATAL_ERROR
          "ROS2_PACKAGE_ROOTS entry must stay under ${_source_root}: ${_root}")
      endif()

      execute_process(
        COMMAND "${_colcon_executable}" list
          --base-paths "${_root_path}"
          --topological-order
          --names-only
        WORKING_DIRECTORY "${ROS2_WS_DIR}"
        RESULT_VARIABLE _root_colcon_result
        OUTPUT_VARIABLE _root_colcon_output
        ERROR_VARIABLE _root_colcon_error
        OUTPUT_STRIP_TRAILING_WHITESPACE
      )
      if(NOT _root_colcon_result EQUAL 0)
        string(STRIP "${_root_colcon_error}" _root_colcon_error)
        message(FATAL_ERROR
          "colcon package discovery failed for ROS2_PACKAGE_ROOTS entry '${_root}': "
          "${_root_colcon_error}")
      endif()

      _ros2_lines_to_list(_root_packages "${_root_colcon_output}")
      if(NOT _root_packages)
        message(FATAL_ERROR
          "ROS2_PACKAGE_ROOTS entry contains no discoverable ROS 2 packages: ${_root}")
      endif()
      list(APPEND _selection_seeds ${_root_packages})
    endforeach()
  endif()
  list(REMOVE_DUPLICATES _selection_seeds)

  set(_colcon_command
    "${_colcon_executable}" list
    --base-paths "${ROS2_WS_DIR}/src"
    --topological-order
    --paths-only
  )
  if(_selection_seeds)
    list(APPEND _colcon_command --packages-up-to ${_selection_seeds})
  endif()

  execute_process(
    COMMAND ${_colcon_command}
    WORKING_DIRECTORY "${ROS2_WS_DIR}"
    RESULT_VARIABLE _colcon_result
    OUTPUT_VARIABLE _colcon_output
    ERROR_VARIABLE _colcon_error
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT _colcon_result EQUAL 0)
    string(STRIP "${_colcon_error}" _colcon_error)
    message(FATAL_ERROR "colcon package discovery failed: ${_colcon_error}")
  endif()

  _ros2_lines_to_list(_package_paths "${_colcon_output}")
  set("${_paths_out}" "${_package_paths}" PARENT_SCOPE)
  set("${_selection_seeds_out}" "${_selection_seeds}" PARENT_SCOPE)
endfunction()

function(_ros2_add_ament_packages)
  set_property(GLOBAL PROPERTY _ROS2_COLCON_BUILD_TARGETS "")
  set_property(GLOBAL PROPERTY _ROS2_COLCON_TEST_TARGETS "")

  # This glob only triggers reconfiguration when package manifests change;
  # colcon remains the source of truth for discovery and topological ordering.
  file(GLOB_RECURSE _discovered_package_xmls
    CONFIGURE_DEPENDS
    LIST_DIRECTORIES FALSE
    "${ROS2_WS_DIR}/src/*/package.xml"
  )
  _ros2_colcon_package_paths(_package_dirs _selection_seeds)

  if(ROS2_PACKAGE_ROOTS)
    message(STATUS "ROS2_PACKAGE_ROOTS: ${ROS2_PACKAGE_ROOTS}")
  endif()
  if(ROS2_PACKAGES_SELECT)
    message(STATUS "ROS2_PACKAGES_SELECT: ${ROS2_PACKAGES_SELECT}")
  endif()
  if(_selection_seeds)
    message(STATUS
      "ROS 2 package selection seeds: ${_selection_seeds} (including workspace dependencies)")
  else()
    message(STATUS "ROS 2 package selection: loading all ament_cmake packages")
  endif()

  foreach(_pkg_dir IN LISTS _package_dirs)
    set(_pkg_xml "${_pkg_dir}/package.xml")
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

    message(STATUS "Add ament package: ${_pkg_name} (${_rel})")
    ros2_add_colcon_build_target("${_pkg_name}")
    ros2_add_colcon_test_target("${_pkg_name}")
    add_subdirectory("${_rel}")
  endforeach()

  ros2_finalize_colcon_build_targets()
  ros2_finalize_colcon_test_targets()
endfunction()

_ros2_add_ament_packages()
