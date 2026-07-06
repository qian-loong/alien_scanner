# 为每个 ament 包注册 CMake custom target：
#   colcon_build_<pkg>  → colcon build --packages-select <pkg>
#   colcon_test_<pkg>    → colcon test  --packages-select <pkg> --ctest-args tests
#                          （依赖 colcon_build_<pkg>，改测试代码后 Run test 会先 build）
# CLion：Reload CMake 后，运行配置列表里选 colcon_test_my_package → Run

set(_ROS2_COLCON_BUILD_SCRIPT "${CMAKE_SOURCE_DIR}/scripts/colcon-build.sh")
set(_ROS2_COLCON_TEST_SCRIPT "${CMAKE_SOURCE_DIR}/scripts/colcon-test.sh")
function(ros2_add_colcon_build_target _pkg_name)
  if(NOT ROS2_COLCON_BUILD_TARGETS)
    return()
  endif()

  if(NOT EXISTS "${_ROS2_COLCON_BUILD_SCRIPT}")
    message(WARNING "Missing ${_ROS2_COLCON_BUILD_SCRIPT}, skip colcon_build_${_pkg_name}")
    return()
  endif()

  set(_target "colcon_build_${_pkg_name}")
  if(TARGET "${_target}")
    return()
  endif()

  add_custom_target(
    "${_target}"
    COMMAND ${CMAKE_COMMAND} -E env ROS2_COLCON_QUIET=1
            /bin/bash "${_ROS2_COLCON_BUILD_SCRIPT}" "${_pkg_name}"
    WORKING_DIRECTORY "${ROS2_WS_DIR}"
    COMMENT "colcon build --packages-select ${_pkg_name}"
    VERBATIM
  )
  set_property(GLOBAL APPEND PROPERTY _ROS2_COLCON_BUILD_TARGETS "${_target}")
  message(STATUS "Colcon target: ${_target}")
endfunction()

function(ros2_finalize_colcon_build_targets)
  if(NOT ROS2_COLCON_BUILD_TARGETS)
    return()
  endif()

  get_property(_targets GLOBAL PROPERTY _ROS2_COLCON_BUILD_TARGETS)
  if(NOT _targets)
    return()
  endif()

  if(NOT TARGET colcon_build_all)
    add_custom_target(colcon_build_all COMMENT "colcon build all registered packages")
    foreach(_t IN LISTS _targets)
      add_dependencies(colcon_build_all "${_t}")
    endforeach()
    message(STATUS "Colcon target: colcon_build_all (${_targets})")
  endif()
endfunction()

function(ros2_add_colcon_test_target _pkg_name)
  if(NOT ROS2_COLCON_TEST_TARGETS)
    return()
  endif()

  if(NOT EXISTS "${_ROS2_COLCON_TEST_SCRIPT}")
    message(WARNING "Missing ${_ROS2_COLCON_TEST_SCRIPT}, skip colcon_test_${_pkg_name}")
    return()
  endif()

  set(_target "colcon_test_${_pkg_name}")
  if(TARGET "${_target}")
    return()
  endif()

  add_custom_target(
    "${_target}"
    COMMAND ${CMAKE_COMMAND} -E env ROS2_COLCON_QUIET=1
            /bin/bash "${_ROS2_COLCON_TEST_SCRIPT}" "${_pkg_name}"
    WORKING_DIRECTORY "${ROS2_WS_DIR}"
    COMMENT "colcon test --packages-select ${_pkg_name} --ctest-args tests"
    VERBATIM
  )

  set(_build_target "colcon_build_${_pkg_name}")
  if(TARGET "${_build_target}")
    add_dependencies("${_target}" "${_build_target}")
  endif()

  set_property(GLOBAL APPEND PROPERTY _ROS2_COLCON_TEST_TARGETS "${_target}")
  message(STATUS "Colcon target: ${_target}")
endfunction()

function(ros2_finalize_colcon_test_targets)
  if(NOT ROS2_COLCON_TEST_TARGETS)
    return()
  endif()

  get_property(_targets GLOBAL PROPERTY _ROS2_COLCON_TEST_TARGETS)
  if(NOT _targets)
    return()
  endif()

  if(NOT TARGET colcon_test_all)
    add_custom_target(colcon_test_all COMMENT "colcon test all registered packages")
    foreach(_t IN LISTS _targets)
      add_dependencies(colcon_test_all "${_t}")
    endforeach()
    if(TARGET colcon_build_all)
      add_dependencies(colcon_test_all colcon_build_all)
    endif()
    message(STATUS "Colcon target: colcon_test_all (${_targets})")
  endif()
endfunction()
