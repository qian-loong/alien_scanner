cmake_minimum_required(VERSION 3.8)
project(perception_local_map)

option(
  PERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS
  "Compile profiling-only LTTng-UST stage latency tracepoints"
  OFF
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
if(NOT TARGET perception_core::perception_core)
  find_package(perception_core REQUIRED)
endif()
if(NOT TARGET perception_map_update::core AND
   NOT TARGET perception_map_update::perception_map_update_core)
  find_package(perception_map_update REQUIRED)
endif()
if(TARGET perception_map_update::core)
  set(PERCEPTION_MAP_UPDATE_CORE_TARGET perception_map_update::core)
else()
  set(PERCEPTION_MAP_UPDATE_CORE_TARGET
    perception_map_update::perception_map_update_core
  )
endif()
if(TARGET perception_map_update::ros)
  set(PERCEPTION_MAP_UPDATE_ROS_TARGET perception_map_update::ros)
else()
  set(PERCEPTION_MAP_UPDATE_ROS_TARGET
    perception_map_update::perception_map_update_ros
  )
endif()
if(TARGET perception_map_update::parameters)
  set(PERCEPTION_MAP_UPDATE_PARAMETERS_TARGET perception_map_update::parameters)
else()
  set(PERCEPTION_MAP_UPDATE_PARAMETERS_TARGET
    perception_map_update::perception_map_update_parameters
  )
endif()
if(NOT TARGET perception_interfaces__rosidl_typesupport_cpp)
  find_package(perception_interfaces REQUIRED)
endif()
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

if(PERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LTTNG_UST REQUIRED IMPORTED_TARGET lttng-ust)
  add_library(perception_local_map_stage_latency SHARED
    src/StageLatencyInstrumentation.cpp
    src/StageLatencyTracepoints.cpp
  )
  target_sources(perception_local_map_stage_latency PRIVATE
    src/StageLatencyInstrumentation.hpp
    src/StageLatencyTracepoints.hpp
  )
  target_include_directories(perception_local_map_stage_latency PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(perception_local_map_stage_latency PRIVATE PkgConfig::LTTNG_UST)
endif()

set(PERCEPTION_LOCAL_MAP_CORE_HEADERS
  include/perception_local_map/MapTypes.hpp
  include/perception_local_map/OccupancyBackend.hpp
  include/perception_local_map/SensorExtrinsicRegistry.hpp
  include/perception_local_map/LocalObservationMapper.hpp
  include/perception_local_map/CanonicalSnapshotAdapter.hpp
  include/perception_local_map/AsyncMapUpdateProducer.hpp
)

add_library(perception_local_map_core STATIC
  src/MapTypes.cpp
  src/SensorExtrinsicRegistry.cpp
  src/LocalObservationMapper.cpp
  src/CanonicalSnapshotAdapter.cpp
  src/AsyncMapUpdateProducer.cpp
)
target_sources(perception_local_map_core PRIVATE ${PERCEPTION_LOCAL_MAP_CORE_HEADERS})
add_library(${PROJECT_NAME}::core ALIAS perception_local_map_core)
add_library(${PROJECT_NAME}::perception_local_map_core ALIAS perception_local_map_core)
target_include_directories(perception_local_map_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(perception_local_map_core PUBLIC cxx_std_17)
target_link_libraries(perception_local_map_core PUBLIC
  perception_core::perception_core
  ${PERCEPTION_MAP_UPDATE_CORE_TARGET}
  Threads::Threads
)
if(PERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS)
  target_compile_definitions(perception_local_map_core PRIVATE
    PERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS=1
  )
  target_link_libraries(perception_local_map_core PRIVATE perception_local_map_stage_latency)
endif()

add_library(perception_local_map_octomap STATIC
  src/OctoMapBackend.cpp
)
target_sources(perception_local_map_octomap PRIVATE
  include/perception_local_map/OctoMapBackend.hpp
)
add_library(${PROJECT_NAME}::octomap ALIAS perception_local_map_octomap)
add_library(${PROJECT_NAME}::perception_local_map_octomap ALIAS perception_local_map_octomap)
target_include_directories(perception_local_map_octomap PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
)
target_compile_features(perception_local_map_octomap PUBLIC cxx_std_17)
target_link_libraries(perception_local_map_octomap
  PUBLIC perception_local_map_core
  PRIVATE ${OCTOMAP_LIBRARIES}
)

add_executable(perception_local_map_node
  src/PerceptionLocalMapNode.cpp
  src/OctoMapSnapshotBridge.cpp
)
target_include_directories(perception_local_map_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  ${EIGEN3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)
target_link_libraries(perception_local_map_node
  perception_local_map_core
  perception_local_map_octomap
  ${PERCEPTION_MAP_UPDATE_ROS_TARGET}
  ${PERCEPTION_MAP_UPDATE_PARAMETERS_TARGET}
  ${OCTOMAP_LIBRARIES}
)
if(PERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS)
  target_compile_definitions(perception_local_map_node PRIVATE
    PERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS=1
  )
  target_link_libraries(perception_local_map_node perception_local_map_stage_latency)
endif()
if(TARGET perception_interfaces__rosidl_typesupport_cpp)
  target_link_libraries(perception_local_map_node perception_interfaces__rosidl_typesupport_cpp)
endif()
ament_target_dependencies(perception_local_map_node
  rclcpp geometry_msgs diagnostic_msgs tf2 tf2_ros octomap_msgs
)
if(perception_interfaces_FOUND)
  ament_target_dependencies(perception_local_map_node perception_interfaces)
endif()

install(DIRECTORY include/ DESTINATION include)
set(PERCEPTION_LOCAL_MAP_EXPORTED_TARGETS
  perception_local_map_core
  perception_local_map_octomap
)
if(PERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS)
  list(APPEND PERCEPTION_LOCAL_MAP_EXPORTED_TARGETS perception_local_map_stage_latency)
endif()
install(TARGETS ${PERCEPTION_LOCAL_MAP_EXPORTED_TARGETS}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS perception_local_map_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(cave_world REQUIRED)
  find_package(drone_scanner REQUIRED)
  find_package(yaml_cpp_vendor REQUIRED)
  find_package(yaml-cpp REQUIRED)
  if(NOT TARGET perception_fixtures::fixture_scene)
    find_package(perception_fixtures REQUIRED)
  endif()
  ament_add_gtest(TestLocalMap
    test/TestLocalMap.cpp
    test/DeterministicVoxelBackend.cpp
  )
  if(TARGET TestLocalMap)
    target_include_directories(TestLocalMap PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/test
      ${OCTOMAP_INCLUDE_DIRS}
    )
    target_link_libraries(TestLocalMap
      perception_local_map_core
      perception_local_map_octomap
      perception_fixtures::fixture_scene
      ${OCTOMAP_LIBRARIES}
    )
  endif()

  ament_add_gtest(TestCaveFullRayScene test/TestCaveFullRayScene.cpp)
  if(TARGET TestCaveFullRayScene)
    target_compile_definitions(TestCaveFullRayScene PRIVATE
      CAVE_FULL_RAY_SCENE_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/cave_full_ray_scene.yaml"
    )
    target_link_libraries(TestCaveFullRayScene
      perception_local_map_core
      perception_local_map_octomap
      cave_world::cave_geometry
      drone_scanner::drone_lidar
      yaml-cpp::yaml-cpp
      ${OCTOMAP_LIBRARIES}
    )
  endif()

  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(
    test_cave_full_ray_scene_assets
    test/test_cave_full_ray_scene_assets.py
    TIMEOUT 30
  )
  add_launch_test(test/test_local_map_integration.py TIMEOUT 90)
  add_launch_test(test/test_map_update_closed_loop.py TIMEOUT 60)
  add_launch_test(test/test_local_map_invalid_config.py TIMEOUT 30)
  add_launch_test(test/test_cave_full_ray_scene_integration.py TIMEOUT 45)
  add_launch_test(test/test_cave_full_ray_scene_rviz_smoke.py TIMEOUT 30)
endif()

ament_export_include_directories(include)
ament_export_dependencies(perception_core perception_map_update Eigen3 octomap Threads)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
