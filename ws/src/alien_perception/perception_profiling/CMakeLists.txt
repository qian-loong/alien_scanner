cmake_minimum_required(VERSION 3.8)
project(perception_profiling)

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
if(NOT TARGET perception_local_map::perception_local_map_core)
  find_package(perception_local_map REQUIRED)
endif()
if(NOT TARGET perception_map_update::core AND
   NOT TARGET perception_map_update::perception_map_update_core)
  find_package(perception_map_update REQUIRED)
endif()
if(TARGET perception_map_update::core)
  set(PERCEPTION_MAP_UPDATE_CORE_TARGET perception_map_update::core)
  set(PERCEPTION_MAP_UPDATE_OCTOMAP_TARGET perception_map_update::octomap)
else()
  set(PERCEPTION_MAP_UPDATE_CORE_TARGET
    perception_map_update::perception_map_update_core)
  set(PERCEPTION_MAP_UPDATE_OCTOMAP_TARGET
    perception_map_update::perception_map_update_octomap)
endif()
if(NOT TARGET perception_interfaces__rosidl_typesupport_cpp)
  find_package(perception_interfaces REQUIRED)
endif()
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cave_world REQUIRED)
find_package(drone_scanner REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

set(PERCEPTION_PROFILING_CORE_HEADERS
  include/perception_profiling/ProfileScenario.hpp
  include/perception_profiling/ProfileMapHarness.hpp
  include/perception_profiling/ProfileOracle.hpp
  include/perception_profiling/MapUpdateReplayOracle.hpp
  include/perception_profiling/MapUpdateAcceptanceScenarios.hpp
  include/perception_profiling/ProfileDataSink.hpp
  include/perception_profiling/ChunkLayoutEstimator.hpp
)

add_library(perception_profiling_core STATIC
  src/ProfileScenario.cpp
  src/ProfileMapHarness.cpp
  src/ProfileOracle.cpp
  src/MapUpdateReplayOracle.cpp
  src/MapUpdateAcceptanceScenarios.cpp
  src/ProfileDataSink.cpp
  src/ChunkLayoutEstimator.cpp
)
target_sources(perception_profiling_core PRIVATE ${PERCEPTION_PROFILING_CORE_HEADERS})
add_library(${PROJECT_NAME}::core ALIAS perception_profiling_core)
add_library(${PROJECT_NAME}::perception_profiling_core ALIAS perception_profiling_core)
target_include_directories(perception_profiling_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(perception_profiling_core PUBLIC cxx_std_17)
target_link_libraries(perception_profiling_core PUBLIC
  perception_core::perception_core
  perception_local_map::perception_local_map_core
  perception_local_map::perception_local_map_octomap
  ${PERCEPTION_MAP_UPDATE_CORE_TARGET}
  cave_world::cave_geometry
  drone_scanner::drone_lidar
  yaml-cpp::yaml-cpp
)

add_library(perception_profiling_ros STATIC
  src/ProfileRosConversions.cpp
)
target_sources(perception_profiling_ros PRIVATE src/ProfileRosConversions.hpp)
target_include_directories(perception_profiling_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_link_libraries(perception_profiling_ros PUBLIC perception_profiling_core)
if(TARGET perception_interfaces__rosidl_typesupport_cpp)
  target_link_libraries(perception_profiling_ros PUBLIC
    perception_interfaces__rosidl_typesupport_cpp)
endif()
ament_target_dependencies(perception_profiling_ros PUBLIC
  builtin_interfaces geometry_msgs)
if(perception_interfaces_FOUND)
  ament_target_dependencies(perception_profiling_ros PUBLIC perception_interfaces)
endif()

add_executable(perception_profile_fixture src/ProfileFixtureNode.cpp)
target_link_libraries(perception_profile_fixture
  perception_profiling_core perception_profiling_ros)
ament_target_dependencies(perception_profile_fixture rclcpp tf2_ros geometry_msgs)

add_executable(perception_profile_sink src/ProfileSinkNode.cpp)
target_link_libraries(perception_profile_sink
  perception_profiling_core perception_profiling_ros)
ament_target_dependencies(perception_profile_sink
  rclcpp diagnostic_msgs octomap_msgs)

add_executable(perception_profile_oracle src/ProfileOracleMain.cpp)
target_link_libraries(perception_profile_oracle
  perception_profiling_core yaml-cpp::yaml-cpp)

add_executable(perception_chunk_layout_estimator src/ChunkLayoutEstimatorMain.cpp)
target_link_libraries(perception_chunk_layout_estimator
  perception_profiling_core yaml-cpp::yaml-cpp)

add_executable(
  map_update_replay_visualization_node
  src/MapUpdateReplayVisualizationNode.cpp
)
target_link_libraries(map_update_replay_visualization_node
  perception_profiling_core
  ${PERCEPTION_MAP_UPDATE_OCTOMAP_TARGET}
)
ament_target_dependencies(map_update_replay_visualization_node
  rclcpp diagnostic_msgs geometry_msgs std_msgs octomap_msgs tf2_ros
  visualization_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS perception_profiling_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS
  perception_profile_fixture
  perception_profile_sink
  perception_profile_oracle
  perception_chunk_layout_estimator
  map_update_replay_visualization_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(TestPerceptionProfiling test/TestPerceptionProfiling.cpp)
  if(TARGET TestPerceptionProfiling)
    target_link_libraries(TestPerceptionProfiling perception_profiling_core)
  endif()

  add_launch_test(test/test_profile_integration.py TIMEOUT 35)
  add_launch_test(test/test_canonical_equivalence.py TIMEOUT 45)
  add_launch_test(test/test_map_update_replay_visualization.py TIMEOUT 45)
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  perception_core perception_local_map perception_map_update cave_world drone_scanner
  yaml_cpp_vendor)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
