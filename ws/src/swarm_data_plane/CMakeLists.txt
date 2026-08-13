cmake_minimum_required(VERSION 3.8)
project(swarm_data_plane)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenSSL REQUIRED)
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
if(TARGET perception_map_update::octomap)
  set(PERCEPTION_MAP_UPDATE_OCTOMAP_TARGET perception_map_update::octomap)
else()
  set(PERCEPTION_MAP_UPDATE_OCTOMAP_TARGET
    perception_map_update::perception_map_update_octomap
  )
endif()
if(NOT TARGET perception_interfaces__rosidl_typesupport_cpp)
  find_package(perception_interfaces REQUIRED)
endif()
if(NOT TARGET swarm_data_interfaces__rosidl_typesupport_cpp)
  find_package(swarm_data_interfaces REQUIRED)
endif()
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

set(SWARM_DATA_PLANE_CORE_HEADERS
  include/swarm_data_plane/AggregateContract.hpp
  include/swarm_data_plane/DataPlaneTypes.hpp
  include/swarm_data_plane/MapUpdateIngress.hpp
  include/swarm_data_plane/RoutedMapValidator.hpp
  include/swarm_data_plane/RoutedResync.hpp
  include/swarm_data_plane/TrustValidator.hpp
)

set(SWARM_DATA_PLANE_ROS_HEADERS
  include/swarm_data_plane/ros/AggregateConversions.hpp
  include/swarm_data_plane/ros/QosProfiles.hpp
  include/swarm_data_plane/ros/RoutedMapConversions.hpp
  include/swarm_data_plane/ros/RoutedResyncConversions.hpp
)

add_library(swarm_data_plane_core STATIC
  src/AggregateContract.cpp
  src/DataPlaneTypes.cpp
  src/MapUpdateIngress.cpp
  src/RoutedMapValidator.cpp
  src/RoutedResync.cpp
  src/TrustValidator.cpp
)
target_sources(swarm_data_plane_core PRIVATE ${SWARM_DATA_PLANE_CORE_HEADERS})
add_library(${PROJECT_NAME}::core ALIAS swarm_data_plane_core)
add_library(${PROJECT_NAME}::swarm_data_plane_core ALIAS swarm_data_plane_core)
target_include_directories(swarm_data_plane_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(swarm_data_plane_core PUBLIC cxx_std_17)
target_link_libraries(swarm_data_plane_core
  PUBLIC ${PERCEPTION_MAP_UPDATE_CORE_TARGET}
  PRIVATE OpenSSL::Crypto
)

add_library(swarm_data_plane_ros STATIC
  src/AggregateConversions.cpp
  src/QosProfiles.cpp
  src/RoutedMapConversions.cpp
  src/RoutedResyncConversions.cpp
)
target_sources(swarm_data_plane_ros PRIVATE ${SWARM_DATA_PLANE_ROS_HEADERS})
add_library(${PROJECT_NAME}::ros ALIAS swarm_data_plane_ros)
add_library(${PROJECT_NAME}::swarm_data_plane_ros ALIAS swarm_data_plane_ros)
target_include_directories(swarm_data_plane_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(swarm_data_plane_ros PUBLIC cxx_std_17)
target_link_libraries(swarm_data_plane_ros PUBLIC
  swarm_data_plane_core
  ${PERCEPTION_MAP_UPDATE_ROS_TARGET}
)
if(TARGET perception_interfaces__rosidl_typesupport_cpp)
  target_link_libraries(swarm_data_plane_ros PUBLIC
    perception_interfaces__rosidl_typesupport_cpp
  )
endif()
if(TARGET swarm_data_interfaces__rosidl_typesupport_cpp)
  target_link_libraries(swarm_data_plane_ros PUBLIC
    swarm_data_interfaces__rosidl_typesupport_cpp
  )
endif()
ament_target_dependencies(swarm_data_plane_ros PUBLIC rclcpp)
if(perception_interfaces_FOUND)
  ament_target_dependencies(swarm_data_plane_ros PUBLIC perception_interfaces)
endif()
if(swarm_data_interfaces_FOUND)
  ament_target_dependencies(swarm_data_plane_ros PUBLIC swarm_data_interfaces)
endif()

add_executable(map_update_route_source_node
  src/MapUpdateRouteSourceNode.cpp
)
target_link_libraries(map_update_route_source_node
  swarm_data_plane_core
  swarm_data_plane_ros
)
ament_target_dependencies(map_update_route_source_node rclcpp)

add_executable(map_update_route_receiver_node
  src/MapUpdateRouteReceiverNode.cpp
)
target_link_libraries(map_update_route_receiver_node
  swarm_data_plane_core
  swarm_data_plane_ros
)
ament_target_dependencies(map_update_route_receiver_node rclcpp)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS swarm_data_plane_core swarm_data_plane_ros
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS
  map_update_route_source_node
  map_update_route_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(octomap_msgs REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(TestDataPlaneCore test/TestDataPlaneCore.cpp)
  if(TARGET TestDataPlaneCore)
    target_link_libraries(TestDataPlaneCore swarm_data_plane_core)
  endif()
  ament_add_gtest(TestRoutedMapConversions test/TestRoutedMapConversions.cpp)
  if(TARGET TestRoutedMapConversions)
    target_link_libraries(TestRoutedMapConversions
      swarm_data_plane_core
      swarm_data_plane_ros
    )
  endif()
  add_library(swarm_data_plane_logical_link_fixture STATIC
    test/LogicalLinkAdapter.cpp
  )
  target_sources(swarm_data_plane_logical_link_fixture PRIVATE
    test/LogicalLinkAdapter.hpp
  )
  target_include_directories(swarm_data_plane_logical_link_fixture PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  target_link_libraries(swarm_data_plane_logical_link_fixture
    PUBLIC swarm_data_plane_core
  )
  ament_add_gtest(TestLogicalLinkAdapter test/TestLogicalLinkAdapter.cpp)
  if(TARGET TestLogicalLinkAdapter)
    target_link_libraries(TestLogicalLinkAdapter
      swarm_data_plane_logical_link_fixture
    )
  endif()

  add_executable(map_update_route_producer_fixture
    test/MapUpdateRouteProducerFixture.cpp
  )
  target_link_libraries(map_update_route_producer_fixture
    swarm_data_plane_core
    swarm_data_plane_ros
  )
  ament_target_dependencies(map_update_route_producer_fixture rclcpp)

  add_executable(map_update_route_link_fixture
    test/MapUpdateRouteLinkFixture.cpp
  )
  target_link_libraries(map_update_route_link_fixture
    swarm_data_plane_logical_link_fixture
    swarm_data_plane_ros
  )
  ament_target_dependencies(map_update_route_link_fixture rclcpp)

  add_executable(c4_visualization_fixture
    test/C4VisualizationFixture.cpp
  )
  target_link_libraries(c4_visualization_fixture
    swarm_data_plane_core
  )
  ament_target_dependencies(c4_visualization_fixture
    rclcpp
    visualization_msgs
  )

  add_executable(c4_cave_receiver_fixture
    test/C4CaveReceiverFixture.cpp
  )
  target_link_libraries(c4_cave_receiver_fixture
    swarm_data_plane_logical_link_fixture
    swarm_data_plane_ros
    ${PERCEPTION_MAP_UPDATE_OCTOMAP_TARGET}
  )
  ament_target_dependencies(c4_cave_receiver_fixture
    geometry_msgs
    nav_msgs
    octomap_msgs
    perception_interfaces
    rclcpp
    swarm_data_interfaces
    visualization_msgs
  )

  add_executable(c4_resource_profile_source
    test/C4ResourceProfileSource.cpp
  )
  target_link_libraries(c4_resource_profile_source
    swarm_data_plane_core
    swarm_data_plane_ros
  )
  ament_target_dependencies(c4_resource_profile_source rclcpp)

  add_executable(c4_resource_profile_receiver
    test/C4ResourceProfileReceiver.cpp
  )
  target_link_libraries(c4_resource_profile_receiver
    swarm_data_plane_core
    swarm_data_plane_ros
  )
  ament_target_dependencies(c4_resource_profile_receiver rclcpp)

  install(TARGETS
    map_update_route_producer_fixture
    map_update_route_link_fixture
    c4_visualization_fixture
    c4_cave_receiver_fixture
    c4_resource_profile_source
    c4_resource_profile_receiver
    DESTINATION lib/${PROJECT_NAME}
  )
  add_test(
    NAME c4_resource_profile_smoke
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/run_c4_resource_profile.py
      --source $<TARGET_FILE:c4_resource_profile_source>
      --receiver $<TARGET_FILE:c4_resource_profile_receiver>
      --source-count 2
      --cells-per-source 1000
      --initial-cells-per-source 1000
      --delta-operations 64
      --rate-hz-per-source 5
      --warmup-s 2
      --window-s 5
      --sample-period-s 1
      --domain-id 184
  )
  set_tests_properties(c4_resource_profile_smoke PROPERTIES TIMEOUT 30)
  add_test(
    NAME c4_resource_profile_expanding_smoke
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/run_c4_resource_profile.py
      --source $<TARGET_FILE:c4_resource_profile_source>
      --receiver $<TARGET_FILE:c4_resource_profile_receiver>
      --workload-mode expanding
      --source-count 2
      --cells-per-source 4000
      --initial-cells-per-source 1000
      --delta-operations 256
      --rate-hz-per-source 10
      --warmup-s 1
      --window-s 3
      --sample-period-s 1
      --domain-id 186
  )
  set_tests_properties(c4_resource_profile_expanding_smoke PROPERTIES TIMEOUT 30)
  add_test(
    NAME c4_resource_profile_keyframe_replacement_smoke
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/run_c4_resource_profile.py
      --source $<TARGET_FILE:c4_resource_profile_source>
      --receiver $<TARGET_FILE:c4_resource_profile_receiver>
      --workload-mode keyframe-replacement
      --source-count 2
      --cells-per-source 4000
      --initial-cells-per-source 4000
      --delta-operations 256
      --rate-hz-per-source 2
      --warmup-s 1
      --window-s 3
      --sample-period-s 1
      --domain-id 187
  )
  set_tests_properties(c4_resource_profile_keyframe_replacement_smoke PROPERTIES TIMEOUT 30)
  add_launch_test(
    test/test_swarm_data_plane_closed_loop.py
    TIMEOUT 45
  )
  add_launch_test(
    test/test_c4_visualization.py
    TIMEOUT 30
  )
  add_launch_test(
    test/test_c4_cave_visual_validation.py
    TIMEOUT 45
  )
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  perception_map_update perception_interfaces swarm_data_interfaces rclcpp OpenSSL
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
