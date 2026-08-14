cmake_minimum_required(VERSION 3.8)
project(perception_map_update)

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
find_package(OpenSSL REQUIRED)
if(NOT TARGET perception_interfaces__rosidl_typesupport_cpp)
  find_package(perception_interfaces REQUIRED)
endif()
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

set(PERCEPTION_MAP_UPDATE_CORE_HEADERS
  include/perception_map_update/CanonicalCodec.hpp
  include/perception_map_update/CellSnapshotStore.hpp
  include/perception_map_update/ContentHasher.hpp
  include/perception_map_update/MapUpdateApplier.hpp
  include/perception_map_update/MapUpdateLimits.hpp
  include/perception_map_update/MapUpdateProducer.hpp
  include/perception_map_update/MapUpdateTypes.hpp
  include/perception_map_update/ResyncStateMachine.hpp
  include/perception_map_update/SnapshotDiffer.hpp
  include/perception_map_update/SpatialChunkLayout.hpp
)

set(PERCEPTION_MAP_UPDATE_ROS_HEADERS
  include/perception_map_update/ros/MapUpdateConversions.hpp
)

set(PERCEPTION_MAP_UPDATE_PARAMETER_HEADERS
  include/perception_map_update/ros/MapUpdateParameters.hpp
)

set(PERCEPTION_MAP_UPDATE_OCTOMAP_HEADERS
  include/perception_map_update/OctoMapViewAdapter.hpp
)

add_library(perception_map_update_core STATIC
  src/CanonicalCodec.cpp
  src/CellSnapshotStore.cpp
  src/ContentHasher.cpp
  src/MapUpdateApplier.cpp
  src/MapUpdateProducer.cpp
  src/MapUpdateTypes.cpp
  src/ResyncStateMachine.cpp
  src/SnapshotDiffer.cpp
  src/SpatialChunkLayout.cpp
)
target_sources(perception_map_update_core PRIVATE ${PERCEPTION_MAP_UPDATE_CORE_HEADERS})
add_library(${PROJECT_NAME}::core ALIAS perception_map_update_core)
add_library(${PROJECT_NAME}::perception_map_update_core ALIAS perception_map_update_core)
target_include_directories(perception_map_update_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(perception_map_update_core PUBLIC cxx_std_17)
target_link_libraries(perception_map_update_core
  PUBLIC perception_core::perception_core
  PRIVATE OpenSSL::Crypto
)

add_library(perception_map_update_ros STATIC
  src/MapUpdateConversions.cpp
)
target_sources(perception_map_update_ros PRIVATE ${PERCEPTION_MAP_UPDATE_ROS_HEADERS})
add_library(${PROJECT_NAME}::ros ALIAS perception_map_update_ros)
add_library(${PROJECT_NAME}::perception_map_update_ros ALIAS perception_map_update_ros)
target_include_directories(perception_map_update_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(perception_map_update_ros PUBLIC cxx_std_17)
target_link_libraries(perception_map_update_ros PUBLIC perception_map_update_core)
if(TARGET perception_interfaces__rosidl_typesupport_cpp)
  target_link_libraries(perception_map_update_ros PUBLIC
    perception_interfaces__rosidl_typesupport_cpp
  )
endif()
ament_target_dependencies(perception_map_update_ros PUBLIC
  builtin_interfaces std_msgs geometry_msgs
)
if(perception_interfaces_FOUND)
  ament_target_dependencies(perception_map_update_ros PUBLIC perception_interfaces)
endif()

add_library(perception_map_update_parameters STATIC
  src/MapUpdateParameters.cpp
)
target_sources(perception_map_update_parameters
  PRIVATE ${PERCEPTION_MAP_UPDATE_PARAMETER_HEADERS}
)
add_library(${PROJECT_NAME}::parameters ALIAS perception_map_update_parameters)
add_library(
  ${PROJECT_NAME}::perception_map_update_parameters
  ALIAS perception_map_update_parameters
)
target_include_directories(perception_map_update_parameters PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(perception_map_update_parameters PUBLIC cxx_std_17)
target_link_libraries(perception_map_update_parameters PUBLIC perception_map_update_core)
ament_target_dependencies(perception_map_update_parameters PUBLIC rclcpp)

add_library(perception_map_update_octomap STATIC
  src/OctoMapViewAdapter.cpp
)
target_sources(perception_map_update_octomap PRIVATE ${PERCEPTION_MAP_UPDATE_OCTOMAP_HEADERS})
add_library(${PROJECT_NAME}::octomap ALIAS perception_map_update_octomap)
add_library(${PROJECT_NAME}::perception_map_update_octomap ALIAS perception_map_update_octomap)
target_include_directories(perception_map_update_octomap PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
)
target_compile_features(perception_map_update_octomap PUBLIC cxx_std_17)
target_link_libraries(perception_map_update_octomap
  PUBLIC perception_map_update_core
  PRIVATE ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(perception_map_update_octomap PUBLIC octomap_msgs)

add_executable(perception_map_update_receiver_node
  src/PerceptionMapUpdateReceiverNode.cpp
)
target_include_directories(perception_map_update_receiver_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OCTOMAP_INCLUDE_DIRS}
)
target_link_libraries(perception_map_update_receiver_node
  perception_map_update_core
  perception_map_update_ros
  perception_map_update_parameters
  perception_map_update_octomap
  ${OCTOMAP_LIBRARIES}
)
if(TARGET perception_interfaces__rosidl_typesupport_cpp)
  target_link_libraries(perception_map_update_receiver_node
    perception_interfaces__rosidl_typesupport_cpp
  )
endif()
ament_target_dependencies(perception_map_update_receiver_node
  rclcpp diagnostic_msgs octomap_msgs
)
if(perception_interfaces_FOUND)
  ament_target_dependencies(perception_map_update_receiver_node perception_interfaces)
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS
    perception_map_update_core
    perception_map_update_ros
    perception_map_update_parameters
    perception_map_update_octomap
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS perception_map_update_receiver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(TestMapUpdateCore test/TestMapUpdateCore.cpp)
  if(TARGET TestMapUpdateCore)
    target_link_libraries(TestMapUpdateCore perception_map_update_core)
  endif()
  ament_add_gtest(TestSpatialChunkLayout test/TestSpatialChunkLayout.cpp)
  if(TARGET TestSpatialChunkLayout)
    target_link_libraries(TestSpatialChunkLayout perception_map_update_core)
  endif()
  ament_add_gtest(TestCellSnapshotStore test/TestCellSnapshotStore.cpp)
  if(TARGET TestCellSnapshotStore)
    target_link_libraries(TestCellSnapshotStore perception_map_update_core)
  endif()
  ament_add_gtest(
    TestMapUpdateStorageConformance
    test/TestMapUpdateStorageConformance.cpp
  )
  if(TARGET TestMapUpdateStorageConformance)
    target_link_libraries(
      TestMapUpdateStorageConformance
      perception_map_update_core
    )
  endif()
  ament_add_gtest(TestMapUpdateConversions test/TestMapUpdateConversions.cpp)
  if(TARGET TestMapUpdateConversions)
    target_link_libraries(TestMapUpdateConversions
      perception_map_update_core
      perception_map_update_ros
      perception_map_update_octomap
      ${OCTOMAP_LIBRARIES}
    )
  endif()
  ament_add_gtest(TestMapUpdateParameters test/TestMapUpdateParameters.cpp)
  if(TARGET TestMapUpdateParameters)
    target_link_libraries(TestMapUpdateParameters perception_map_update_parameters)
  endif()
  ament_add_gtest(
    TestMapUpdateCapacity
    test/TestMapUpdateCapacity.cpp
    TIMEOUT 180
  )
  if(TARGET TestMapUpdateCapacity)
    target_link_libraries(TestMapUpdateCapacity perception_map_update_core)
  endif()
  add_executable(map_update_receiver_fixture test/MapUpdateReceiverFixture.cpp)
  target_link_libraries(map_update_receiver_fixture
    perception_map_update_core
    perception_map_update_ros
  )
  if(TARGET perception_interfaces__rosidl_typesupport_cpp)
    target_link_libraries(map_update_receiver_fixture
      perception_interfaces__rosidl_typesupport_cpp
    )
  endif()
  ament_target_dependencies(map_update_receiver_fixture rclcpp std_msgs)
  if(perception_interfaces_FOUND)
    ament_target_dependencies(map_update_receiver_fixture perception_interfaces)
  endif()
  install(TARGETS map_update_receiver_fixture DESTINATION lib/${PROJECT_NAME})
  add_launch_test(test/test_map_update_receiver_rejection.py TIMEOUT 45)
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  perception_core perception_interfaces builtin_interfaces std_msgs geometry_msgs OpenSSL
  rclcpp octomap octomap_msgs
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
