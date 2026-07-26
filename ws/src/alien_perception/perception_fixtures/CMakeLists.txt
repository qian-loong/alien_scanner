cmake_minimum_required(VERSION 3.8)
project(perception_fixtures)

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
if(NOT TARGET perception_interfaces__rosidl_typesupport_cpp)
    find_package(perception_interfaces REQUIRED)
endif()
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(fixture_scene
        src/FixtureScene.cpp
        src/RayEvidenceDebugGeometryBuilder.cpp
)
target_sources(fixture_scene
        PRIVATE
        include/perception_fixtures/FixtureScene.hpp
        include/perception_fixtures/RayEvidenceDebugGeometryBuilder.hpp
)
target_include_directories(fixture_scene
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(fixture_scene PUBLIC cxx_std_17)
target_link_libraries(fixture_scene PUBLIC perception_core::perception_core)

add_executable(perception_fixture_publisher
        src/FixturePublisher.cpp
)
target_include_directories(perception_fixture_publisher PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(perception_fixture_publisher fixture_scene)
ament_target_dependencies(perception_fixture_publisher rclcpp sensor_msgs)

add_executable(ray_evidence_debug_node
        src/RayEvidenceDebugNode.cpp
)
target_include_directories(ray_evidence_debug_node PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(ray_evidence_debug_node
        fixture_scene
        perception_core::perception_core
)
if(TARGET perception_interfaces__rosidl_typesupport_cpp)
    target_link_libraries(ray_evidence_debug_node
            perception_interfaces__rosidl_typesupport_cpp
    )
endif()
ament_target_dependencies(ray_evidence_debug_node
        rclcpp
        geometry_msgs
        std_msgs
        visualization_msgs
)
if(perception_interfaces_FOUND)
    ament_target_dependencies(ray_evidence_debug_node perception_interfaces)
endif()

install(DIRECTORY include/
        DESTINATION include
)
install(TARGETS fixture_scene
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)
install(TARGETS perception_fixture_publisher ray_evidence_debug_node
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_fixture_scene test/TestFixtureScene.cpp)
    target_link_libraries(test_fixture_scene fixture_scene)

    find_package(ament_cmake_ros REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/test_perception_input_integration.py TIMEOUT 60)
    add_launch_test(test/test_perception_input_mixed_integration.py TIMEOUT 60)
    add_launch_test(test/test_perception_input_health_integration.py TIMEOUT 90)
    add_launch_test(test/test_perception_input_descriptor_freeze_integration.py TIMEOUT 60)
    add_launch_test(test/test_perception_input_tf_integration.py TIMEOUT 60)
    add_launch_test(test/test_fixture_visualization_integration.py TIMEOUT 30)
    add_launch_test(test/test_perception_input_session_restart_integration.py TIMEOUT 60)
    add_launch_test(test/test_perception_input_performance_integration.py TIMEOUT 90)
    add_launch_test(test/test_perception_input_invalid_ray_evidence.py TIMEOUT 30)
    add_launch_test(test/test_ray_evidence_debug_integration.py TIMEOUT 60)
endif()

ament_package()
