cmake_minimum_required(VERSION 3.8)
project(perception_input_node)

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
if(NOT TARGET perception_adapters::perception_adapters)
    find_package(perception_adapters REQUIRED)
endif()
if(NOT TARGET perception_interfaces__rosidl_typesupport_cpp)
    find_package(perception_interfaces REQUIRED)
endif()
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(sensor_session_manager
        src/SensorSessionManager.cpp
)
target_sources(sensor_session_manager
        PRIVATE
        include/perception_input_node/SensorSessionManager.hpp
)

target_include_directories(sensor_session_manager
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(sensor_session_manager PUBLIC cxx_std_17)
target_link_libraries(sensor_session_manager PUBLIC perception_core::perception_core)

add_executable(perception_input_node
        src/PerceptionInputNode.cpp
)

target_include_directories(perception_input_node
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${EIGEN3_INCLUDE_DIRS}
)

target_compile_features(perception_input_node PRIVATE cxx_std_17)
target_link_libraries(perception_input_node
        sensor_session_manager
        perception_core::perception_core
        perception_adapters::perception_adapters
)
if(TARGET perception_interfaces__rosidl_typesupport_cpp)
    target_link_libraries(perception_input_node
            perception_interfaces__rosidl_typesupport_cpp
    )
endif()
ament_target_dependencies(perception_input_node
        rclcpp
        sensor_msgs
        nav_msgs
        geometry_msgs
        tf2_msgs
)
if(perception_interfaces_FOUND)
    ament_target_dependencies(perception_input_node perception_interfaces)
endif()

install(DIRECTORY include/
        DESTINATION include
)

install(TARGETS sensor_session_manager
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS perception_input_node
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_sensor_session_manager
            test/TestSensorSessionManager.cpp
    )
    target_link_libraries(test_sensor_session_manager
            sensor_session_manager
            perception_core::perception_core
    )
endif()

ament_package()
