cmake_minimum_required(VERSION 3.8)
project(perception_adapters)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
if(NOT TARGET perception_core::perception_core)
    find_package(perception_core REQUIRED)
endif()
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Adapter library
add_library(${PROJECT_NAME}
        src/laser_scan_adapter.cpp
        src/point_cloud2_adapter.cpp
        src/odometry_adapter.cpp
)

set(PERCEPTION_ADAPTER_HEADER_FILES
        include/perception_adapters/laser_scan_adapter.hpp
        include/perception_adapters/odometry_adapter.hpp
        include/perception_adapters/point_cloud2_adapter.hpp
)
target_sources(${PROJECT_NAME}
        PRIVATE
        ${PERCEPTION_ADAPTER_HEADER_FILES}
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_link_libraries(${PROJECT_NAME}
        PUBLIC perception_core::perception_core
)

ament_target_dependencies(${PROJECT_NAME}
        PUBLIC
        sensor_msgs
        nav_msgs
        geometry_msgs
)

# Install headers
install(DIRECTORY include/
        DESTINATION include
)

# Install library
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

# Testing
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # Unit tests
    ament_add_gtest(test_perception_adapters
            test/test_adapters.cpp
    )
    target_include_directories(test_perception_adapters PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            ${EIGEN3_INCLUDE_DIRS}
    )
    ament_target_dependencies(test_perception_adapters
            sensor_msgs
            nav_msgs
    )
    target_link_libraries(test_perception_adapters ${PROJECT_NAME} perception_core::perception_core)
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
        perception_core
        sensor_msgs
        nav_msgs
        geometry_msgs
        Eigen3
)

ament_package()
