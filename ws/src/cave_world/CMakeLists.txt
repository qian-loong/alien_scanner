cmake_minimum_required(VERSION 3.8)
project(cave_world)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(cave_geometry STATIC
        src/ProceduralCaveField.cpp
        src/TreeCaveField.cpp
        src/CaveFieldFactory.cpp
)
target_include_directories(cave_geometry PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(cave_geometry PUBLIC cxx_std_17)

install(
        FILES
        include/cave_world/ICaveField.hpp
        include/cave_world/ProceduralCaveField.hpp
        include/cave_world/TreeCaveField.hpp
        include/cave_world/CaveFieldFactory.hpp
        DESTINATION include/cave_world
)
install(
        DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)
install(
        TARGETS cave_geometry
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

add_executable(cave_publisher
        src/CavePublisherNode.cpp
        src/CavePublisherMain.cpp
)
target_include_directories(cave_publisher PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(cave_publisher cave_geometry)
ament_target_dependencies(cave_publisher rclcpp sensor_msgs)

install(
        TARGETS cave_publisher
        DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(TestProceduralCaveField test/TestProceduralCaveField.cpp)
    if(TARGET TestProceduralCaveField)
        target_link_libraries(TestProceduralCaveField cave_geometry)
        target_include_directories(TestProceduralCaveField PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    ament_add_gtest(TestTreeCaveField test/TestTreeCaveField.cpp)
    if(TARGET TestTreeCaveField)
        target_link_libraries(TestTreeCaveField cave_geometry)
        target_include_directories(TestTreeCaveField PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    find_package(ament_cmake_ros REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)
    function(add_ros_isolated_launch_test path)
        set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
        add_launch_test("${path}" RUNNER "${RUNNER}" TIMEOUT 120 ${ARGN})
    endfunction()
    add_ros_isolated_launch_test(test/test_cave_publisher_integration.py)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
