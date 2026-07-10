cmake_minimum_required(VERSION 3.8)
project(swarm_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

add_library(swarm_octomap STATIC
        src/OctoMapBuilder.cpp
)
target_include_directories(swarm_octomap PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OCTOMAP_INCLUDE_DIRS}
)
target_link_libraries(swarm_octomap PUBLIC ${OCTOMAP_LIBRARIES})
target_compile_features(swarm_octomap PUBLIC cxx_std_17)

add_library(swarm_exploration STATIC
        src/FrontierExplorationStrategy.cpp
)
target_include_directories(swarm_exploration PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OCTOMAP_INCLUDE_DIRS}
)
target_link_libraries(swarm_exploration PUBLIC ${OCTOMAP_LIBRARIES})
target_compile_features(swarm_exploration PUBLIC cxx_std_17)

install(
        FILES
        include/swarm_controller/Point3f.hpp
        include/swarm_controller/Pose3D.hpp
        include/swarm_controller/RayReturn.hpp
        include/swarm_controller/OctoMapBuilder.hpp
        include/swarm_controller/OctoMapBuilderNode.hpp
        include/swarm_controller/IExplorationStrategy.hpp
        include/swarm_controller/FrontierExplorationStrategy.hpp
        DESTINATION include/swarm_controller
)
install(
        TARGETS swarm_octomap swarm_exploration
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

add_executable(octomap_builder
        src/OctoMapBuilderNode.cpp
        src/OctoMapBuilderMain.cpp
)
target_include_directories(octomap_builder PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(octomap_builder swarm_octomap)
ament_target_dependencies(octomap_builder rclcpp sensor_msgs tf2 tf2_ros octomap_msgs)

install(
        TARGETS octomap_builder
        DESTINATION lib/${PROJECT_NAME}
)
install(
        DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(TestOctoMapBuilder test/TestOctoMapBuilder.cpp)
    if(TARGET TestOctoMapBuilder)
        target_link_libraries(TestOctoMapBuilder swarm_octomap)
        target_include_directories(TestOctoMapBuilder PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    ament_add_gtest(
            TestFrontierExplorationStrategy
            test/TestFrontierExplorationStrategy.cpp
    )
    if(TARGET TestFrontierExplorationStrategy)
        target_link_libraries(TestFrontierExplorationStrategy swarm_exploration)
        target_include_directories(TestFrontierExplorationStrategy PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    find_package(ament_cmake_ros REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test(test/test_octomap_builder_integration.py RUNNER "${RUNNER}" TIMEOUT 120)
endif()

ament_export_include_directories(include)
ament_export_dependencies(octomap)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
