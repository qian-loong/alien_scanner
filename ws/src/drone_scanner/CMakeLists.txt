cmake_minimum_required(VERSION 3.8)
project(drone_scanner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cave_world REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(drone_trajectory STATIC
        src/LineTrajectory.cpp
        src/PoseSegmentTrajectory.cpp
)
target_include_directories(drone_trajectory PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

add_library(drone_lidar STATIC
        src/FakeLidar.cpp
        src/AltitudeAdapter.cpp
)
target_include_directories(drone_lidar PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(drone_lidar cave_world::cave_geometry)
ament_target_dependencies(drone_lidar cave_world)

add_library(drone_accumulator STATIC
        src/RigidTransform.cpp
        src/PointCloudAccumulator.cpp
)
target_include_directories(drone_accumulator PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

install(
        FILES
        include/drone_scanner/Point3f.hpp
        include/drone_scanner/RigidTransform.hpp
        include/drone_scanner/PointCloudAccumulator.hpp
        DESTINATION include/drone_scanner
)
install(
        TARGETS drone_accumulator
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        FILES
        include/drone_scanner/FakeLidar.hpp
        include/drone_scanner/AltitudeAdapter.hpp
        DESTINATION include/drone_scanner
)
install(
        TARGETS drone_lidar
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        FILES
        include/drone_scanner/ITrajectory.hpp
        include/drone_scanner/Pose3D.hpp
        include/drone_scanner/LineTrajectory.hpp
        include/drone_scanner/PoseSegmentTrajectory.hpp
        DESTINATION include/drone_scanner
)
install(
        TARGETS drone_trajectory
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

add_executable(fake_odom
        src/FakeOdomNode.cpp
        src/FakeOdomMain.cpp
)
target_include_directories(fake_odom PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(fake_odom drone_trajectory drone_lidar)
ament_target_dependencies(fake_odom rclcpp nav_msgs geometry_msgs sensor_msgs tf2_ros)

install(
        TARGETS fake_odom
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(fake_lidar
        src/FakeLidarNode.cpp
        src/FakeLidarMain.cpp
        src/CaveFieldFromParameters.cpp
)
target_include_directories(fake_lidar PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(fake_lidar drone_lidar cave_world::cave_geometry)
ament_target_dependencies(fake_lidar rclcpp nav_msgs sensor_msgs tf2_ros cave_world)

install(
        TARGETS fake_lidar
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(scan_accumulator
        src/ScanAccumulatorNode.cpp
        src/ScanAccumulatorMain.cpp
)
target_include_directories(scan_accumulator PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(scan_accumulator drone_accumulator)
ament_target_dependencies(scan_accumulator rclcpp sensor_msgs tf2_ros)

install(
        TARGETS scan_accumulator
        DESTINATION lib/${PROJECT_NAME}
)
install(
        DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(TestLineTrajectory test/TestLineTrajectory.cpp)
    if(TARGET TestLineTrajectory)
        target_link_libraries(TestLineTrajectory drone_trajectory)
        target_include_directories(TestLineTrajectory PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    ament_add_gtest(TestPoseSegmentTrajectory test/TestPoseSegmentTrajectory.cpp)
    if(TARGET TestPoseSegmentTrajectory)
        target_link_libraries(TestPoseSegmentTrajectory drone_trajectory)
        target_include_directories(TestPoseSegmentTrajectory PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    ament_add_gtest(TestFakeLidar test/TestFakeLidar.cpp)
    if(TARGET TestFakeLidar)
        target_link_libraries(TestFakeLidar drone_lidar)
        ament_target_dependencies(TestFakeLidar cave_world)
        target_include_directories(TestFakeLidar PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    ament_add_gtest(TestAltitudeAdapter test/TestAltitudeAdapter.cpp)
    if(TARGET TestAltitudeAdapter)
        target_link_libraries(TestAltitudeAdapter drone_lidar)
        target_include_directories(TestAltitudeAdapter PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    ament_add_gtest(TestPointCloudAccumulator test/TestPointCloudAccumulator.cpp)
    if(TARGET TestPointCloudAccumulator)
        target_link_libraries(TestPointCloudAccumulator drone_accumulator)
        target_include_directories(TestPointCloudAccumulator PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    find_package(ament_cmake_ros REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)
    function(add_ros_isolated_launch_test path)
        set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
        add_launch_test("${path}" RUNNER "${RUNNER}" TIMEOUT 120 ${ARGN})
    endfunction()
    add_ros_isolated_launch_test(test/test_fake_odom_integration.py)
    add_ros_isolated_launch_test(test/test_fake_odom_goal_integration.py)
    add_ros_isolated_launch_test(test/test_fake_lidar_integration.py)
    add_ros_isolated_launch_test(test/test_cloud_map_integration.py)
    add_ros_isolated_launch_test(test/test_stop_scan_integration.py)
endif()

ament_package()
