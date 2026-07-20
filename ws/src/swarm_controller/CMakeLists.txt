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
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(swarm_controller_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(drone_scanner REQUIRED)
find_package(cave_world REQUIRED)

add_library(swarm_octomap STATIC
        src/OctoMapBuilder.cpp
        src/OctoMapMerger.cpp
)
target_include_directories(swarm_octomap PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OCTOMAP_INCLUDE_DIRS}
)
target_link_libraries(swarm_octomap PUBLIC ${OCTOMAP_LIBRARIES})
target_compile_features(swarm_octomap PUBLIC cxx_std_17)

add_library(swarm_exploration STATIC
        src/GlobalFrontierDetector.cpp
        src/GlobalTaskAllocator.cpp
        src/FrontierExplorationStrategy.cpp
        src/KnownFreePathChecker.cpp
        src/PeerStateTracker.cpp
        src/PlanningSnapshotGuard.cpp
        src/TaskLeaseTracker.cpp
        src/SingleDroneExplorer.cpp
)
target_include_directories(swarm_exploration PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OCTOMAP_INCLUDE_DIRS}
)
target_link_libraries(swarm_exploration PUBLIC ${OCTOMAP_LIBRARIES})
target_compile_features(swarm_exploration PUBLIC cxx_std_17)

add_library(swarm_geometry_demo STATIC
        src/FrontierGeometryDemo.cpp
)
target_include_directories(swarm_geometry_demo PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
        swarm_geometry_demo PUBLIC
        swarm_exploration
        swarm_octomap
        drone_scanner::drone_lidar
)
target_compile_features(swarm_geometry_demo PUBLIC cxx_std_17)

add_library(swarm_component_audit_replay STATIC
        src/FrontierComponentAuditReplay.cpp
)
target_include_directories(swarm_component_audit_replay PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(swarm_component_audit_replay PUBLIC cxx_std_17)

install(
        FILES
        include/swarm_controller/Point3f.hpp
        include/swarm_controller/Pose3D.hpp
        include/swarm_controller/RayReturn.hpp
        include/swarm_controller/OctoMapBuilder.hpp
        include/swarm_controller/OctoMapMerger.hpp
        include/swarm_controller/OctoMapBuilderNode.hpp
        include/swarm_controller/IExplorationStrategy.hpp
        include/swarm_controller/FrontierExplorationStrategy.hpp
        include/swarm_controller/KnownFreePathChecker.hpp
        include/swarm_controller/PeerStateTracker.hpp
        include/swarm_controller/PlanningSnapshotGuard.hpp
        include/swarm_controller/ExplorationDiagnostics.hpp
        include/swarm_controller/ExplorationTask.hpp
        include/swarm_controller/GlobalFrontierDetector.hpp
        include/swarm_controller/GlobalTaskAllocator.hpp
        include/swarm_controller/LatestSnapshotSlot.hpp
        include/swarm_controller/TaskLeaseTracker.hpp
        include/swarm_controller/SingleDroneExplorer.hpp
        include/swarm_controller/SingleDroneExplorerNode.hpp
        include/swarm_controller/FrontierGeometryDemo.hpp
        include/swarm_controller/FrontierComponentAuditReplay.hpp
        DESTINATION include/swarm_controller
)
install(
        TARGETS swarm_octomap swarm_exploration swarm_geometry_demo
                swarm_component_audit_replay
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

add_executable(global_map_merger
        src/GlobalMapMergerNode.cpp
)
target_include_directories(global_map_merger PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(global_map_merger swarm_octomap)
ament_target_dependencies(global_map_merger rclcpp diagnostic_msgs octomap_msgs)

add_executable(single_drone_explorer
        src/SingleDroneExplorerNode.cpp
        src/SingleDroneExplorerMain.cpp
)
target_include_directories(single_drone_explorer PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(single_drone_explorer swarm_exploration)
ament_target_dependencies(
        single_drone_explorer
        rclcpp diagnostic_msgs geometry_msgs nav_msgs octomap_msgs tf2 tf2_geometry_msgs tf2_ros
        visualization_msgs swarm_controller_interfaces
)

add_executable(global_task_allocator
        src/GlobalTaskAllocatorNode.cpp
)
target_include_directories(global_task_allocator PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(global_task_allocator swarm_exploration)
ament_target_dependencies(
        global_task_allocator
        rclcpp diagnostic_msgs geometry_msgs nav_msgs octomap_msgs tf2 tf2_geometry_msgs tf2_ros
        visualization_msgs swarm_controller_interfaces
)

add_executable(frontier_geometry_demo
        src/FrontierGeometryDemoNode.cpp
)
target_include_directories(frontier_geometry_demo PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(frontier_geometry_demo swarm_geometry_demo)
ament_target_dependencies(
        frontier_geometry_demo
        rclcpp geometry_msgs visualization_msgs
)

add_executable(frontier_component_audit_replay
        src/FrontierComponentAuditReplayNode.cpp
)
target_include_directories(frontier_component_audit_replay PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(frontier_component_audit_replay swarm_component_audit_replay)
ament_target_dependencies(
        frontier_component_audit_replay
        rclcpp geometry_msgs visualization_msgs
)

install(
        TARGETS octomap_builder global_map_merger single_drone_explorer global_task_allocator
                frontier_geometry_demo frontier_component_audit_replay
        DESTINATION lib/${PROJECT_NAME}
)
install(
        DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(rosbag2_cpp REQUIRED)
    find_package(rosbag2_storage REQUIRED)
    find_package(std_msgs REQUIRED)

    add_library(
            global_frontier_bag_analyzer_support STATIC
            test/GlobalFrontierBagAnalyzer.cpp
    )
    target_include_directories(
            global_frontier_bag_analyzer_support PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/test
    )
    target_link_libraries(global_frontier_bag_analyzer_support swarm_exploration)
    ament_target_dependencies(
            global_frontier_bag_analyzer_support
            rclcpp octomap_msgs rosbag2_cpp rosbag2_storage
    )

    add_executable(
            global_frontier_bag_analyzer
            test/GlobalFrontierBagAnalyzerMain.cpp
    )
    target_link_libraries(
            global_frontier_bag_analyzer
            global_frontier_bag_analyzer_support
    )

    add_executable(
            global_map_late_subscriber_probe
            test/GlobalMapLateSubscriberProbe.cpp
    )
    target_link_libraries(global_map_late_subscriber_probe swarm_octomap)
    ament_target_dependencies(
            global_map_late_subscriber_probe
            rclcpp diagnostic_msgs octomap_msgs std_msgs
    )
    install(
            TARGETS global_map_late_subscriber_probe
            DESTINATION lib/${PROJECT_NAME}
    )

    add_executable(
            global_task_allocator_integration_probe
            test/GlobalTaskAllocatorIntegrationProbe.cpp
    )
    target_link_libraries(global_task_allocator_integration_probe swarm_exploration)
    ament_target_dependencies(
            global_task_allocator_integration_probe
            rclcpp diagnostic_msgs nav_msgs octomap_msgs std_msgs
    )
    install(
            TARGETS global_task_allocator_integration_probe
            DESTINATION lib/${PROJECT_NAME}
    )

    ament_add_gtest(TestOctoMapBuilder test/TestOctoMapBuilder.cpp)
    if(TARGET TestOctoMapBuilder)
        target_link_libraries(TestOctoMapBuilder swarm_octomap)
        target_include_directories(TestOctoMapBuilder PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    ament_add_gtest(TestOctoMapMerger test/TestOctoMapMerger.cpp)
    if(TARGET TestOctoMapMerger)
        target_link_libraries(TestOctoMapMerger swarm_octomap)
        target_include_directories(TestOctoMapMerger PUBLIC
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

    ament_add_gtest(TestKnownFreePathChecker test/TestKnownFreePathChecker.cpp)
    if(TARGET TestKnownFreePathChecker)
        target_link_libraries(TestKnownFreePathChecker swarm_exploration)
        target_include_directories(TestKnownFreePathChecker PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    ament_add_gtest(TestPeerStateTracker test/TestPeerStateTracker.cpp)
    if(TARGET TestPeerStateTracker)
        target_link_libraries(TestPeerStateTracker swarm_exploration)
        target_include_directories(TestPeerStateTracker PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    ament_add_gtest(TestPlanningSnapshotGuard test/TestPlanningSnapshotGuard.cpp)
    if(TARGET TestPlanningSnapshotGuard)
        target_link_libraries(TestPlanningSnapshotGuard swarm_exploration)
        target_include_directories(TestPlanningSnapshotGuard PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    ament_add_gtest(TestTaskLeaseTracker test/TestTaskLeaseTracker.cpp)
    if(TARGET TestTaskLeaseTracker)
        target_link_libraries(TestTaskLeaseTracker swarm_exploration)
        target_include_directories(TestTaskLeaseTracker PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    ament_add_gtest(TestGlobalFrontierDetector test/TestGlobalFrontierDetector.cpp)
    if(TARGET TestGlobalFrontierDetector)
        target_link_libraries(TestGlobalFrontierDetector swarm_exploration)
        target_include_directories(TestGlobalFrontierDetector PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    ament_add_gtest(TestFrontierGeometryDemo test/TestFrontierGeometryDemo.cpp)
    if(TARGET TestFrontierGeometryDemo)
        target_link_libraries(TestFrontierGeometryDemo swarm_geometry_demo)
        target_include_directories(TestFrontierGeometryDemo PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    ament_add_gtest(
            TestFrontierComponentAuditReplay
            test/TestFrontierComponentAuditReplay.cpp
    )
    if(TARGET TestFrontierComponentAuditReplay)
        target_link_libraries(
                TestFrontierComponentAuditReplay
                swarm_component_audit_replay
        )
        target_include_directories(TestFrontierComponentAuditReplay PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
        target_compile_definitions(
                TestFrontierComponentAuditReplay PRIVATE
                SWARM_CONTROLLER_TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
        )
    endif()

    ament_add_gtest(
            TestGlobalFrontierBagAnalyzer
            test/TestGlobalFrontierBagAnalyzer.cpp
    )
    if(TARGET TestGlobalFrontierBagAnalyzer)
        target_link_libraries(
                TestGlobalFrontierBagAnalyzer
                global_frontier_bag_analyzer_support
        )
        ament_target_dependencies(
                TestGlobalFrontierBagAnalyzer
                rclcpp octomap_msgs rosbag2_cpp rosbag2_storage std_msgs
        )
    endif()

    ament_add_gtest(TestGlobalTaskAllocator test/TestGlobalTaskAllocator.cpp)
    if(TARGET TestGlobalTaskAllocator)
        target_link_libraries(TestGlobalTaskAllocator swarm_exploration)
        target_include_directories(TestGlobalTaskAllocator PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    ament_add_gtest(TestLatestSnapshotSlot test/TestLatestSnapshotSlot.cpp)
    if(TARGET TestLatestSnapshotSlot)
        target_include_directories(TestLatestSnapshotSlot PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    ament_add_gtest(TestMapPipelineTiming test/TestMapPipelineTiming.cpp)
    if(TARGET TestMapPipelineTiming)
        target_include_directories(TestMapPipelineTiming PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    ament_add_gtest(TestSingleDroneExplorer test/TestSingleDroneExplorer.cpp)
    if(TARGET TestSingleDroneExplorer)
        target_link_libraries(TestSingleDroneExplorer swarm_exploration)
        target_include_directories(TestSingleDroneExplorer PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    ament_add_gtest(TestYawSweepCoverage test/TestYawSweepCoverage.cpp)
    if(TARGET TestYawSweepCoverage)
        target_link_libraries(TestYawSweepCoverage swarm_exploration swarm_octomap)
        target_include_directories(TestYawSweepCoverage PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
    endif()

    find_package(ament_cmake_ros REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test(test/test_octomap_builder_integration.py RUNNER "${RUNNER}" TIMEOUT 120)
    add_launch_test(
            test/test_octomap_observation_stamp_integration.py
            RUNNER "${RUNNER}"
            TIMEOUT 120
    )
    add_launch_test(
            test/test_frontier_geometry_demo_integration.py
            RUNNER "${RUNNER}"
            TIMEOUT 120
    )
    add_launch_test(
            test/test_single_drone_exploration_integration.py
            RUNNER "${RUNNER}"
            TIMEOUT 120
    )
    add_launch_test(
            test/test_multi_drone_sensing_integration.py
            RUNNER "${RUNNER}"
            TIMEOUT 180
    )
    add_launch_test(
            test/test_single_peer_wiring_integration.py
            RUNNER "${RUNNER}"
            TIMEOUT 120
    )
    add_launch_test(
            test/test_multi_drone_exploration_integration.py
            RUNNER "${RUNNER}"
            TIMEOUT 180
    )
    add_launch_test(
            test/test_global_map_merger_integration.py
            RUNNER "${RUNNER}"
            TIMEOUT 120
    )
    add_launch_test(
            test/test_global_task_allocator_integration.py
            RUNNER "${RUNNER}"
            TIMEOUT 120
    )
    add_launch_test(
            test/test_global_task_allocator_stale_replay.py
            RUNNER "${RUNNER}"
            TIMEOUT 120
    )
    add_launch_test(
            test/test_global_task_allocator_pipeline.py
            RUNNER "${RUNNER}"
            TIMEOUT 120
    )
    add_launch_test(
            test/test_global_task_allocator_pipeline_load.py
            RUNNER "${RUNNER}"
            TIMEOUT 120
    )
    add_launch_test(
            test/test_global_task_allocator_clock_reset.py
            RUNNER "${RUNNER}"
            TIMEOUT 120
    )
endif()

ament_export_include_directories(include)
ament_export_dependencies(octomap diagnostic_msgs drone_scanner cave_world)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
