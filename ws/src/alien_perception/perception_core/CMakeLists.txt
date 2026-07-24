cmake_minimum_required(VERSION 3.8)
project(perception_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# Core library (ROS-free)
add_library(${PROJECT_NAME}
        src/health/mapper_health_gate.cpp
)

set(PERCEPTION_CORE_HEADER_FILES
        include/perception_core/health/health_state.hpp
        include/perception_core/health/mapper_health_gate.hpp
        include/perception_core/health/mapper_input_contract.hpp
        include/perception_core/observation/lidar_observation.hpp
        include/perception_core/observation/pose_estimate.hpp
        include/perception_core/observation/sensor_descriptor.hpp
        include/perception_core/types/identity.hpp
        include/perception_core/types/timestamp.hpp
)
target_sources(${PROJECT_NAME}
        PRIVATE
        ${PERCEPTION_CORE_HEADER_FILES}
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

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
    ament_add_gtest(test_perception_core
            test/test_perception_core.cpp
    )
    target_link_libraries(test_perception_core ${PROJECT_NAME})
    target_include_directories(test_perception_core PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            ${EIGEN3_INCLUDE_DIRS}
    )
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)

ament_package()
