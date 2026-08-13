cmake_minimum_required(VERSION 3.8)
project(swarm_data_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(perception_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AggregateManifest.msg"
  "msg/AggregateMapUpdate.msg"
  "msg/ContributorRevision.msg"
  "msg/DeliveryAck.msg"
  "msg/LinkDiagnostic.msg"
  "msg/ResyncAck.msg"
  "msg/ResyncIntent.msg"
  "msg/RoutedMapUpdate.msg"
  "srv/RequestRoutedMapResync.srv"
  DEPENDENCIES perception_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
