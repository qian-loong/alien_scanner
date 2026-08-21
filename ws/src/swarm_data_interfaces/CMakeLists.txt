cmake_minimum_required(VERSION 3.8)
project(swarm_data_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(perception_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ApplyRoleTransition.action"
  "msg/AggregateManifest.msg"
  "msg/AggregateMapUpdate.msg"
  "msg/CapabilityEvidence.msg"
  "msg/CapabilityRegistration.msg"
  "msg/ComponentRegistration.msg"
  "msg/ContributorRevision.msg"
  "msg/DeliveryAck.msg"
  "msg/GraphEdge.msg"
  "msg/LinkDiagnostic.msg"
  "msg/LinkDescriptor.msg"
  "msg/MemberRecord.msg"
  "msg/ResyncAck.msg"
  "msg/ResyncIntent.msg"
  "msg/RoutedMapUpdate.msg"
  "msg/RouteDescriptor.msg"
  "msg/RouteHop.msg"
  "msg/RoleAssignment.msg"
  "msg/RoleSnapshot.msg"
  "msg/RoleTransitionAck.msg"
  "msg/RoleTransitionDescriptor.msg"
  "msg/SensorDescriptorIdentity.msg"
  "msg/ServiceAssignment.msg"
  "msg/ServiceBudget.msg"
  "msg/ServiceHealthEvidence.msg"
  "msg/TopologySnapshot.msg"
  "msg/VehicleIdentity.msg"
  "srv/RequestRoutedMapResync.srv"
  DEPENDENCIES perception_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
