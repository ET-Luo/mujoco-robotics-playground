cmake_minimum_required(VERSION 3.16)
project(s17_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SurfaceReference.msg" "msg/SurfaceState.msg" "action/FollowSurface.action")
ament_export_dependencies(rosidl_default_runtime)
ament_package()
