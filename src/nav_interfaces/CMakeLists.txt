cmake_minimum_required(VERSION 3.8)
project(nav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(srv_files
  srv/SetVoxelMap.srv
  srv/PlanPath.srv
  srv/GetDistanceField.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES geometry_msgs nav_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
