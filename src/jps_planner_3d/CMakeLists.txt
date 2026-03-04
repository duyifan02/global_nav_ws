cmake_minimum_required(VERSION 3.8)
project(jps_planner_3d)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}_core SHARED
  src/jps_3d.cpp
  src/voxel_map.cpp
  src/pcd_voxel_loader.cpp
)

ament_target_dependencies(${PROJECT_NAME}_core
  geometry_msgs
  nav_msgs
)

target_compile_definitions(${PROJECT_NAME}_core
  PRIVATE "JPS_PLANNER_3D_BUILDING_DLL"
)

add_executable(jps_planner_node
  src/jps_planner_node.cpp
)

ament_target_dependencies(jps_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  nav_interfaces
  rosidl_default_runtime
)

target_link_libraries(jps_planner_node
  ${PROJECT_NAME}_core
)

# nav_interfaces exports its generated typesupport targets under
# nav_interfaces_TARGETS__rosidl_typesupport_cpp (e.g. nav_interfaces::nav_interfaces__rosidl_typesupport_cpp)
if(nav_interfaces_TARGETS__rosidl_typesupport_cpp)
  target_link_libraries(jps_planner_node
    ${nav_interfaces_TARGETS__rosidl_typesupport_cpp}
  )
else()
  message(FATAL_ERROR "nav_interfaces does not export C++ typesupport targets; cannot link service typesupport")
endif()

target_include_directories(jps_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(jps_planner_node PUBLIC cxx_std_17)

target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

install(TARGETS
  ${PROJECT_NAME}_core
  jps_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY doc
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
