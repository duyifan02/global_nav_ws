cmake_minimum_required(VERSION 3.8)
project(topo_nav)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(jps_planner_3d REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# ── Core algorithm library ────────────────────────────────────────────────────
add_library(${PROJECT_NAME}_core SHARED
  src/persistence.cpp
  src/room_segmenter.cpp
  src/floor_detector.cpp
  src/hierarchical_planner.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# jps_planner_3d exports jps_planner_3d::jps_planner_3d_core
target_link_libraries(${PROJECT_NAME}_core
  jps_planner_3d::jps_planner_3d_core
)

ament_target_dependencies(${PROJECT_NAME}_core
  geometry_msgs
  nav_msgs
  jps_planner_3d
)

target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

# ── ROS2 node executable ──────────────────────────────────────────────────────
add_executable(topo_nav_node
  src/topo_nav_node.cpp
)

target_link_libraries(topo_nav_node
  ${PROJECT_NAME}_core
  jps_planner_3d::jps_planner_3d_core
)

ament_target_dependencies(topo_nav_node
  rclcpp
  geometry_msgs
  nav_msgs
  nav_interfaces
  jps_planner_3d
  rosidl_default_runtime
)

if(nav_interfaces_TARGETS__rosidl_typesupport_cpp)
  target_link_libraries(topo_nav_node
    ${nav_interfaces_TARGETS__rosidl_typesupport_cpp}
  )
else()
  message(FATAL_ERROR "nav_interfaces does not export C++ typesupport targets")
endif()

target_include_directories(topo_nav_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(topo_nav_node PUBLIC cxx_std_17)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}_core topo_nav_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ── Python tools ──────────────────────────────────────────────────────────────
ament_python_install_package(topo_nav_tools)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
