cmake_minimum_required(VERSION 3.5)
project(generate_trajectory_from_waypoints)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: Choosing 'Release' for maximum performance")
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Boost REQUIRED date_time filesystem thread system)
find_package(warehouse_ros REQUIRED)
find_package(moveit_ros_warehouse REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(control_msgs REQUIRED)

# Main plugin to generate trajectories from waypoints
add_library(generate_trajectory_from_waypoints SHARED src/generate_trajectory_from_waypoints.cpp)
target_include_directories(generate_trajectory_from_waypoints 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_node(generate_trajectory_from_waypoints 
  PLUGIN "generate_trajectory_from_waypoints::GenerateTrajectoryFromWaypoints"
  EXECUTABLE generate_trajectory_from_waypoints_node
)
ament_target_dependencies(generate_trajectory_from_waypoints
  rclcpp
  Boost
  warehouse_ros
  moveit_ros_warehouse
  moveit_ros_planning_interface
  custom_interfaces
)

# Node to trigger trajectories generation
add_executable(generate_trajectories_trigger src/generate_trajectories_trigger.cpp)
ament_target_dependencies(generate_trajectories_trigger
  rclcpp
  rclcpp_action
  custom_interfaces
)

# Node to play generated trajectories using dummy_arm_controller
add_executable(play_generated_trajectories src/play_generated_trajectories.cpp)
ament_target_dependencies(play_generated_trajectories
  rclcpp
  control_msgs
  rclcpp_action
)

install(
  TARGETS generate_trajectory_from_waypoints
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS generate_trajectories_trigger play_generated_trajectories
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
