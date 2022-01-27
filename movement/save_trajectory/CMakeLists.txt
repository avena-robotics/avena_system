cmake_minimum_required(VERSION 3.5)
project(save_trajectory)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(save_trajectory SHARED src/save_trajectory.cpp)
target_include_directories(save_trajectory 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_node(save_trajectory 
  PLUGIN "save_trajectory::SaveTrajectory"
  EXECUTABLE save_trajectory_node
)

ament_target_dependencies(save_trajectory
  rclcpp
  rclcpp_action
  rclcpp_components
  control_msgs
  sensor_msgs
)

# install(
#   TARGETS save_trajectory
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  TARGETS save_trajectory
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
