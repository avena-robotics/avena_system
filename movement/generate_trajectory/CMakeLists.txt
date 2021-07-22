cmake_minimum_required(VERSION 3.5)
project(generate_trajectory)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_BUILD_TYPE RELEASE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(helpers_commons REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)

# include_directories(include)

add_library(generate_trajectory SHARED 
  src/generate_trajectory.cpp src/path.cpp 
  src/trajectory.cpp
)
target_include_directories(generate_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(generate_trajectory PRIVATE "GENERATE_TRAJECTORY_BUILDING_LIBRARY")
ament_target_dependencies(generate_trajectory
  rclcpp
  trajectory_msgs
  rclcpp_action
  custom_interfaces
  helpers_commons
  rclcpp_components
  Eigen3
)
rclcpp_components_register_node(generate_trajectory
  PLUGIN "generate_trajectory::GenerateTrajectory"
  EXECUTABLE generate_trajectory_node)

install(TARGETS
  generate_trajectory
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
