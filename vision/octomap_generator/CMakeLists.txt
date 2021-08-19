cmake_minimum_required(VERSION 3.5)
project(octomap_generator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_BUILD_TYPE RELEASE)

find_package(ament_cmake REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(helpers_vision REQUIRED)
find_package(helpers_commons REQUIRED)
find_package(ptcld_transformer REQUIRED)
find_package(robot_self_filter REQUIRED)
find_package(rclcpp_components REQUIRED)


include_directories(
  include
)



add_library(octomap_generator SHARED src/octomap_generator.cpp)
target_compile_definitions(octomap_generator PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(octomap_generator
  custom_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  OpenCV
  helpers_vision
  helpers_commons
  ptcld_transformer
  robot_self_filter
  )

rclcpp_components_register_nodes(octomap_generator "octomap_generator::OctomapGenerator")# EXECUTABLE compose_items)

install(TARGETS
octomap_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
