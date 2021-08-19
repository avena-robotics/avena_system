cmake_minimum_required(VERSION 3.5)
project(octomap_filter)

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
set(CMAKE_BUILD_TYPE RELEASE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(nlohmann_json REQUIRED)

find_package(helpers_vision REQUIRED)
find_package(helpers_commons REQUIRED)

add_library(octomap_filter SHARED src/octomap_filter.cpp)

target_include_directories(octomap_filter
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(octomap_filter PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(octomap_filter  
  rclcpp
  rclcpp_action
  rclcpp_components
  custom_interfaces
  # sensor_msgs
  OpenCV
  PCL
  helpers_vision
  # nlohmann_json

)

rclcpp_components_register_nodes(octomap_filter "octomap_filter::OctomapFilter")
 set(node_plugins "${node_plugins}octomap_filter::OctomapFilter;$<TARGET_FILE:octomap_filter>\n")

add_library(octomap_filter_debug SHARED src/octomap_filter_debug.cpp)
target_include_directories(octomap_filter_debug PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(octomap_filter_debug PRIVATE "OCTOMAP_FILTER_BUILDING_LIBRARY")
ament_target_dependencies(octomap_filter_debug
  rclcpp
  rclcpp_components
  custom_interfaces
  helpers_vision)
rclcpp_components_register_nodes(octomap_filter_debug "ros2mysql::OctomapFilter")

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  octomap_filter 
  octomap_filter_debug
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



ament_package()
