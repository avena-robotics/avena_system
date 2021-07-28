cmake_minimum_required(VERSION 3.5)
project(rgbd_sync)

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
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)

find_package(custom_interfaces REQUIRED)
find_package(helpers_vision REQUIRED)
find_package(helpers_commons REQUIRED)

# include_directories(
#   include
# )

add_library(rgbd_sync SHARED src/rgbd_sync.cpp)
target_include_directories(rgbd_sync 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(rgbd_sync PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(rgbd_sync
  custom_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  helpers_vision
  helpers_commons
)
rclcpp_components_register_nodes(rgbd_sync "rgbd_sync::RgbdSyncronizer")

install(
  TARGETS rgbd_sync
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
