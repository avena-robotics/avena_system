cmake_minimum_required(VERSION 3.5)
project(rgb_diff)


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

find_package(custom_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(helpers_vision REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(
  include
)



add_library(rgb_diff SHARED src/rgb_diff_server.cpp)
target_compile_definitions(rgb_diff PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(rgb_diff
  custom_interfaces
  helpers_vision
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  )
rclcpp_components_register_nodes(rgb_diff "rgb_diff_action_server::RgbDiffActionServer" EXECUTABLE rgb_diff)
set(node_plugins "${node_plugins}rgb_diff_action_server::RgbDiffActionServer;$<TARGET_FILE:rgb_diff>\n")



install(TARGETS
  rgb_diff
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()