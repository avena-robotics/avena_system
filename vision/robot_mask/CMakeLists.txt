cmake_minimum_required(VERSION 3.5)
project(robot_mask)


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

add_library(robot_mask SHARED src/robot_mask.cpp)
target_include_directories(robot_mask 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(robot_mask PRIVATE "ROBOT_MASK_BUILDING_LIBRARY")
ament_target_dependencies(robot_mask
rclcpp
custom_interfaces
helpers_vision
rclcpp_action
rclcpp_components
std_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS robot_mask
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_export_include_directories(
  include
)
ament_export_libraries(
  robot_mask
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(rclcpp custom_interfaces helpers_vision rclcpp_action rclcpp_components std_msgs)


# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()