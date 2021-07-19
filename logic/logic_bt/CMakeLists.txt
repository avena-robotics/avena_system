cmake_minimum_required(VERSION 3.5)
project(logic_bt)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(BehaviorTreeV3 REQUIRED)
# find_package(custom_interfaces REQUIRED)
# find_package(rclcpp_action REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(trajectory_msgs REQUIRED)
# find_package(helpers_commons REQUIRED)



# add_library(${PROJECT_NAME} SHARED src/logic_bt.cpp)

# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# ament_target_dependencies(
#   ${PROJECT_NAME}
#   "BehaviorTreeV3"
#   "custom_interfaces"
#   "rclcpp_action"
#   "rclcpp"
#   "sensor_msgs"
#   "trajectory_msgs"
#   "helpers_commons"
#   "rclcpp_components"
# )
# target_link_libraries(${PROJECT_NAME} BT::behaviortree_cpp_v3)
# rclcpp_components_register_nodes(${PROJECT_NAME} "logic_bt::Logic")

# install(TARGETS
# ${PROJECT_NAME}
# ARCHIVE DESTINATION lib
# LIBRARY DESTINATION lib
# RUNTIME DESTINATION bin)

# install(DIRECTORY
#   launch
#   xml
#   DESTINATION share/${PROJECT_NAME}/
# )

include_directories(include)
add_executable(security_rgb src/security_rgb.cpp)
ament_target_dependencies(security_rgb
  "rclcpp"
  "std_msgs")
install(TARGETS security_rgb
DESTINATION lib/${PROJECT_NAME})
ament_package()
