cmake_minimum_required(VERSION 3.5)
project(dummy_arm_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
find_package(rclcpp_components REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(dummy_arm_controller SHARED src/dummy_arm_controller.cpp)
target_include_directories(dummy_arm_controller 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_node(dummy_arm_controller 
  PLUGIN "dummy_arm_controller::DummyArmController"
  EXECUTABLE dummy_arm_controller_node
)
ament_target_dependencies(dummy_arm_controller
  rclcpp
  rclcpp_action
  rclcpp_components
  control_msgs
  sensor_msgs
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(dummy_arm_controller PRIVATE "DUMMY_ARM_CONTROLLER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS dummy_arm_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  dummy_arm_controller
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
