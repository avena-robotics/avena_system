cmake_minimum_required(VERSION 3.5)
project(path_buffer)

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

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(helpers_commons REQUIRED)

include_directories(
  include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(${PROJECT_NAME} SHARED 
  src/path_buffer.cpp
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(${PROJECT_NAME}
  trajectory_msgs
  custom_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  helpers_commons
)
rclcpp_components_register_nodes(${PROJECT_NAME} "path_buffer::PathBuffer")


install(TARGETS 
  path_buffer 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
