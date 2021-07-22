cmake_minimum_required(VERSION 3.5)
project(get_cameras_data)

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
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(helpers_vision REQUIRED)
find_package(helpers_commons REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(get_cameras_data SHARED 
  src/get_cameras_data.cpp
)
target_include_directories(get_cameras_data 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(get_cameras_data PRIVATE "GET_CAMERAS_DATA_BUILDING_LIBRARY")
ament_target_dependencies(get_cameras_data
  rclcpp
  rclcpp_components
  message_filters
  custom_interfaces
  helpers_vision
  helpers_commons
  tf2_msgs
  tf2_ros
  geometry_msgs
)
rclcpp_components_register_nodes(get_cameras_data "get_cameras_data::GetCamerasData")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS get_cameras_data
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
  get_cameras_data
)
ament_export_targets(
  export_${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
