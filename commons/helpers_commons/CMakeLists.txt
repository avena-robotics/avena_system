cmake_minimum_required(VERSION 3.5)
project(helpers_commons)

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
find_package(rosbag2_cpp REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(urdf REQUIRED)

add_library(helpers_commons SHARED 
  src/generic_subscription.cpp
  src/logging.cpp
  src/robot_utils.cpp
  src/subscriptions_manager.cpp
  src/watchdog.cpp
)
target_include_directories(helpers_commons 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(helpers_commons PRIVATE "HELPERS_COMMONS_BUILDING_LIBRARY")
ament_target_dependencies(helpers_commons
  rclcpp
  custom_interfaces
  nlohmann_json
  urdf
  rosbag2_cpp
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS helpers_commons
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
  helpers_commons
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(rclcpp custom_interfaces nlohmann_json urdf rosbag2_cpp)

ament_package()
