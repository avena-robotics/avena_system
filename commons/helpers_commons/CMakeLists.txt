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
find_package(Eigen3 REQUIRED)

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
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:${EIGEN3_INCLUDE_DIR}>
)
target_compile_definitions(helpers_commons PRIVATE "HELPERS_COMMONS_BUILDING_LIBRARY")
ament_target_dependencies(helpers_commons
  rclcpp
  custom_interfaces
  nlohmann_json
  urdf
  rosbag2_cpp
  Eigen3
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_geometry
    test/test_geometry.cpp
  )
  target_link_libraries(test_geometry
    helpers_commons
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  helpers_commons
)
ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)
ament_export_dependencies(
  rclcpp 
  custom_interfaces 
  nlohmann_json 
  urdf 
  rosbag2_cpp
  Eigen3
)

ament_package()
