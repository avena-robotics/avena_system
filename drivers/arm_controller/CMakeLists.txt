cmake_minimum_required(VERSION 3.5)
project(arm_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -lrt -g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(helpers_commons REQUIRED)
find_package(candriver REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenMP REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(base_controller
    src/base_controller.cpp
    src/PID.cpp)

target_include_directories(base_controller  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(base_controller  Eigen3::Eigen OpenMP::OpenMP_CXX rt)


ament_target_dependencies(base_controller 
  pluginlib
  rclcpp
  std_msgs 
  sensor_msgs
  custom_interfaces
  helpers_commons
  candriver
  pinocchio
  Boost
  OpenMP
)


add_executable(hw_interface
    src/hw_interface.cpp
    )

target_include_directories(hw_interface  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(hw_interface rt)


ament_target_dependencies(hw_interface 
  pluginlib
  rclcpp
  std_msgs 
  sensor_msgs
  custom_interfaces
  helpers_commons
  candriver
  pinocchio
  Boost
)


add_executable(trajectory_feeder
    src/trajectory_feeder.cpp
    )

target_include_directories(trajectory_feeder  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(trajectory_feeder rt)


ament_target_dependencies(trajectory_feeder 
  pluginlib
  rclcpp
  std_msgs 
  sensor_msgs
  custom_interfaces
  Boost
)

add_executable(diagnostics
    src/diagnostics.cpp
    )

target_include_directories(diagnostics  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(diagnostics  base_controller_lib Eigen3::Eigen)

ament_target_dependencies(diagnostics 
  pluginlib
  rclcpp
  std_msgs 
  sensor_msgs
  custom_interfaces
  helpers_commons
  candriver
  pinocchio
  Boost
)


add_executable(single_joint_debug
    src/single_joint_debug.cpp
    src/PID.cpp
    )

target_include_directories(single_joint_debug  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  
 target_link_libraries(single_joint_debug  base_controller_lib Eigen3::Eigen)
  
 ament_target_dependencies(single_joint_debug 
    pluginlib
    rclcpp
    std_msgs 
    sensor_msgs
    custom_interfaces
    helpers_commons
    candriver
  )

add_executable(friction_calibration
    src/friction_calibration.cpp
    src/PID.cpp)

target_include_directories(friction_calibration  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(friction_calibration  base_controller_lib Eigen3::Eigen)


ament_target_dependencies(friction_calibration 
  pluginlib
  rclcpp
  std_msgs 
  helpers_commons
  candriver
  custom_interfaces
  sensor_msgs
  pinocchio
  Boost
)


add_library(base_controller_lib SHARED 
  src/base_controller.cpp
  src/PID.cpp
)

target_include_directories(base_controller_lib  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(base_controller_lib  Eigen3::Eigen OpenMP::OpenMP_CXX rt)

target_compile_definitions(base_controller_lib PRIVATE "BASE_CONTROLLER_BUILDING_LIBRARY")

ament_target_dependencies(base_controller_lib 
  pluginlib
  rclcpp
  std_msgs 
  sensor_msgs
  custom_interfaces
  helpers_commons
  candriver
  pinocchio
  Boost
  OpenMP
)

install(TARGETS
  base_controller
  diagnostics
  friction_calibration
  hw_interface
  single_joint_debug
  trajectory_feeder
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS base_controller_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  base_controller_lib
)

ament_package()
