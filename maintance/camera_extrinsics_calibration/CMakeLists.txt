cmake_minimum_required(VERSION 3.5)
project(camera_extrinsics_calibration)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(helpers_vision REQUIRED)
find_package(helpers_commons REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()



include_directories(
  include
)
add_executable(calibrate src/calibrate.cpp )
ament_target_dependencies(calibrate
  custom_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  helpers_vision
  rclcpp
  sensor_msgs
  tf2_ros
  PCL
  yaml-cpp
  helpers_commons
  )
  
target_link_libraries(calibrate 
  yaml-cpp
)

include_directories(
  include
)


add_library(hand_eye SHARED src/hand_eye.cpp )
# add_executable(hand_eye src/hand_eye.cpp )
ament_target_dependencies(hand_eye
  custom_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  helpers_vision
  rclcpp
  sensor_msgs
  tf2_ros
  PCL
  yaml-cpp
  helpers_commons
  )
target_link_libraries(hand_eye 
  yaml-cpp
)
rclcpp_components_register_nodes(hand_eye "PclCalibrator")



install(TARGETS
calibrate
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# - install --------------------------------------------------------
install(TARGETS
  hand_eye
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_package()
