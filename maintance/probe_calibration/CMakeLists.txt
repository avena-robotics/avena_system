cmake_minimum_required(VERSION 3.5)
project(probe_calibration)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(helpers_vision REQUIRED)
find_package(helpers_commons REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED 
  src/${PROJECT_NAME}.cpp
)
target_include_directories(${PROJECT_NAME} 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "PROBE_CALIBRATION_BUILDING_LIBRARY")
set(DEPENDENCIES 
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  custom_interfaces
  helpers_vision
  helpers_commons
  tf2_ros
)
ament_target_dependencies(${PROJECT_NAME}
  ${DEPENDENCIES}
)
rclcpp_components_register_node(${PROJECT_NAME} 
  PLUGIN "probe_calibration::ProbeCalibration" 
  EXECUTABLE ${PROJECT_NAME}_node
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  # EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
# ament_export_targets(
#   export_${PROJECT_NAME}
# )
ament_export_dependencies(${DEPENDENCIES})


ament_package()
