cmake_minimum_required(VERSION 3.5)
project(helpers_vision)

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
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(helpers_commons REQUIRED)

add_library(helpers_vision SHARED 
  src/converters.cpp
  src/vision.cpp
  src/visualization.cpp
)
target_include_directories(helpers_vision 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(helpers_vision PRIVATE "HELPERS_VISION_BUILDING_LIBRARY")
target_link_libraries(helpers_vision 
  ${PCL_LIBRARIES}  # without this link, packages which uses helpers cannot load some PCL module
)
ament_target_dependencies(helpers_vision
  rclcpp
  cv_bridge
  PCL
  Eigen3
  OpenCV
  nlohmann_json
  geometry_msgs
  sensor_msgs
  tf2_ros
  helpers_commons
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS helpers_vision
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_mask_serialization
    test/test_mask_serialization.cpp
  )
  target_link_libraries(test_mask_serialization
    helpers_vision
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  helpers_vision
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(helpers_commons rclcpp cv_bridge PCL Eigen3 OpenCV nlohmann_json rosbag2_cpp geometry_msgs tf2_ros sensor_msgs)

ament_package()
