cmake_minimum_required(VERSION 3.5)
project(filter_detections)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_BUILD_TYPE RELEASE)

find_package(ament_cmake REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(helpers_vision REQUIRED)
find_package(rclcpp_components REQUIRED)
# find_package(db_connector REQUIRED)


include_directories(
  include
)




add_library(filter_detections SHARED src/filter_detections.cpp)
target_compile_definitions(filter_detections PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(filter_detections
  custom_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  OpenCV
  helpers_vision
  )
rclcpp_components_register_nodes(filter_detections "FilterDetection")




install(TARGETS
filter_detections
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)









ament_package()
