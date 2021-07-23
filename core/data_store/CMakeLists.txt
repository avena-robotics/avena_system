cmake_minimum_required(VERSION 3.5)
project(data_store)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(helpers_commons REQUIRED)



add_library(${PROJECT_NAME} SHARED src/data_store.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${PROJECT_NAME}
  "custom_interfaces"
  "rclcpp"
  "helpers_commons"
  "rclcpp_components"
)
rclcpp_components_register_nodes(${PROJECT_NAME} "data_store::DataStore")

install(TARGETS
${PROJECT_NAME}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


add_executable(rgb_insert_pub test/src/rgb/insert_pub.cpp)
ament_target_dependencies(rgb_insert_pub rclcpp custom_interfaces)
install(TARGETS
rgb_insert_pub
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(cam1_insert_pub test/src/cam1/insert_pub.cpp)
ament_target_dependencies(cam1_insert_pub rclcpp custom_interfaces)
install(TARGETS
cam1_insert_pub
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(cam2_insert_pub test/src/cam2/insert_pub.cpp)
ament_target_dependencies(cam2_insert_pub rclcpp custom_interfaces)
install(TARGETS
cam2_insert_pub
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
