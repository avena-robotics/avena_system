cmake_minimum_required(VERSION 3.5)
project(visualization_tools)

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
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(helpers_commons REQUIRED)
find_package(helpers_vision REQUIRED)
find_package(custom_interfaces REQUIRED)

##############################################################################
# Joint to tf library
add_library(joint_state_to_transform SHARED 
  src/joint_state_to_transform.cpp
)
target_include_directories(joint_state_to_transform
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(joint_state_to_transform
  kdl_parser
  orocos_kdl
  tf2_ros
  urdf
)
ament_export_targets(export_joint_state_to_transform HAS_LIBRARY_TARGET)
ament_export_dependencies(kdl_parser orocos_kdl tf2_ros urdf)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS joint_state_to_transform
  EXPORT export_joint_state_to_transform
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

##############################################################################
# Generated path as trajectory to cartesian end effector pose
set(LIBRARY_NAME movement_visualization)
add_library(${LIBRARY_NAME} SHARED 
  src/${LIBRARY_NAME}.cpp
)
target_include_directories(${LIBRARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${LIBRARY_NAME}
  joint_state_to_transform
)
ament_target_dependencies(${LIBRARY_NAME}
  rclcpp
  rclcpp_components
  nav_msgs
  trajectory_msgs
  helpers_commons
  helpers_vision
)
target_compile_definitions(${LIBRARY_NAME} PRIVATE "DEBUGGING_TOOLS_BUILDING_LIBRARY")
rclcpp_components_register_node(${LIBRARY_NAME}
  PLUGIN "visualization_tools::MovementVisualization"
  EXECUTABLE ${LIBRARY_NAME}_node)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${LIBRARY_NAME}
  EXPORT export_${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_dependencies(
  rclcpp 
  rclcpp_components
  trajectory_msgs
  nav_msgs
  helpers_commons
  helpers_vision
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${LIBRARY_NAME}
)
ament_export_targets(
  export_${LIBRARY_NAME}
)
##############################################################################

# Go to pose command
set(LIBRARY_NAME go_to_pose_command)
add_library(${LIBRARY_NAME} SHARED 
  src/${LIBRARY_NAME}.cpp
)
target_include_directories(${LIBRARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${LIBRARY_NAME}
  rclcpp
  rclcpp_components
  interactive_markers
  custom_interfaces
  helpers_vision
)
target_compile_definitions(${LIBRARY_NAME} PRIVATE "DEBUGGING_TOOLS_BUILDING_LIBRARY")
rclcpp_components_register_node(${LIBRARY_NAME}
  PLUGIN "visualization_tools::GoToPoseCommand"
  EXECUTABLE ${LIBRARY_NAME}_node)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${LIBRARY_NAME}
  EXPORT export_${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_dependencies(
  rclcpp 
  rclcpp_components
  trajectory_msgs
  nav_msgs
  helpers_commons
  helpers_vision
  custom_interfaces
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${LIBRARY_NAME}
)
ament_export_targets(
  export_${LIBRARY_NAME}
)
##############################################################################

# Go to pose command
set(LIBRARY_NAME scene_visualization)
add_library(${LIBRARY_NAME} SHARED 
  src/${LIBRARY_NAME}.cpp
)
target_include_directories(${LIBRARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${LIBRARY_NAME}
  rclcpp
  rclcpp_components
  helpers_commons
  helpers_vision
  custom_interfaces
  visualization_msgs
)
target_compile_definitions(${LIBRARY_NAME} PRIVATE "DEBUGGING_TOOLS_BUILDING_LIBRARY")
rclcpp_components_register_node(${LIBRARY_NAME}
  PLUGIN "visualization_tools::SceneVisualization"
  EXECUTABLE ${LIBRARY_NAME}_node)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${LIBRARY_NAME}
  EXPORT export_${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_dependencies(
  rclcpp 
  rclcpp_components
  helpers_commons
  helpers_vision
  custom_interfaces
  visualization_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${LIBRARY_NAME}
)
ament_export_targets(
  export_${LIBRARY_NAME}
)
##############################################################################

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
