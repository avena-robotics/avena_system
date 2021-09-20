cmake_minimum_required(VERSION 3.5)
project(compose_items)

# Default to C++17
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
find_package(helpers_commons REQUIRED)
find_package(rclcpp_components REQUIRED)


include_directories(
  include
)

# # - change_detect_server
# add_executable(compose_items_server 
#   src/compose_items_server.cpp
# )

# target_include_directories(compose_items_server
#   PUBLIC
#   include
#   ${OpenCV_INCLUDE_DIRS}
#   ${PCL_INCLUDE_DIRS}
# )


# add_library(pcl_methods
#     src/pcl_methods/create_ptcld.cpp
# )
# target_include_directories(pcl_methods PUBLIC include)
# target_link_libraries(pcl_methods 
#     ${OpenCV_LIBS}
#     ${PCL_LIBRARIES}
#     ${nlohmann_json_LIBRARIES}
# )

# ament_target_dependencies(pcl_methods
#   rclcpp
#   helpers_vision
# )

# target_link_libraries(compose_items_server
#   ${PCL_LIBRARIES}
#   pcl_methods
#   ${nlohmann_json_LIBRARIES}
# )

# ament_target_dependencies(compose_items_server
#   rclcpp
#   rclcpp_action
#   custom_interfaces
#   sensor_msgs
#   OpenCV
#   helpers_vision
# )

# install(TARGETS
# compose_items_server
#   DESTINATION lib/${PROJECT_NAME}
# )


add_library(compose_items SHARED src/compose_items.cpp src/pcl_methods/create_ptcld.cpp)
target_compile_definitions(compose_items PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(compose_items
  custom_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  OpenCV
  helpers_vision
  helpers_commons
  )
rclcpp_components_register_nodes(compose_items "compose_items::ComposeItems")# EXECUTABLE compose_items)
# set(node_plugins "${node_plugins}::ComposeItems;$<TARGET_FILE:compose_items>\n")



install(TARGETS
compose_items
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
