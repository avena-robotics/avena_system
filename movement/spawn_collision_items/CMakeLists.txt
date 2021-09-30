cmake_minimum_required(VERSION 3.5)
project(spawn_collision_items)

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
find_package(custom_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(helpers_commons REQUIRED)
find_package(helpers_vision REQUIRED)
find_package(physics_client_handler REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/spawn_collision_items.cpp
)
target_include_directories(${PROJECT_NAME} 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "SPAWN_COLLISION_ITEMS_BUILDING_LIBRARY")
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  custom_interfaces
  helpers_commons
  helpers_vision
  physics_client_handler
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

############################################
# Test program to prepare simulation objects
add_executable(load_scene 
  tests/load_scene.cpp
)
ament_target_dependencies(load_scene
  rclcpp
  custom_interfaces
  ament_index_cpp
  helpers_vision
)
install(
  TARGETS load_scene
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)
############################################

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
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(
  rclcpp
  custom_interfaces
  helpers_commons
  helpers_vision
)

ament_package()
