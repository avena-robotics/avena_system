cmake_minimum_required(VERSION 3.5)
project(generate_path)

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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(helpers_commons REQUIRED)

find_package(Bullet REQUIRED PATHS /home/avena/repos/bullet3_install/lib/cmake/bullet NO_DEFAULT_PATH)

message(${BULLET_ROOT_DIR})
message(${BULLET_USE_FILE})
message(${BULLET_INCLUDE_DIRS})
message(${BULLET_LIBRARY_DIRS})

set(BULLET_INCLUDE ${BULLET_ROOT_DIR} / ${BULLET_INCLUDE_DIRS})
set(BULLET_LIBS ${BULLET_ROOT_DIR} / ${BULLET_LIBRARY_DIRS})

message(${BULLET_INCLUDE})
message(${BULLET_LIBS})

include_directories(
  include
  ${BULLET_INCLUDE}
)

add_library(${PROJECT_NAME} SHARED 
  src/generate_path.cpp
)
target_include_directories(${PROJECT_NAME} 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "GENERATE_PATH_BUILDING_LIBRARY")
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  custom_interfaces
  rclcpp_action
  rclcpp_components
  helpers_commons
)
target_link_libraries(${PROJECT_NAME}
  ${BULLET_LIBS}/libBulletRobotics.so
)
rclcpp_components_register_nodes(${PROJECT_NAME} "generate_path::GeneratePath")
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

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
