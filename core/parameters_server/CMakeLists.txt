cmake_minimum_required(VERSION 3.5)
project(parameters_server)

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
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(helpers_commons REQUIRED)

add_library(parameters_server SHARED src/parameters_server.cpp)
# target_include_directories(parameters_server 
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
target_compile_definitions(parameters_server PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(parameters_server
  rclcpp
  rclcpp_components
  helpers_commons
)
rclcpp_components_register_nodes(parameters_server "parameters_server::ParametersServer")
set(node_plugins "${node_plugins}parameters_server::ParametersServer;$<TARGET_FILE:parameters_server>\n")

install(TARGETS
  parameters_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
