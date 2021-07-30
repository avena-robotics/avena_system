cmake_minimum_required(VERSION 3.5)
project(example_logic_client)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
find_package(custom_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)

ament_export_include_directories(
        include
)
# Executables
set(ACTION_CLIENTS 
  rgbd_sync
)

foreach(action_client ${ACTION_CLIENTS})
  set(ACTION_NAME ${action_client})
  configure_file(src/cli_simple_action_client_template.cpp.in src/${action_client}_client.cpp)
  add_library(${action_client}_client SHARED
          src/${action_client}_client.cpp)
  rclcpp_components_register_node(${action_client}_client PLUGIN "cli::${action_client}Client" EXECUTABLE ${action_client})
  target_compile_definitions(${action_client}_client
          PRIVATE "${action_client}_CLIENT_BUILDING_DLL")
  ament_target_dependencies(${action_client}_client
          rclcpp
          rclcpp_action
          custom_interfaces
          rclcpp_components)

  install(TARGETS
          ${action_client}_client
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION bin)
endforeach()


ament_package()
