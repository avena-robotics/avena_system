cmake_minimum_required(VERSION 3.5)
project(candriver)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(helpers_commons REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(socketcan REQUIRED)
find_package(Boost REQUIRED
  COMPONENTS
    chrono
    system
    thread
)
find_package(Threads REQUIRED)


add_executable(can_node 
  src/can_node.cpp
  src/can_operations.cpp
)
target_include_directories(can_node PRIVATE include)
# target_link_libraries(can_node can_lib)
ament_target_dependencies(can_node
  pluginlib
  rclcpp
  helpers_commons
  std_msgs 
  custom_interfaces
  sensor_msgs
  # socketcan
)


add_executable(talker src/talker.cpp)
target_include_directories(talker PRIVATE include)
# target_link_libraries(talker can_lib)
ament_target_dependencies(talker rclcpp std_msgs custom_interfaces)

# INSTALL
install(TARGETS
  can_node
  # can_lib
  talker
  DESTINATION lib/${PROJECT_NAME}
  )
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()














