cmake_minimum_required(VERSION 3.5)

project(custom_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/CamerasData.msg"
  "msg/RgbImages.msg"
  "msg/DepthImages.msg"
  "msg/Ptclds.msg"
  "msg/Watchdog.msg"
  "msg/Heartbeat.msg"
  "msg/Statistics.msg"
  "msg/ModuleCommand.msg"
  "msg/GeneratedPath.msg"
)

# set(srv_files
# 
# )

set(action_files
  "action/GeneratePathPose.action"
  "action/SimpleAction.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES sensor_msgs
  DEPENDENCIES trajectory_msgs
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
