cmake_minimum_required(VERSION 3.5)
project(candriver)

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


add_library(candriver SHARED 
  src/lib.c
  src/candriver.cpp
)
target_include_directories(candriver 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(candriver PRIVATE "CANDRIVER_BUILDING_LIBRARY")

# INSTALL
install(TARGETS
  candriver
  DESTINATION lib/${PROJECT_NAME}
  )
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS candriver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  candriver
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_package()














