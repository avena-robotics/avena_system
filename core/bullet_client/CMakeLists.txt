cmake_minimum_required(VERSION 3.5)
project(bullet_client)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
set(CMAKE_BUILD_TYPE RELEASE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(Bullet REQUIRED)

message(${BULLET_INCLUDE_DIRS})

add_library(bullet_client SHARED
  src/b3RobotSimulatorClientAPI.cpp
)
target_include_directories(bullet_client 
  PUBLIC
    ${BULLET_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bullet_client 
  Bullet
)
target_link_libraries(bullet_client
  ${BULLET_LIBRARIES}
  /usr/local/lib/libBulletRobotics.so
)
target_compile_definitions(bullet_client PRIVATE "BULLET_CLIENT_BUILDING_LIBRARY")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Bullet)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS bullet_client
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(bullet_client)

ament_package()
