cmake_minimum_required(VERSION 3.5)
project(bullet_server)

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
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(helpers_commons REQUIRED)
find_package(bullet_client REQUIRED)
find_package(Bullet REQUIRED)

# Bullet shared memory server - only without GUI
add_library(bullet_server SHARED
  src/bullet_server.cpp
)
target_include_directories(bullet_server 
  PUBLIC
    ${BULLET_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(bullet_server PRIVATE "BULLET_SERVER_BUILDING_LIBRARY")
ament_target_dependencies(bullet_server 
  rclcpp
  rclcpp_components
  ament_index_cpp
  Bullet
  bullet_client
  helpers_commons
)
target_link_libraries(bullet_server
  ${BULLET_LIBRARIES}
  /usr/local/lib/libBullet3Common.so
  /usr/local/lib/libBulletRoboticsGUI.so
)
rclcpp_components_register_nodes(bullet_server "bullet_server::BulletServer")
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS bullet_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  bullet_server
)
ament_export_targets(
  export_${PROJECT_NAME}
)

###############################################################
# # Bullet server with shared memory with GUI
# add_executable(bullet_server_shared_memory_gui
#   src/main_opengl_single_example.cpp
# )
# target_include_directories(bullet_server_shared_memory_gui 
#   PUBLIC
#     ${BULLET_INCLUDE_DIRS}
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(bullet_server_shared_memory_gui 
#   bullet_client
#   Bullet
# )
# target_link_libraries(bullet_server_shared_memory_gui
#   ${BULLET_LIBRARIES}
#   /usr/local/lib/libBulletRoboticsGUI.so
#   /usr/local/lib/libBulletExampleBrowserLib.so
#   /usr/local/lib/libOpenGLWindow.so
# )

# libBullet2FileLoader.so -> libBullet2FileLoader.so.3.17
# libBullet2FileLoader.so.3.17
# libBullet3Collision.so -> libBullet3Collision.so.3.17
# libBullet3Collision.so.3.17
# libBullet3Common.so -> libBullet3Common.so.3.17
# libBullet3Common.so.3.17
# libBullet3Dynamics.so -> libBullet3Dynamics.so.3.17
# libBullet3Dynamics.so.3.17
# libBullet3Geometry.so -> libBullet3Geometry.so.3.17
# libBullet3Geometry.so.3.17
# libBullet3OpenCL_clew.so -> libBullet3OpenCL_clew.so.3.17
# libBullet3OpenCL_clew.so.3.17
# libBulletCollision.so -> libBulletCollision.so.3.17
# libBulletCollision.so.3.17
# libBulletDynamics.so -> libBulletDynamics.so.3.17
# libBulletDynamics.so.3.17
# libBulletExampleBrowserLib.so -> libBulletExampleBrowserLib.so.3.17
# libBulletExampleBrowserLib.so.3.17
# libBulletFileLoader.so -> libBulletFileLoader.so.3.17
# libBulletFileLoader.so.3.17
# libBulletInverseDynamics.so -> libBulletInverseDynamics.so.3.17
# libBulletInverseDynamics.so.3.17
# libBulletInverseDynamicsUtils.so -> libBulletInverseDynamicsUtils.so.3.17
# libBulletInverseDynamicsUtils.so.3.17
# libBulletRoboticsGUI.so -> libBulletRoboticsGUI.so.3.17
# libBulletRoboticsGUI.so.3.17
# libBulletRobotics.so -> libBulletRobotics.so.3.17
# libBulletRobotics.so.3.17
# libBulletSoftBody.so -> libBulletSoftBody.so.3.17
# libBulletSoftBody.so.3.17
# libBulletWorldImporter.so -> libBulletWorldImporter.so.3.17
# libBulletWorldImporter.so.3.17
# libBulletXmlWorldImporter.so -> libBulletXmlWorldImporter.so.3.17
# libBulletXmlWorldImporter.so.3.17
# libBussIK.so
# libclsocket.a
# libConvexDecomposition.so -> libConvexDecomposition.so.3.17
# libConvexDecomposition.so.3.17
# libGIMPACTUtils.so -> libGIMPACTUtils.so.3.17
# libGIMPACTUtils.so.3.17
# libgwen.so
# libHACD.so -> libHACD.so.3.17
# libHACD.so.3.17
# libLinearMath.so -> libLinearMath.so.3.17
# libLinearMath.so.3.17
# libOpenGLWindow.so
###############################################################

ament_package()
