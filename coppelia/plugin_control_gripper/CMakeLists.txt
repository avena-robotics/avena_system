cmake_minimum_required(VERSION 3.5)
project(plugin_control_gripper)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall --std=c++11 -O3 -fPIC -pedantic" )
set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wall -O3 -fPIC" )

# Coppelia path
set(COPPELIASIM_ROOT_DIR /opt/avena/coppelia_brain)
set(LIBPLUGIN_DIR ${COPPELIASIM_ROOT_DIR}/programming/libPlugin)

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
    ${LIBPLUGIN_DIR}/cmake)

find_package(Boost REQUIRED COMPONENTS system serialization)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(helpers_commons REQUIRED)
find_package(helpers_vision REQUIRED)
find_package(CoppeliaSim 4.1.0.0 REQUIRED)

set(INCLUDE_DIRS
    include
    ${COPPELIASIM_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/generated
    ${CMAKE_CURRENT_SOURCE_DIR}/external
    ${Boost_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIR}
)

coppeliasim_generate_stubs(${CMAKE_CURRENT_BINARY_DIR}/generated 
    XML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/callbacks.xml 
    LUA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/simExtROS2PluginControlGripper.lua
)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h ESCAPE_QUOTES)

set(GRIPPERS
    src/grippers/franka_gripper.cpp
)

set(TARGET_NAME simExtROS2PluginControlGripper)
add_library(${TARGET_NAME} SHARED 
    src/plugin.cpp 
    ${GRIPPERS}
    ${COPPELIASIM_EXPORTED_SOURCES}
)
target_include_directories(${TARGET_NAME}
  PUBLIC
  ${INCLUDE_DIRS}
)
target_link_libraries(${TARGET_NAME} 
    ${Boost_LIBRARIES} 
    ${COPPELIASIM_LIBRARIES} 
)
target_compile_definitions(${TARGET_NAME} PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(${TARGET_NAME}
    rclcpp
    custom_interfaces
    rclcpp_action
    helpers_commons
    helpers_vision
    std_srvs
)

set_target_properties(${TARGET_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS ${TARGET_NAME} DESTINATION ${COPPELIASIM_ROOT_DIR})

ament_package()
