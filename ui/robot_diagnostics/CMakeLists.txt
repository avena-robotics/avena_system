cmake_minimum_required(VERSION 3.5)

project(robot_diagnostics)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(rclcpp REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp_action REQUIRED)

set(
    robot_diagnostics_SRCS
    src/robot_diagnostics/robot_diagnostics.cpp
)

set(
    robot_diagnostics_HDRS
    include/robot_diagnostics/robot_diagnostics.h
)

set(
    robot_diagnostics_UIS
    src/robot_diagnostics/robot_diagnostics.ui
)

set(
    robot_diagnostics_INCLUDE_DIRS
    include
    ${rclcpp_INCLUDE_DIRS}
    ${qt_gui_cpp_INCLUDE_DIRS}
    ${rqt_gui_cpp_INCLUDE_DIRS}
    ${custom_interfaces_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
)

qt5_wrap_cpp(
    robot_diagnostics_MOCS
    ${robot_diagnostics_HDRS}
)

qt5_wrap_ui(
    robot_diagnostics_UIS_H
    ${robot_diagnostics_UIS}
)

add_library(
    ${PROJECT_NAME} SHARED
    ${robot_diagnostics_SRCS}
    ${robot_diagnostics_MOCS}
    ${robot_diagnostics_UIS_H}
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    ${robot_diagnostics_INCLUDE_DIRS}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${rclcpp_LIBRARIES}
    ${qt_gui_cpp_LIBRARIES}
    ${rqt_gui_cpp_LIBRARIES}
    yaml-cpp
    Qt5::Widgets
)

ament_target_dependencies(
    ${PROJECT_NAME}
    rclcpp
    custom_interfaces
    yaml-cpp 
    rclcpp_action 
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

install(PROGRAMS scripts/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(custom_interfaces yaml-cpp)

ament_package()

