cmake_minimum_required(VERSION 3.5)

project(data_store_view)

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
find_package(rclcpp_action REQUIRED)
find_package(helpers_commons REQUIRED)
find_package(helpers_vision REQUIRED)

set(
    data_store_view_SRCS
    src/data_store_view/data_store_view.cpp
    src/data_store_view/cameras_view.cpp
    src/data_store_view/items_view.cpp
    src/data_store_view/utils.cpp
)

set(
    data_store_view_HDRS
    include/data_store_view/data_store_view.h
    include/data_store_view/cameras_view.h
    include/data_store_view/items_view.h
    include/data_store_view/utils.h
)

set(
    data_store_view_UIS
    src/data_store_view/data_store_view.ui
)

set(
    data_store_view_INCLUDE_DIRS
    include
    ${rclcpp_INCLUDE_DIRS}
    ${qt_gui_cpp_INCLUDE_DIRS}
    ${rqt_gui_cpp_INCLUDE_DIRS}
    ${custom_interfaces_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
)

qt5_wrap_cpp(
    data_store_view_MOCS
    ${data_store_view_HDRS}
)

qt5_wrap_ui(
    data_store_view_UIS_H
    ${data_store_view_UIS}
)

add_library(
    ${PROJECT_NAME} SHARED
    ${data_store_view_SRCS}
    ${data_store_view_MOCS}
    ${data_store_view_UIS_H}
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    ${data_store_view_INCLUDE_DIRS}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${rclcpp_LIBRARIES}
    ${qt_gui_cpp_LIBRARIES}
    ${rqt_gui_cpp_LIBRARIES}
    Qt5::Widgets
)

ament_target_dependencies(
    ${PROJECT_NAME}
    rclcpp
    custom_interfaces
    rclcpp_action 
    helpers_commons
    helpers_vision
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

install(
    DIRECTORY assets/
    DESTINATION share/${PROJECT_NAME}/assets/
)

install(
    DIRECTORY rviz_configs/
    DESTINATION share/${PROJECT_NAME}/rviz_configs/
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
ament_export_dependencies(custom_interfaces helpers_commons helpers_vision)

ament_package()
