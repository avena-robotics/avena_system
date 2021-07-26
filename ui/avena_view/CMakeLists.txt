cmake_minimum_required(VERSION 3.5)

project(avena_view)

find_package(ament_cmake REQUIRED)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_VERBOSE_MAKEFILE ON)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp_action REQUIRED)

set(avena_view_SRCS
  src/avena_view/avena_view.cpp
  src/avena_view/detectron_runner.cpp
  src/avena_view/utils.cpp
)

set(avena_view_HDRS
  include/avena_view/avena_view.h
  include/avena_view/detectron_runner.h
  include/avena_view/utils.h
)

set(avena_view_UIS
  src/avena_view/avena_view.ui
)

set(avena_view_INCLUDE_DIRECTORIES
  include
  ${rclcpp_INCLUDE_DIRS}
  ${qt_gui_cpp_INCLUDE_DIRS}
  ${rqt_gui_cpp_INCLUDE_DIRS}
  ${custom_interfaces_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

qt5_wrap_cpp(avena_view_MOCS ${avena_view_HDRS})

qt5_wrap_ui(avena_view_UIS_H ${avena_view_UIS})

include_directories(
  ${avena_view_INCLUDE_DIRECTORIES}
)

add_library(${PROJECT_NAME} SHARED
  ${avena_view_SRCS}
  ${avena_view_MOCS}
  ${avena_view_UIS_H}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${qt_gui_cpp_LIBRARIES}
  ${rqt_gui_cpp_LIBRARIES}
  yaml-cpp
  Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME}
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

install(PROGRAMS scripts/avena_view
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/avena_view
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY resource
#   DESTINATION share/${PROJECT_NAME}
# )

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(custom_interfaces yaml-cpp)

ament_package()
