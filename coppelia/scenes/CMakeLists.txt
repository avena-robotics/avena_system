cmake_minimum_required(VERSION 3.5)
project(scenes)

find_package(ament_cmake REQUIRED)

set(BASE_PATH /opt/avena/scenes)

install(
  DIRECTORY scenes
  DESTINATION ${BASE_PATH}
)

install(
  DIRECTORY models
  DESTINATION ${BASE_PATH}
)

ament_package()
