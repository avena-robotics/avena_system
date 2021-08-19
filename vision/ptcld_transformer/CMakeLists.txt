cmake_minimum_required(VERSION 3.5)
project(ptcld_transformer)

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
find_package(custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(PCL REQUIRED)
find_package(helpers_vision REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(ptcld_transformer SHARED src/ptcld_transformer.cpp)
target_include_directories(ptcld_transformer 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(ptcld_transformer PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(ptcld_transformer
  rclcpp
  rclcpp_components
  custom_interfaces
  helpers_vision
  nlohmann_json
  PCL
  )


install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(rclcpp custom_interfaces helpers_vision rclcpp_action rclcpp_components std_msgs)


ament_package()
