cmake_minimum_required(VERSION 3.5)
project(estimate_shape_manager)

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

set(FIT_METHODS 
  estimate_shape_fit_box
  estimate_shape_fit_sphere
  estimate_shape_fit_cylinder
  estimate_shape_fit_bowl
  estimate_shape_fit_plate
  estimate_shape_fit_cutting_board
  estimate_shape_fit_knife
  estimate_shape_fit_spatula
  # fit_banana
  # fit_broccoli
  # fit_octosphere
)

foreach(fit_method ${FIT_METHODS})
  find_package(${fit_method} REQUIRED)
endforeach(fit_method)

add_library(estimate_shape_manager SHARED 
  src/estimate_shape_manager.cpp
)
target_include_directories(estimate_shape_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(estimate_shape_manager PRIVATE "ESTIMATE_SHAPE_MANAGER_BUILDING_LIBRARY")

ament_target_dependencies(estimate_shape_manager
  ${FIT_METHODS}
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS estimate_shape_manager
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  estimate_shape_manager
)
ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)
ament_export_dependencies(
  ${FIT_METHODS}
)

ament_package()
