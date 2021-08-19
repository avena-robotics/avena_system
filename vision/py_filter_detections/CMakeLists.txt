cmake_minimum_required(VERSION 3.16)
set (CMAKE_CXX_STANDARD 17)
project(py_filter_detections)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package(pybind11 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Python ${PYTHON_VERSION} REQUIRED COMPONENTS Development)
pybind11_add_module(py_filter_detections src/main.cpp)


target_compile_definitions(py_filter_detections PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
target_link_libraries(py_filter_detections PRIVATE ${OpenCV_LIBS} )
execute_process(
  COMMAND "${PYTHON_EXECUTABLE}" -m site --user-site
  OUTPUT_VARIABLE PYTHON_SITE
  OUTPUT_STRIP_TRAILING_WHITESPACE)

message(${PYTHON_SITE})
install(TARGETS
    py_filter_detections
    COMPONENT python
    LIBRARY DESTINATION ${PYTHON_SITE}
)











