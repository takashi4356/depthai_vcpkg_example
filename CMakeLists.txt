cmake_minimum_required(VERSION 3.10)

project(cmake_example)

find_package(depthai CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)

add_executable(example src/main.cpp)

target_link_libraries(example PRIVATE depthai::core)
target_link_libraries(example PRIVATE opencv_core opencv_highgui opencv_imgproc)
