cmake_minimum_required(VERSION 3.10)

project(cmake_example)

find_package(depthai CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)
# Find Python
find_package(Python REQUIRED COMPONENTS Development)
# OpenMPを探す
find_package(OpenMP REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Create a project with name 'myapp'
set(TARGET_NAME myapp)
set(CMAKE_BUILD_TYPE Release) # Debug or Release 
project(${TARGET_NAME})

# Add source files
add_executable("${TARGET_NAME}"
    src/main3.cpp
)
target_include_directories(myapp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)


# OpenMPのフラグを追加
if(OpenMP_CXX_FOUND)
    target_link_libraries("${TARGET_NAME}" PRIVATE OpenMP::OpenMP_CXX)
endif()
#ライブラリディレクトリ指定 
set(INCLUDE_DIR
#  ${OPENCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  /.../plog/include
  /.../json/include
  /usr/include
  /home/pi
  /usr/include/eigen3
  #/home/pi/WiringPi/wiringPi
  ${PROJECT_SOURCE_DIR}/include
  ${Python_INCLUDE_DIRS}
#windows対応
#C:\\Users\\Takashi\\OneDrive\\depthaiC++\\depthai-core-example\\plog\\include
#C:\\Users\\Takashi\\OneDrive\\depthaiC++\\depthai-core-example\\json\\include
)
# Add a include files
include_directories("${INCLUDE_DIR}")

#find_library(WIRINGPI_LIBRARIES NAMES wiringPi)

# Link with libraries
target_link_libraries(${TARGET_NAME}
    PUBLIC
        ${OpenCV_LIBS} # optional, used for example
        #${WIRINGPI_LIBRARIES}
        /usr/local/lib/libpigpiod_if2.so
        #/home/pi/gpiozerocpp/bin/Release/libgpiozerocpp.so
        /usr/lib/aarch64-linux-gnu/libgpiod.so
        /usr/lib/aarch64-linux-gnu/libgpiodcxx.so
        ${Python_LIBRARIES}
    PRIVATE depthai::core
    opencv_core opencv_highgui opencv_imgproc
        )

# Set some errors
if(NOT MSVC)
    target_compile_options(${TARGET_NAME} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Werror=return-type>)
endif()
#windows対応
#add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
#add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

# Set compiler features (c++11), and disables extensions (g++)
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 11)
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_EXTENSIONS OFF)