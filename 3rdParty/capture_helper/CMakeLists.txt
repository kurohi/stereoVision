cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project( captureHelper )


add_definitions(-std=c++11)

find_package( OpenCV REQUIRED )

include_directories(include)
file( GLOB LIB_SOURCES lib/*.cpp )
file( GLOB LIB_HEADERS include/*.hpp )

add_library( camera_control lib/camera_control.cpp )
add_library( stereo_capture lib/stereo_capture.cpp )


add_executable( capture_helper src/capture_helper.cpp )

target_link_libraries( capture_helper ${OpenCV_LIBS} stereo_capture camera_control )
