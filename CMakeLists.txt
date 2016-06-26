cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project( stereoVision )

find_package( PCL 1.2 REQUIRED )
find_package(VTK 6.1 REQUIRED NO_MODULE)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package( OpenCV REQUIRED )
include_directories(include)
file( GLOB LIB_SOURCES lib/*.cpp )
file( GLOB LIB_HEADERS include/*.hpp )
add_library( stereoVisionLibs ${LIB_SOURCES} ${LIB_HEADERS} )


add_executable( setTruckPlanes src/SetTruckPlanes.cpp )
add_executable( setEmptyTruck src/SetEmptyTruck.cpp )
add_executable( getPercentage src/GetPercentage.cpp )
add_executable( stereocalibrate src/stereocalibrate.cpp )
add_executable( StereoParamSet src/StereoParamSet.cpp )
#test programs
add_executable( testCamera test/testCamera.cpp )
add_executable( testPclView test/testPclView.cpp )
add_executable( testPclGetPlanes test/testPclGetPlanes.cpp )
add_executable( testLoadPlane test/testLoadPlane.cpp )


target_link_libraries( setTruckPlanes ${OpenCV_LIBS} ${PCL_LIBRARIES} stereoVisionLibs )
target_link_libraries( setEmptyTruck ${OpenCV_LIBS} ${PCL_LIBRARIES} stereoVisionLibs )
target_link_libraries( getPercentage ${OpenCV_LIBS} ${PCL_LIBRARIES} stereoVisionLibs )
target_link_libraries( stereocalibrate ${OpenCV_LIBS} stereoVisionLibs )
target_link_libraries( StereoParamSet ${OpenCV_LIBS} stereoVisionLibs )
#test program linkers
target_link_libraries( testCamera ${OpenCV_LIBS} stereoVisionLibs )
target_link_libraries( testPclView ${OpenCV_LIBS} ${PCL_LIBRARIES} stereoVisionLibs )
target_link_libraries( testPclGetPlanes ${OpenCV_LIBS} ${PCL_LIBRARIES} stereoVisionLibs )
target_link_libraries( testLoadPlane ${OpenCV_LIBS} ${PCL_LIBRARIES} stereoVisionLibs )
