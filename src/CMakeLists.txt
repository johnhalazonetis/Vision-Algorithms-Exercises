cmake_minimum_required(VERSION 3.0)

project( 3DVis )

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( capture src/basics/capture.cpp )
add_executable( aruco src/aruco.cpp )
add_executable( calibrate src/calibrate.cpp )

target_link_libraries( capture ${OpenCV_LIBS} )
target_link_libraries( aruco ${OpenCV_LIBS} )
target_link_libraries( calibrate ${OpenCV_LIBS} )
