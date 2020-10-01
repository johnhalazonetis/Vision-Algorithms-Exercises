cmake_minimum_required(VERSION 3.0)

project( 3DVis )

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} include )

add_executable( ex1 src/ex1.cpp )
add_executable( ex2 src/ex2.cpp )

target_link_libraries( ex1 ${OpenCV_LIBS} )
target_link_libraries( ex2 ${OpenCV_LIBS} )
