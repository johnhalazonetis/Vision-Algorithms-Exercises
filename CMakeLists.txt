cmake_minimum_required(VERSION 3.0)

project( 3DVis )

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( findcheckerboard src/ex1/findcheckerboard.cpp )

target_link_libraries( findcheckerboard ${OpenCV_LIBS} )
