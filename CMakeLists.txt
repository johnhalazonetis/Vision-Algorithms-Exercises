cmake_minimum_required(VERSION 3.0)

project( 3DVis )

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( ex1 src/ex1/ex1.cpp )
add_executable( ex1mod src/ex1/ex1mod.cpp )

target_link_libraries( ex1 ${OpenCV_LIBS} )
target_link_libraries( ex1mod ${OpenCV_LIBS} )
