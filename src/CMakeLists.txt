cmake_minimum_required(VERSION 3.0)

project( 3DVis )

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( OpenAndMoveWindows src/basics/OpenAndMoveWindows.cpp )
add_executable( PixelPerfect src/basics/PixelPerfect.cpp )
add_executable( SplitAndMerge src/basics/SplitAndMerge.cpp )
add_executable( dft src/dft.cpp )
add_executable( capture src/basics/capture.cpp )
add_executable( aruco src/aruco.cpp )
add_executable( calibrate src/calibrate.cpp )

target_link_libraries( OpenAndMoveWindows ${OpenCV_LIBS} )
target_link_libraries( PixelPerfect ${OpenCV_LIBS} )
target_link_libraries( SplitAndMerge ${OpenCV_LIBS} )
target_link_libraries( dft ${OpenCV_LIBS} )
target_link_libraries( capture ${OpenCV_LIBS} )
target_link_libraries( aruco ${OpenCV_LIBS} )
target_link_libraries( calibrate ${OpenCV_LIBS} )
