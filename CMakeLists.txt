cmake_minimum_required(VERSION 3.0)

project( VisionAlgorithmsExercises )

find_package( OpenCV REQUIRED )
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories( ${OpenCV_INCLUDE_DIRS} include )

add_executable( ex1 src/ex1.cpp )
add_executable( ex2 src/ex2.cpp )
add_executable( ex3 src/ex3.cpp )
add_executable( ex4 src/ex4.cpp )
add_executable( ex5 src/ex5.cpp )

target_link_libraries( ex1 ${OpenCV_LIBS} Eigen3::Eigen )
target_link_libraries( ex2 ${OpenCV_LIBS} Eigen3::Eigen )
target_link_libraries( ex3 ${OpenCV_LIBS} Eigen3::Eigen )
target_link_libraries( ex4 ${OpenCV_LIBS} Eigen3::Eigen )
target_link_libraries( ex5 ${OpenCV_LIBS} Eigen3::Eigen )
