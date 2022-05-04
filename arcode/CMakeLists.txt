cmake_minimum_required(VERSION 2.8)

project( detect_markers )
find_package( OpenCV REQUIRED )
add_executable( detect_markers detect_markers.cpp )
target_link_libraries( detect_markers ${OpenCV_LIBS} )

project( create_marker )
find_package( OpenCV REQUIRED )
add_executable( create_marker create_marker.cpp )
target_link_libraries( create_marker ${OpenCV_LIBS} )



