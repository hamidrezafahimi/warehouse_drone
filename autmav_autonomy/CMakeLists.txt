cmake_minimum_required(VERSION 2.8.3)
project(autmav_autonomy)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
#  mavros
#  mavros_msgs
)
find_package(OpenCV REQUIRED)


catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy std_msgs mavros mavros_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include/${PROJECT_NAME}
  ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include/obstacle_detection
  ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include/arucoBoardDetector
)


add_library(MotionDetection src/MotionDetection.cpp)
target_link_libraries(MotionDetection ${OpenCV_LIBRARIES})

add_library(ObstacleDetection src/ObstacleDetection.cpp)
target_link_libraries(ObstacleDetection MotionDetection)

add_library(arucoBoardDetector src/arucoBoardDetector.cpp)
target_link_libraries(arucoBoardDetector
  ${OpenCV_LIBRARIES}
)


#add_executable(${PROJECT_NAME}_node src/offb_node.cpp)
#target_link_libraries(${PROJECT_NAME}_node
##  ${mavros_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

#add_executable(avoidance_node src/avoidance.cpp)
#target_link_libraries(avoidance_node
##  ${mavros_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

add_executable(detector_node src/detector_node.cpp)
target_link_libraries(detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  MotionDetection
  ObstacleDetection
)

add_executable(platform_detection src/platform_detection.cpp)
target_link_libraries(platform_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  arucoBoardDetector
)





