cmake_minimum_required(VERSION 2.8.3)
project(obstacle_avoidance)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
)

find_package(OpenCV REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include/${PROJECT_NAME}
)

# add_library(MotionDetection src/MotionDetection.cpp)
# target_link_libraries(MotionDetection ${OpenCV_LIBRARIES})

# add_library(ObstacleDetection src/ObstacleDetection.cpp)
# target_link_libraries(ObstacleDetection MotionDetection)

# add_executable(detector_node src/detector_node.cpp)
# target_link_libraries(detector_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   MotionDetection
#   ObstacleDetection
# )
