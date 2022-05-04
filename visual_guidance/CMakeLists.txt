cmake_minimum_required(VERSION 3.0.2)
project(visual_guidance CXX)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV REQUIRED
  geometry_msgs
  message_generation
  gazebo_ros
  nav_msgs
)

find_package(gazebo REQUIRED)

   #-Dlibsdl-dev_DIR=/usr/share/doc
#find_package(libsdl1.2-dev REQUIRED)


#link_directories(/usr/share/doc/libsdl1.2-dev)



set(PLUGIN_INC_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/plugins/)

add_message_files(
  FILES
  sixDof.msg
  shelfGrid.msg
  nodeGrid.msg
  cell.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simulate
  CATKIN_DEPENDS message_runtime
#  CATKIN_DEPENDS message_runtime geometry_msgs roscpp std_msgs 
#  DEPENDS system_lib
)


include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${PLUGIN_INC_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


set(UTILITY utility)
add_library(${UTILITY} SHARED src/${UTILITY}.cpp)
target_link_libraries(${UTILITY} ${OpenCV_LIBS})


set(ARCODE_DETECTOR ArcodeDetector)
add_library(${ARCODE_DETECTOR} SHARED src/${ARCODE_DETECTOR}.cpp)
target_link_libraries(${ARCODE_DETECTOR} ${OpenCV_LIBS})
target_link_libraries(${ARCODE_DETECTOR} ${UTILITY})


set(PATH_GENERATOR pathGeneration)
add_library(${PATH_GENERATOR} SHARED src/${PATH_GENERATOR}.cpp)
target_link_libraries(${PATH_GENERATOR} ${OpenCV_LIBS})
#target_link_libraries(${PATH_GENERATOR} ${UTILITY})


add_executable(cvi src/cvi.cpp)
target_link_libraries(cvi ${OpenCV_LIBS} ${catkin_LIBRARIES} ${ARCODE_DETECTOR} ${PATH_GENERATOR})
#target_link_libraries(cvi ${UTILITY})

add_dependencies(cvi ${catkin_EXPORTED_TARGETS})

# --------------------------------

#add_library(MotionDetection src/MotionDetection.cpp)
#target_link_libraries(MotionDetection ${OpenCV_LIBRARIES})

#add_library(ObstacleDetection src/ObstacleDetection.cpp)
#target_link_libraries(ObstacleDetection MotionDetection)


#add_executable(detector_node src/detector_node.cpp)
#target_link_libraries(detector_node
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#  MotionDetection
#  ObstacleDetection
#)




