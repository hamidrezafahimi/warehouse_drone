cmake_minimum_required(VERSION 3.0.2)
project(dynamic_control)
add_compile_options(-std=c++11)


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


add_message_files(
  FILES
  sixDof.msg
  #Message2.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
	#visual_guidance
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simulate
  CATKIN_DEPENDS message_runtime
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


add_executable(controller src/controller.cpp)


target_link_libraries(controller ${catkin_LIBRARIES})


add_dependencies(controller ${catkin_EXPORTED_TARGETS})


