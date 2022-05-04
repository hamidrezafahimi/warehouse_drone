cmake_minimum_required(VERSION 3.0.2)
project(drone_description)


add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  sensor_msgs
  cv_bridge
  image_transport
  #OpenCV REQUIRED
  geometry_msgs
  message_generation
  gazebo_ros
  nav_msgs
)

find_package(gazebo REQUIRED)
find_pacKage(OpenCV REQUIRED)

   #-Dlibsdl-dev_DIR=/usr/share/doc
#find_package(libsdl1.2-dev REQUIRED)


#link_directories(/usr/share/doc/libsdl1.2-dev)



set(PLUGIN_INC_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/plugins/)

 #add_message_files(
  # FILES
#   imtodyn.msg
#   Message2.msg
 #)


generate_messages(
  DEPENDENCIES
  std_msgs
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

# link_directories(
#   ${GAZEBO_LIBRARY_DIRS}
#   ${SDFormat_LIBRARY_DIRS}
# )

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set(PLUGIN_NAME quadrotor_simple_controller)
add_library(${PLUGIN_NAME} SHARED src/plugins/${PLUGIN_NAME}.cpp)

#add_executable(impro src/impro.cpp)
#add_executable(dyncon src/dyncon.cpp)
# add_executable(pubmsg src/pubmsg.cpp)


#target_link_libraries( impro ${OpenCV_LIBS} )
#target_link_libraries(impro ${catkin_LIBRARIES})
#target_link_libraries(dyncon ${catkin_LIBRARIES})
# target_link_libraries(pubmsg ${catkin_LIBRARIES})
target_link_libraries(${PLUGIN_NAME} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${SDFormat_LIBRARIES} ${Boost_LIBRARIES})

#add_dependencies(impro ${catkin_EXPORTED_TARGETS})
#add_dependencies(dyncon ${catkin_EXPORTED_TARGETS})
# add_dependencies(pubmsg ${catkin_EXPORTED_TARGETS})
