# Warehouse Drone

A software to guide, control and monitor a warehouse inventory flying robot, capable of record the IDs of the shelf packages as well as their position index in among shelf cells. The system is programmed and tested in Gazebo simulation. It is also impleneted in real flight tests performed by DJI Ryze Tello.


## Installation

The whole system is based on ROS. Basic pre-requirements are:

```
ROS (kinetic)
Gazebo-7
Python 2.7.17
C++11
```

To perform the simulation, first, create a ROS workspace and clone the repo in the *src/* folder. Running the codes requires the following python packages must be installed:

```
opencv 4.2.0
rospy
roslib
yaml
sensor_msgs
geometry_msgs
cv_bridge
std_msgs
std_msgs
numpy
```

## Usage

Source the workspace. (In its root folder:)

```
source devel/setup.bash 
```

To run the simulation:

```
roslaunch sim_gazebo wh_sim.launch
```

To perform the inventory task:

```
roslaunch sim_gazebo move.launch
```


