#include <ros/ros.h>

#include <iostream>
#include <string>
#include <mavros_msgs/Trajectory.h>
#include <mavros_msgs/PositionTarget.h>

ros::Publisher vel_pub;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "avoidance");
  ros::NodeHandle nh;

  vel_pub=nh.advertise<mavros_msgs::Trajectory>("/uav0/mavros/trajectory/generated",1000);

  mavros_msgs::Trajectory path_msg; 
  ros::Rate rate(10);

  while(ros::ok())
  {
    path_msg.header.stamp = ros::Time::now();
    path_msg.type = 0;
    path_msg.point_1.position.x = 10;
    path_msg.point_1.position.y = 10;
    path_msg.point_1.position.z = 4;
    path_msg.point_1.velocity.x = NAN;
    path_msg.point_1.velocity.y = NAN;
    path_msg.point_1.velocity.z = NAN;
    path_msg.point_1.acceleration_or_force.x = NAN;
    path_msg.point_1.acceleration_or_force.y = NAN;
    path_msg.point_1.acceleration_or_force.z = NAN;
    path_msg.point_1.yaw = 0;
    path_msg.point_1.yaw_rate = 0;

    fillUnusedTrajectoryPoint(path_msg.point_2);
    fillUnusedTrajectoryPoint(path_msg.point_3);
    fillUnusedTrajectoryPoint(path_msg.point_4);
    fillUnusedTrajectoryPoint(path_msg.point_5);

    path_msg.time_horizon = {NAN, NAN, NAN, NAN, NAN};

    path_msg.point_valid = {true, false, false, false, false};

    vel_pub.publish(path_msg);
  }

  ros::spin();
}

void fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget& point) {
  point.position.x = NAN;
  point.position.y = NAN;
  point.position.z = NAN;
  point.velocity.x = NAN;
  point.velocity.y = NAN;
  point.velocity.z = NAN;
  point.acceleration_or_force.x = NAN;
  point.acceleration_or_force.y = NAN;
  point.acceleration_or_force.z = NAN;
  point.yaw = NAN;
  point.yaw_rate = NAN;
}