#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "visual_guidance/sixDof.h"
#include <sstream>
using namespace std;


float lvx, lvy, lvz;
bool flag = false, enterance = false, go = false;
int init = 0;


void dataCallback(const visual_guidance::sixDof msg)
{
  cout<<"\n\n\n";
  flag = true;
  init++;

  lvx = -(0.5*msg.x);
  lvy = -(0.5*msg.y);
  lvz = -(0.5*msg.z);
  // avz = 0.005 * msg.b;

  enterance = msg.enterance;
  cout<<"dyncon subscribed data\tvy:"<<lvy<<"\tvz:\t"<<lvz<<endl;
}


int main(int argc, char **argv)
{
  float gain = 1, vey, vez;
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("moveCommand", 1, dataCallback);
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("quadrotor_1/cmd_vel", 1);
  ros::Rate loop_rate(10); // Loop at 10Hz
  int count = 0, subCount = 0, t = 0 ;
  geometry_msgs::Twist gmsg;
  float th;

  // std::cout << "hiiiii" << '\n';
  // avz = gain *

  while (ros::ok())
  {

    gmsg.linear.x = 0;
    gmsg.linear.y = lvy;
    gmsg.linear.z = lvz;
    gmsg.angular.x = 0;
    gmsg.angular.y = 0;
    gmsg.angular.z = 0;

    // gmsg.linear.x = 0;
    // gmsg.linear.y = 0.1;
    // gmsg.linear.z = 0;
    // gmsg.angular.x = 0;
    // gmsg.angular.y = 0;
    // gmsg.angular.z = 0;

    pub.publish(gmsg);
    cout<<"dyncon published data\tgmsg.linear.y:"<<gmsg.linear.y<<"\tgmsg.linear.z:\t"<<gmsg.linear.z<<endl;
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
