#include <ros/ros.h>

#include <iostream>
#include <string>
#include <vector>
#include<numeric>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
// #include <mavros_msgs/PositionTarget.h>
// #include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>

// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/ParamSet.h>

ros::Publisher vel_pub;
ros::Publisher act_pub;
ros::ServiceClient set_mode_client;
std::vector<float> command_buff;
std::vector<float> guss_buff;
float avoidance_t = 0;
float last_t = 0;
float current_t = 0;

int buff_size = 10;
int k = 0.5;

float sigma = 0;
float mu = 0.0;
float vel_thresh = 2;
bool last_activation = false;

geometry_msgs::TwistStamped vel;
mavros_msgs::State   current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

float norm(int idx)
{
  return exp(-pow(idx - mu,2) / (2*pow(sigma,2)) );
}

void formDist()
{
  for (int i=0; i<buff_size; i++)
  {
    guss_buff.push_back(norm(i));
  }
}

void dotdotproduct()
{
  for(int i=0; i<command_buff.size(); i++)
  {
    command_buff.at(i) *= guss_buff.at(i);
  }
}

float updateCMDBuff(float newVal)
{
  // last_t = (current_t != 0) ? current_t : 0;
  // current_t = ros::Time::now().toSec();
  // float dt = (last_t != 0) ? current_t - last_t: 0;
  float result = k*accumulate(command_buff.begin(),command_buff.end(),0);

  if (command_buff.size() < buff_size)
  {
    command_buff.push_back(newVal);
  }
  else
  {
    command_buff.erase(command_buff.begin());
    command_buff.push_back(newVal);
  }

  // dotdotproduct();

  // return accumulate(command_buff.begin(),command_buff.end(),0);
    return result;

}

void sendAvoidanceMsg(geometry_msgs::Twist vel)
{

    mavros_msgs::PositionTarget avoid_msg;

    avoid_msg.header.stamp = ros::Time::now();
    avoid_msg.header.frame_id = "world";
    avoid_msg.coordinate_frame = 8;
    avoid_msg.type_mask = 1991;
    avoid_msg.velocity.x = vel.linear.x;
    avoid_msg.velocity.y = vel.linear.y;
    avoid_msg.velocity.z = vel.linear.z;
    avoid_msg.yaw_rate = vel.angular.z + updateCMDBuff(vel.angular.z);

    // ROS_INFO("vx = %.3f, vy = %.3f, vz = %.3f, yaw_rate = %.3f", avoid_msg.velocity.x, avoid_msg.velocity.y, avoid_msg.velocity.z, avoid_msg.yaw_rate);

    vel_pub.publish(avoid_msg);
}

void cmd_cb(const geometry_msgs::Twist::ConstPtr& msg){

  if (msg->angular.z != 0)
  {
    avoidance_t = ros::Time::now().toSec();
  }

  if(msg->angular.z != 0 && current_state.mode != "OFFBOARD" && current_state.mode == "AUTO.MISSION")
  {

    ROS_ERROR("Obstacle detected!");
    ROS_WARN("Switching to OFFBOARD mode.");

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    set_mode_client.call(offb_set_mode);
  }


  sendAvoidanceMsg(*msg);
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){

  std_msgs::Bool active_msg;

  if (msg->twist.linear.x < vel_thresh && last_activation)
  {

    active_msg.data = false;
    act_pub.publish(active_msg);
    last_activation = active_msg.data;
  }
  else if (msg->twist.linear.x >= vel_thresh && !last_activation)
  {
    active_msg.data = true;
    act_pub.publish(active_msg);
    last_activation = active_msg.data;
  }

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "avoidance");
  ros::NodeHandle nh;

  ros::Subscriber cmd_sub = nh.subscribe<geometry_msgs::Twist>
            ("/avoidance/cmd_vel", 1, cmd_cb);
  ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/uav0/mavros/local_position/velocity_body", 1, vel_cb);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("uav0/mavros/state", 10, state_cb);

  vel_pub=nh.advertise<mavros_msgs::PositionTarget>("/uav0/mavros/setpoint_raw/local",1000);
  act_pub=nh.advertise<std_msgs::Bool>("/avoidance/activate",1);
  ros::ServiceClient param_client = nh.serviceClient<mavros_msgs::ParamSet>
        ("uav0/mavros/param/set");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("uav0/mavros/set_mode");


  // mavros_msgs::ParamSet param_msg;

  // param_msg.request.param_id = "COM_OBS_AVOID";
  // param_msg.request.value.integer = 1;

  // param_client.call(param_msg);

  sigma = buff_size / 4;
  mu = buff_size * 0.7;
  formDist();

  ros::Rate rate(30);

  while(ros::ok())
  {


    if (ros::Time::now().toSec() - avoidance_t > 5 && current_state.mode == "OFFBOARD")
    {
      ROS_WARN("avoidance timed out! continuing mission.");

      float t0 = ros::Time::now().toSec();
      while(current_state.mode != "AUTO.MISSION" && ros::Time::now().toSec() - avoidance_t > 5 && current_state.mode == "OFFBOARD")
      {
        mavros_msgs::SetMode mission_set_mode;
        mission_set_mode.request.custom_mode = "AUTO.MISSION";

        set_mode_client.call(mission_set_mode);

        ros::Duration(0.1).sleep();
        ros::spinOnce();
      }

    }

    rate.sleep();
    ros::spinOnce();
  }

}
