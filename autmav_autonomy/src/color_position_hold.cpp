#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <string>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>

using namespace std;
using namespace cv;

enum Color{
  RED,
  BLUE,
  GREEN
} colorToBeDetected;

// ros::Publisher vis_pub;
// image_transport::Publisher pub;
ros::Publisher vel_pub;
image_transport::Publisher img_pub;
double last_t = 0;
double current_t = 0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
  last_t = (current_t != 0) ? current_t : 0;
  current_t = ros::Time::now().toSec();

  float dt = (last_t != 0) ? current_t - last_t : 0; 

  Mat hsv, dst; 
  cvtColor(image, hsv, SV_RGB2HSV);

  switch (colorToBeDetected)
  {
    case RED:
      {
        Mat mask1, mask2;
        inRange(hsv, Scalar(0, 100, 0), Scalar(20, 255, 255), mask1);
        inRange(hsv, Scalar(170, 100, 0), Scalar(180, 255, 255), mask2);
        addWeighted(mask1, 1, mask2, 1, 0.0, dst);
      };
      break;
    case BLUE:
      {
        inRange(hsv, Scalar(20,50,50), Scalar(35,255,255), dst);
      };
      break;
    case GREEN:
      {
        inRange(hsv, Scalar(20,50,50), Scalar(35,255,255), dst);
      };
      break;
    default:
        break;
  }
  

  mavros_msgs::PositionTarget control_cmd; 

  control_cmd.header.stamp = ros::Time::now();
  control_cmd.header.frame_id = "world";
  control_cmd.coordinate_frame = 8;
  control_cmd.type_mask = 1991;
  control_cmd.velocity.x = vel.linear.x;
  control_cmd.velocity.y = vel.linear.y;
  control_cmd.velocity.z = vel.linear.z;
  control_cmd.yaw_rate = 0;

  vel_pub.publish(control_cmd);

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
  img_pub.publish(msg);

  keyIn = waitKey(30);

}


int main(int argc, char **argv)
{

  string missionPlatform = argv[1];
  bool isSim = (missionPlatform == "sim");

  string topic = (isSim) ? "/iris_0/camera_front/image_raw" : "/usb_cam/image_raw";
  string prefix = (isSim) ? "/uav0" : "";

  ros::init(argc, argv, "color_position_hold");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(topic, 1, imageCallback);
  vel_pub=nh.advertise<mavros_msgs::PositionTarget>(prefix + "/mavros/setpoint_raw/local",1000);
  img_pub = it.advertise("/position_hold/image", 1);

  ros::spin();
}