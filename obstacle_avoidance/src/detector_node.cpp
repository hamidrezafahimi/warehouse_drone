#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <string>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include "ObstacleDetection.h"

using namespace std;
using namespace cv;

// ros::Publisher vis_pub;
// image_transport::Publisher pub;
ObstacleDetection *od;
ros::Publisher vel_pub;
int keyIn = -1;
float maxAngularSpeed = 1.57;
float maxCruiseSpeed = 3;
float st;
float last_t = 0;
float current_t = 0;

RNG rng(12345);

Mat binToRGB(Mat img)
{
  Mat result = cv::Mat::zeros(Size(img.cols, img.rows), CV_8UC3);

  for (int i=0; i<img.cols; i++)
  {
    for (int j=0; j<img.rows; j++)
    {
        result.at<Vec3b>(j,i)[2] = img.at<uchar>(j,i);
    }
  }

  return result;
}

float calcCommand(Mat bin)
{
  float cmd = 0;
  for (int i=0; i<bin.cols; i++)
  {
    for (int j=0; j<bin.rows; j++)
    {
      if (bin.at<uchar>(j,i) != 0)
      {
        cmd += (i>bin.cols/2) ? 1: -1;
      }
    }
  }

  return cmd / (bin.cols*bin.rows/2);
}

float calcSpeed(bool reset)
{
  if (reset)
  {
    st = ros::Time::now().toSec();
  }
  else
  {
    float dt = ros::Time::now().toSec() - st;
    
    return -exp(-0.2 * dt + 1.1) + maxCruiseSpeed;
  }

  return 0;
}

int sign(float val)
{
  return (val > 0) ? 1:-1;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
  last_t = (current_t != 0) ? current_t : 0;
  current_t = ros::Time::now().toSec();

  Mat bin = Mat::zeros(Size(image.cols, image.rows), CV_8U);
  Mat depth = Mat::zeros(Size(int(image.cols/20)+1, int(image.rows/20)+1), CV_8U);

  Mat dst;
  float dt = current_t - last_t;
  od->detectObs(&image, &dst, &bin, &depth, (last_t != 0) ? dt : 0);

  if (bin.cols > 0 && bin.rows > 0)
  {
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours( bin, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    Mat drawing = Mat::zeros( bin.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
      if (contourArea(contours[i]) < 5)
      {
        break;
      }

      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
    }

    // imshow("motion detection", bin);
    // imshow("contours", drawing);
  }

  if (depth.cols > 0 && depth.rows > 0)
  {
    // resize(dst, dst, Size(image.cols, image.rows));


    geometry_msgs::Twist velocity;
    velocity.angular.z = 20*calcCommand(depth) * maxAngularSpeed;
      if (velocity.angular.z != 0) 
        {
          // velocity.angular.z = sign(velocity.angular.z) * max(fabs(velocity.angular.z), 1.0);
          velocity.angular.z = sign(velocity.angular.z) * 3.5;
        }
    velocity.linear.y = calcSpeed( (velocity.angular.z != 0) );
    // velocity.linear.x = -0.02*velocity.angular.z;

    switch (keyIn)
    {
      case 81:
          velocity.angular.z = 0.3;
          keyIn = -1;
        break;

      case 83:
          velocity.angular.z = -0.3;
          keyIn = -1;
        break;

      default:
        break;
    }

    vel_pub.publish(velocity);

    resize(depth, depth, Size(image.cols, image.rows));

    // Mat result = cv::Mat::zeros(Size(image.cols, image.rows), CV_8UC3);
    addWeighted(dst, 1, binToRGB(depth), 0.9, 0.0, dst);

    // imshow("flow", dst);
    // imshow("depth", depth);
    imshow("collision_avoidance", dst);
  }

  keyIn = waitKey(30);

  last_t = ros::Time::now().toSec();
}


int main(int argc, char **argv)
{

  od = new ObstacleDetection(100, 0, 1, 20, 40, 2, 2, 30, 15, 0.75);

  ros::init(argc, argv, "detection");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/iris_0/camera_front/image_raw", 1, imageCallback);
  vel_pub=nh.advertise<geometry_msgs::Twist>("/avoidance/cmd_vel",1000);

  calcSpeed(true);

  ros::spin();
}