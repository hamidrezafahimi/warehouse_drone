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
#include <sstream>

#include "ObstacleDetection.h"

using namespace std;
using namespace cv;

// ros::Publisher vis_pub;
// image_transport::Publisher pub;
ObstacleDetection *od;
ros::Publisher vel_pub;
image_transport::Publisher img_pub;
image_transport::Publisher img_pub1;
image_transport::Publisher img_pub2;
image_transport::Publisher img_pub3;
image_transport::Publisher img_pub4;
int keyIn = -1;
float maxAngularSpeed = 1.57;
float maxCruiseSpeed = 3;
float st;
double last_t = 0;
double current_t = 0;
// bool active = false;
bool active = true;
float maxFPS = 5;
float fpsChecker = 0.0;

RNG rng(12345);

int itit = 0;
int itt = 0;
int ttt = 0;
int tit = 0;

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

    return -exp(-0.4 * dt + 0.8) + maxCruiseSpeed;
  }

  return 0;
}

void normalizeMap(Mat &in)
{
    int rows = in.rows, cols = in.cols;
    for (int i=0; i<rows; i++) {
        for (int j=0; j<cols; j++) {
            if (in.at<uchar>(i,j) < 220)
                in.at<uchar>(i,j) = 220;
        }
    }
    cv::normalize(in, in, 0, 255, NORM_MINMAX, CV_8UC1);
}

int sign(float val)
{
  return (val > 0) ? 1:-1;
}

void act_cb(const std_msgs::BoolConstPtr& msg)
{
  std::cout << "act_cb received" << '\n';
  active = msg->data;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  if (!active)
  {
    return;
  }

  Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

  last_t = (current_t != 0) ? current_t : 0;
  current_t = ros::Time::now().toSec();

  // if(image.rows != 480 || image.cols != 640)
  // {
  //   resize(image, image, Size(640,480));
  // }

  Mat bin = Mat::zeros(Size(image.cols, image.rows), CV_8U);
  Mat depth = Mat::zeros(Size(int(image.cols/20)+1, int(image.rows/20)+1), CV_8U);
  Mat depth_unfiltered = Mat::zeros(Size(int(image.cols/20)+1, int(image.rows/20)+1), CV_8U);
  Mat grayDepthMap = Mat::zeros(Size(int(image.cols/20)+1, int(image.rows/20)+1), CV_8U);

  Mat dst, flowImg;
  float dt = current_t - last_t;

  // if (fpsChecker < 1.0/maxFPS)
  // {
  //   fpsChecker += dt;
  //   return;
  // }
  // else
  // {
  //   fpsChecker = 0;
  // }

  od->detectObs(&image, &dst, &flowImg, &bin, &depth_unfiltered, &depth, &grayDepthMap, (last_t != 0) ? dt : 0, active);


  if (!active)
  {
    depth = Mat::zeros(Size(int(image.cols/20)+1, int(image.rows/20)+1), CV_8U);
  }

  if (bin.cols > 0 && bin.rows > 0)
  {
  //   vector<vector<Point> > contours;
  //   vector<Vec4i> hierarchy;

  //   findContours( bin, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  //   Mat drawing = Mat::zeros( bin.size(), CV_8UC3 );
  //   for( int i = 0; i< contours.size(); i++ )
  //   {
  //     if (contourArea(contours[i]) < 5)
  //     {
  //       break;
  //     }

  //     Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
  //     drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
  //   }

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
    // putText(dst, "fps: " + to_string(float(1/dt)), cv::Point2f(10, 40), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 2);

    std::ostringstream name0;
    name0 << "/home/hamidreza/thesis/workSpace/img_" << ttt++ <<"_" <<to_string(msg->header.stamp.toSec()) << ".png";
    cv::imwrite(name0.str(), dst);

    // imshow("obstacles", dst);
    // imshow("flow", flowImg);
    // imshow("collision_avoidance", dst);
    sensor_msgs::ImagePtr msg0 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
    img_pub.publish(msg0);

    // std::ostringstream name3;
    // name3 << "/home/hamidreza/thesis/workSpace/flow_" << tit++ << ".png";
    // cv::imwrite(name3.str(), flowImg);

    sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", flowImg).toImageMsg();
    img_pub1.publish(msg1);

    cvtColor(depth_unfiltered, depth_unfiltered, cv::COLOR_GRAY2BGR);
    sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_unfiltered).toImageMsg();
    img_pub2.publish(msg2);

    cvtColor(depth, depth, cv::COLOR_GRAY2BGR);

    std::ostringstream name1;
    name1 << "/home/hamidreza/thesis/workSpace/depth_" << itt++ << "_" <<to_string(msg->header.stamp.toSec()) << ".png";
    cv::imwrite(name1.str(), depth);

    sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth).toImageMsg();
    msg3->header.stamp = msg->header.stamp;
    img_pub3.publish(msg3);

    Mat depthMap;
    normalizeMap(grayDepthMap);
    applyColorMap(grayDepthMap, depthMap, COLORMAP_JET);
    // applyColorMap(grayDepthMap, depthMap, COLORMAP_BONE);

    // Mat dm;
    // cv::resize(depthMap, dm, Size(960, 720));
    // std::ostringstream name2;
    // name2 << "/home/hamidreza/thesis/workSpace/depthMap_" << itit++ << ".png";
    // cv::imwrite(name2.str(), dm);

    sensor_msgs::ImagePtr msg4 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depthMap).toImageMsg();
    img_pub4.publish(msg4);
  }

  keyIn = waitKey(30);

  last_t = ros::Time::now().toSec();
}

int main(int argc, char **argv)
{
  od = new ObstacleDetection(60, 100, 0, 1, 20, 60, 2, 2, 50, 3, 0.75, 2);

  ros::init(argc, argv, "detection");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  // image_transport::Subscriber sub = it.subscribe("/iris_0/camera_front/image_raw", 1, imageCallback);
  image_transport::Subscriber sub = it.subscribe("/tello/camera/image_raw", 1, imageCallback);
  // image_transport::Subscriber sub = it.subscribe("/quadrotor_1/front/image_raw", 1, imageCallback);
  ros::Subscriber activation_sub = nh.subscribe<std_msgs::Bool>("/avoidance/activate", 1, act_cb);
  vel_pub=nh.advertise<geometry_msgs::Twist>("/avoidance/cmd_vel",1000);
  img_pub = it.advertise("/avoidance/image", 1);
  img_pub1 = it.advertise("/avoidance/flow", 1);
  img_pub2 = it.advertise("/avoidance/depth_unfiltered", 1);
  img_pub3 = it.advertise("/avoidance/depth", 1);
  img_pub4 = it.advertise("/avoidance/depthMap", 1);

  calcSpeed(true);

  ros::spin();
}
