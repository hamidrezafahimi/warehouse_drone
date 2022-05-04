#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/videoio.hpp"
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <algorithm>
#include "visual_guidance/sixDof.h"
#include <iostream>
#include <math.h>
using namespace cv;
using namespace std;
#define PI 3.14159265




static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}


Vec3d get6dofInFrameBodyCoordinates(Vec3d rvec, Vec3d tvec, Vec3d& r, Vec3d& t)
{
  Mat rot_mat = Mat::zeros(3,3,CV_64F);
  float roll, pitch, yaw, x, y, z;
  // vector<float> v;
  // Vec3d v;

  Rodrigues(rvec, rot_mat);

  roll = atan2 (-rot_mat.at<double>(2,1),rot_mat.at<double>(2,2)) * 180 / PI;
  pitch = asin(rot_mat.at<double>(2,0)) * 180 / PI;
  yaw = atan2(-rot_mat.at<double>(1,0), rot_mat.at<double>(0,0))*180/PI;
  //
  r = {roll, pitch, yaw};

  x = ((-rot_mat.at<double>(0,0))*tvec[0]) + ((-rot_mat.at<double>(1,0))*tvec[1]) + ((-rot_mat.at<double>(2,0))*tvec[2]);
  y = ((-rot_mat.at<double>(0,1))*tvec[0]) + ((-rot_mat.at<double>(1,1))*tvec[1]) + ((-rot_mat.at<double>(2,1))*tvec[2]);
  z = ((-rot_mat.at<double>(0,2))*tvec[0]) + ((-rot_mat.at<double>(1,2))*tvec[1]) + ((-rot_mat.at<double>(2,2))*tvec[2]);

  t = {x, y, z};
  cout<<tvec[2]<<'\t'<<t[2]<<endl;
  // cout<<rot_mat.at<double>(1,2)<<endl;
  // cout<<rot_mat<<endl;

  // return v;

}



  Mat image, imageCopy;

  vector< int > ids;
  vector< vector< Point2f > > corners, rejected;
  vector< Vec3d > rvecs, tvecs;
  int theId = 0;
  bool flag = false;

  float markerLength = 0.1;

  Ptr<aruco::Dictionary> dictionary =
      aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(2));

  Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

  Mat camMatrix, distCoeffs;


float calculateDistance(Vec3d tvec)
{
  return sqrt((tvec[0]*tvec[0])+(tvec[1]*tvec[1])+(tvec[2]*tvec[2]));
}



void imageCallback(const sensor_msgs::ImageConstPtr& msg)    //will get called when a new image has arrived on the
{      //"camera/image" topic. Although the image may have been sent in some arbitrary transport-specific message type,


  float minDist = 1e7, dist;
  flag = false;
  tvecs.clear();
  rvecs.clear();

  image = cv_bridge::toCvShare(msg, "bgr8")->image;


  aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

  // cout<<camMatrix<<endl;
  // cout<<distCoeffs<<endl;

  aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs,
                                   tvecs);
  // vector<float> v;
  // Vec3d r, t;
  vector<Vec3d> rs, ts;

  image.copyTo(imageCopy);
  if(ids.size() > 0) {
    aruco::drawDetectedMarkers(imageCopy, corners, ids);
    cout<<ids.size()<<endl;

    for(unsigned int i = 0; i < ids.size(); i++){
      Vec3d r, t;
      aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
      markerLength * 0.5f);

      cout<<endl;
      // std::cout<<rvecs[i]<<endl;


      for(int k=0; k<corners[i].size(); ++k){
        cout<<corners[i][k]<<'\t';
      }
      cout<<endl;
      get6dofInFrameBodyCoordinates(rvecs[i], tvecs[i], r, t);

      cout<<r<<endl;
      cout<<t<<endl;

      dist = calculateDistance(t);
      if (minDist > dist){
          minDist = dist;
          theId = ids[i];
          flag = true;
        }

      rs.push_back(r);
      ts.push_back(t);
      }

    }

  cout<<"---"<<endl;


  imshow("out", imageCopy);
  waitKey(1);

}

int main(int argc, char *argv[]){

  readCameraParameters("/home/hamidreza/thesis/arcode/calib2.txt", camMatrix, distCoeffs);

  ros::init(argc, argv, "arcodeDetection");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<visual_guidance::sixDof>("visual_guide", 1);
  visual_guidance::sixDof smsg;

  // cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  // ros::Subscriber dynSub = nh.subscribe("dynamic_feedback", 1, dynCallback);
  image_transport::Subscriber sub = it.subscribe("/quadrotor_1/front/image_raw", 1, imageCallback);
  ros::Rate loop_rate(10); // Loop at 10Hz


  while (ros::ok())
  {
    if (flag)
    {
      smsg.x = tvecs[theId][0];
      smsg.y = tvecs[theId][1];
      smsg.z = tvecs[theId][2];
      smsg.a = rvecs[theId][0];
      smsg.b = rvecs[theId][1];
      smsg.c = rvecs[theId][2];
      pub.publish(smsg);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
