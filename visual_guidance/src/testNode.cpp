#include "visual_guidance/sixDof.h"
#include "ArcodeDetector.h"
// #include "pathGeneration.h"


ArcodeDetector ad("/home/hamidreza/thesis/arcode/calib2.txt");


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

  Mat image;
  image = cv_bridge::toCvShare(msg, "bgr8")->image;

  ad.imageCallback(image);

}


// void shelfCallback();


int main(int argc, char *argv[]){

  ros::init(argc, argv, "cvi");
  ros::NodeHandle nh;

  // ros::Publisher pub = nh.advertise<visual_guidance::sixDof>("visual_guide", 1);
  // visual_guidance::sixDof smsg;

  // cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  // image_transport::Subscriber sub = it.subscribe("/quadrotor_1/front/image_raw", 1,\
   imageCallback);
  ros::Rate loop_rate(10); // Loop at 10Hz


  // while (ros::ok())
  // {
  //   if (flag)
  //   {
  //     smsg.x = tvecs[theId][0];
  //     smsg.y = tvecs[theId][1];
  //     smsg.z = tvecs[theId][2];
  //     smsg.a = rvecs[theId][0];
  //     smsg.b = rvecs[theId][1];
  //     smsg.c = rvecs[theId][2];
  //     pub.publish(smsg);
  //   }
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

  return 0;
}
