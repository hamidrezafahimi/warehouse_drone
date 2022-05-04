#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <std_srvs/Empty.h>
// #include <ardrone_autonomy/CamSelect.h>
// #include <tf/transform_listener.h>
// #include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>

#include "arucoBoardDetector.h"

using namespace std;
// using namespace tf;

// tf::TransformListener *listener;
ros::ServiceClient client;
ros::Publisher vis_pub;
ros::Publisher quad_pub;
ros::Publisher target_pub;
image_transport::Publisher pub;
bool new_image_recieved;
int timeout;
int image_counter = 0;
int factor = 1;
arucoBoardDetector *detector;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
 //  	image_counter++;
 //  	if (image_counter == timeout){
	//   	new_image_recieved = true;
	//   	ardrone_autonomy::CamSelect srv;
	// 	srv.request.channel = 0;
	// 	client.call(srv);
	// }
	// else{
	// 	return;
	// }

  	Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
  	Mat result;
  	Vec3f target = detector->findTargetInCamFrame(&image, &result,factor);

  	if(!result.empty()) {
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

  	if(target[0] == 0 && target[1] == 0 && target[2] == 0)
  		return;

	// geometry_msgs::Point target_msg;
	// target_msg.x = target[0];
	// target_msg.y = target[1];
	// target_msg.z = target[2];

	// target_pub.publish(target_msg);
  	ros::Time now = ros::Time::now();
 	// tf::Point target_fcf(-target[0], target[2], target[1]);
 	// Stamped< tf::Point > target_fcf_stamped(target_fcf, now, "/cam_front");
 	// Stamped< tf::Point > target_blf_stamped;
 	// //Stamped< tf::Point > target_fcf_stamped;
 	// Stamped< tf::Point > target_mf_stamped;

  	// ROS_INFO("Target detected: x = %f , y = %f , z = %f", target[0], target[1], target[2]);



    // try{
    //   listener->waitForTransform("/ardrone_base_bottomcam", "/ardrone_base_frontcam", now, ros::Duration(0.5));
    //   listener->transformPoint("/ardrone_base_frontcam", target_bcf_stamped, target_fcf_stamped);
    // }
    // catch (tf::TransformException &ex) {
    //   ROS_ERROR("Transform from base_link frame to front cam frame failed! %s",ex.what());
    //   return;
    // }

 //    try{
 //      listener->waitForTransform("/cam_front", "/map", now, ros::Duration(0.1));
 //      listener->transformPoint("/map", target_fcf_stamped, target_mf_stamped);
 //    }
 //    catch (tf::TransformException &ex) {
 //      ROS_ERROR("Transform from front cam frame to map frame failed! %s",ex.what());
 //      return;
 //    }

 //  	ROS_INFO("Target detected: x = %f , y = %f , z = %f", target_mf_stamped.getX(), target_mf_stamped.getY(), target_mf_stamped.getZ());
  
 //  	visualization_msgs::Marker marker;
	// marker.header.frame_id = "/map";
	// marker.header.stamp = ros::Time::now();
	// marker.lifetime = ros::Duration();
	// marker.ns = "landing_platform";
	// marker.id = 0;
	// marker.type = visualization_msgs::Marker::CUBE;
	// marker.action = visualization_msgs::Marker::ADD;
	// marker.pose.position.x = target_mf_stamped.getX();
	// marker.pose.position.y = target_mf_stamped.getY();
	// marker.pose.position.z = target_mf_stamped.getZ();
	// marker.pose.orientation.x = 0.0;
	// marker.pose.orientation.y = 0.0;
	// marker.pose.orientation.z = 0.0;
	// marker.pose.orientation.w = 1.0;
	// marker.scale.x = 1;
	// marker.scale.y = 1;
	// marker.scale.z = 0.1;
	// marker.color.a = 1.0; 
	// marker.color.r = 1.0;
	// marker.color.g = 0.0;
	// marker.color.b = 0.0;

	// marker.lifetime = ros::Duration();
	// // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	// vis_pub.publish( marker );

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
  }
}

// void poseCallback(const tum_ardrone::filter_stateConstPtr& msg){

// 	visualization_msgs::Marker marker;
// 	marker.header.frame_id = "/map";
// 	marker.header.stamp = ros::Time::now();
// 	marker.lifetime = ros::Duration();
// 	marker.ns = "quad";
// 	marker.id = 1;
// 	marker.type = visualization_msgs::Marker::CUBE;
// 	marker.action = visualization_msgs::Marker::ADD;
// 	marker.pose.position.x = msg->x;
// 	marker.pose.position.y = msg->y;
// 	marker.pose.position.z = msg->z;
// 	marker.scale.x = 0.1;
// 	marker.scale.y = 0.2;
// 	marker.scale.z = 0.03;
// 	marker.color.a = 1.0; 
// 	marker.color.r = 0.0;
// 	marker.color.g = 1.0;
// 	marker.color.b = 0.0;

// 	marker.lifetime = ros::Duration();
// 	// marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
// 	quad_pub.publish( marker );
// }

int main(int argc, char **argv)
{
  float image_request_freq = 0;
  // if (argc > 3){

  //   string freq_str = argv[1];
  //   string timeout_str = argv[2];

  //   image_request_freq = stof(freq_str, NULL);
  //   timeout = stoi(timeout_str);
  //   factor = image_request_freq;
  // }
  // else{
  //   ROS_ERROR("Not enough arguments for platform detection.");
  //   return 0;
  // }

  detector = new arucoBoardDetector(argc, argv, 10);

  ros::init(argc, argv, "platform_detection");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  pub = it.advertise("/landing/image", 1);
  image_transport::Subscriber sub = it.subscribe("/iris_0/camera_bottom/image_raw", 1, imageCallback);
  // ros::Subscriber pose_sub = nh.subscribe("/ardrone/predictedPose", 1, poseCallback);

  // ros::Timer image_request_timer = nh.createTimer(ros::Duration(1.0/image_request_freq), request_image);

  ros::spin();
}