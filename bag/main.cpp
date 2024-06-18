/*
 * @description: 
 * @param : 
 * @return: 
 */
#include<iostream>
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;

void chatterDepthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO("I heard: depth");
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  cv::imwrite("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/get_image/bag/depth.png", cv_ptr->image);
}

void chatterColorCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO("I heard: color");
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  cv::imwrite("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/get_image/bag/color.png", cv_ptr->image);
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  
  ros::Subscriber sub_depth = n.subscribe("/camera/depth/image_rect_raw", 1, chatterDepthCallback);
  ros::Subscriber sub_color = n.subscribe("/camera/color/image_raw", 1, chatterColorCallback);
  ros::spin();
  return 0;
}