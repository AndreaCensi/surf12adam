#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "ImageTunnel.h"
#include "logitech_cam/CamCmd.h"
#include <iostream>
//using namespace std;

/*
 * This node reads an image_raw and send out the same imageon one channel. 
 * A single image can be asked for at a time by signal at camtrigger and 
 * the actual image will be returned on CameraPic  
 *
 * Adam Nilsson, 7/1/12
 */
ros::Publisher video_pub;
ros::Publisher camera_pub;
int trig;
void camera_trigged(const std_msgs::Int64 msg)
{
	ROS_INFO("Camera Trigged");
	trig = 1;
}
void video_in(const sensor_msgs::Image msg)
{
//	ROS_INFO("Camera Trigged");
	video_pub.publish(msg);
	if(trig == 1){
		camera_pub.publish(msg);
		trig = 0;
	}
}
int main(int argc, char **argv)
{
	trig = 0;
	ROS_INFO("Instructor Starting");

	ros::init(argc, argv, "ImageTunnel");

	ros::NodeHandle n;
	ros::Subscriber video_sub = n.subscribe("/usb_cam/image_raw",1000, video_in);
	ros::Subscriber trigger_sub = n.subscribe("CameraTrigger",1000, camera_trigged);
	video_pub = n.advertise<sensor_msgs::Image>("VideoOut", 1000);
	camera_pub = n.advertise<sensor_msgs::Image>("CameraOut", 1000);
	ros::Rate loop_rate(10);
	ros::spin();
	return 0;
}
