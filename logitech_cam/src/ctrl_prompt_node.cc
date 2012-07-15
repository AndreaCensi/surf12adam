#include "ros/ros.h"
#include "std_msgs/String.h"
#include "CreeperCam.h"
#include "logitech_cam/CamCmd.h"
#include <iostream>
using namespace std;

/*
 *
 * Adam Nilsson, 7/2/12
 */


int main(int argc, char **argv)
{

	ROS_INFO("Instructor Starting");
	//ROS_INFO(argv[1]);
	ros::init(argc, argv, "CameraInstructor");

	ros::NodeHandle n;
//	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("Instructions", 1000);
	ros::Publisher chatter_pub = n.advertise<logitech_cam::CamCmd>("logger_instr", 1000);
	ros::Rate loop_rate(10);
	int count = 0;
	int input;
	while (ros::ok())
	{
		logitech_cam::CamCmd msg;

		cin >> input;
		msg.Pvalue = input;
		cin >> input;
		msg.Tvalue = input;
		ROS_INFO("going to send message");
		chatter_pub.publish(msg);
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}
//int main(int argc, char **argv)
//{
//
//	ROS_INFO("Instructor Starting");
//
//	ros::init(argc, argv, "Camera Trigger Console");
//
//	ros::NodeHandle n;
//	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("CameraTrigger", 1000);
//	ros::Rate loop_rate(10);
//	char *input;
//	while (ros::ok())
//	{
//		std_msgs::String msg;
//		cin >> input;
//		msg.data = input;
//		ROS_INFO("going to send message");
//		chatter_pub.publish(msg);
//		ros::spinOnce();
//
//		loop_rate.sleep();
//	}
//
//	return 0;
//}
