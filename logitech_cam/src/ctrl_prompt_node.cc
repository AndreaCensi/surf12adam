#include "ros/ros.h"
#include "std_msgs/String.h"
#include "CreeperCam.h"
#include "logitech_cam/CamCmd.h"
#include <iostream>
using namespace std;

/*
 * This node reads control signals prom the cammand prompt and send the
 * to the channel Instructions.
 *
 * Adam Nilsson, 7/1/12
 */

int main(int argc, char **argv)
{

	ROS_INFO("Instructor Starting");

	ros::init(argc, argv, "CameraInstructor");

	ros::NodeHandle n;
//	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("Instructions", 1000);
	ros::Publisher chatter_pub = n.advertise<logitech_cam::CamCmd>("Instructions", 1000);
	ros::Rate loop_rate(10);
	int count = 0;
	int input;
	while (ros::ok())
	{
//		std_msgs::String msg;
			logitech_cam::CamCmd msg;
		//  msg.data = "Hej";
//		std::stringstream ss;
//		ss << "hello world " << count;
		cin >> input;
//		msg.data = input;
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
