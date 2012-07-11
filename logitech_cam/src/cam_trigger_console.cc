#include "ros/ros.h"
#include "std_msgs/Int64.h"
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

		ros::init(argc, argv, "CameraTriggerConsole");

		ros::NodeHandle n;
		ros::Publisher chatter_pub = n.advertise<std_msgs::Int64>("CameraTrigger", 1000);
		ros::Rate loop_rate(10);
		int input;
		while (ros::ok())
		{
			std_msgs::Int64 msg;
			cin >> input;
			msg.data = input;
			ROS_INFO("going to send message");
			chatter_pub.publish(msg);
			ros::spinOnce();

			loop_rate.sleep();
		}
		return 0;
}
