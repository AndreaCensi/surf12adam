#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "ImageTunnel.h"
#include "logitech_cam/CamCmd.h"
#include "logitech_cam/CmdActionData.h"
#include <iostream>
//using namespace std;

/*
 *
 * Adam Nilsson, 7/2/12
 */
ros::Publisher pubY0;
ros::Publisher pubY1;
/**
 * Read input command, save initial image, execute command and read final image.
 */
void data_in(const logitech_cam::CmdActionData msg)
{
	ROS_INFO("Logged Command");
	pubY0.publish(msg.Y0);
	pubY1.publish(msg.Y1);
}

int main(int argc, char **argv)
{
	ROS_INFO("Logger Player Starting");

	ros::init(argc, argv, "LoggerDataPlayer");

	ros::NodeHandle n;

	// Listen to input logger data
	ros::Subscriber instream = n.subscribe("/LoggerData",1000, data_in);


	// Send out the instruction
	pubY0 = n.advertise<sensor_msgs::Image>("loggerY0", 1000);

	// Send data to logger
	pubY1 = n.advertise<sensor_msgs::Image>("loggerY1", 1000);


	ros::Rate loop_rate(10);
	ros::spin();
	return 0;
}
