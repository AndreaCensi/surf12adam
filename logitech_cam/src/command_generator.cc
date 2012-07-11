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

	ROS_INFO("Random Command Generator Starting");
	//ROS_INFO(argv[1]);
	ros::init(argc, argv, "command_generator");

	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<logitech_cam::CamCmd>("/logitech_cam/camera_instr", 1000);
	ros::Rate loop_rate(1);
	int count = 0;

	logitech_cam::CamCmd commands[6];
	commands[0].Pvalue = 250;
	commands[0].Tvalue = 0;
	commands[0].Zvalue = 0;
	commands[1].Pvalue = -250;
	commands[1].Tvalue = 0;
	commands[1].Zvalue = 0;
	commands[2].Pvalue = 0;
	commands[2].Tvalue = 150;
	commands[2].Zvalue = 0;
	commands[3].Pvalue = 0;
	commands[3].Tvalue = -150;
	commands[3].Zvalue = 0;
	commands[4].Pvalue = 0;
	commands[4].Tvalue = 0;
	commands[4].Zvalue = -10;
	commands[5].Pvalue = 0;
	commands[5].Tvalue = 0;
	commands[5].Zvalue = 10;

//	commands[4].Pvalue = 250;
//	commands[4].Tvalue = 0;
//	commands[4].Zvalue = 0;
//	commands[5].Pvalue = -250;
//	commands[5].Tvalue = 0;
//	commands[5].Zvalue = 0;
//	commands[6].Pvalue = 0;
//	commands[6].Tvalue = 150;
//	commands[6].Zvalue = 0;
//	commands[7].Pvalue = 0;
//	commands[7].Tvalue = -150;
//	commands[7].Zvalue = 0;
//
//	commands[8].Pvalue = 250;
//	commands[8].Tvalue = 0;
//	commands[8].Zvalue = 10;
//	commands[9].Pvalue = -250;
//	commands[9].Tvalue = 0;
//	commands[9].Zvalue = 10;
//	commands[10].Pvalue = 0;
//	commands[10].Tvalue = 150;
//	commands[10].Zvalue = 10;
//	commands[11].Pvalue = 0;
//	commands[11].Tvalue = -150;
//	commands[11].Zvalue = 10;

	ros::Duration(10).sleep();
	ROS_INFO("Ready to send commands");
	srand(time(NULL));
	while (ros::ok())
	{
		ROS_INFO("going to send message");
		int ci = rand() % 6;
		chatter_pub.publish(commands[ci]);
		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
