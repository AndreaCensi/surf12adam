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
ros::Publisher logger;
ros::Publisher commander;
sensor_msgs::Image picture1;
sensor_msgs::Image picture2;
int takepic;

void set_pic(const sensor_msgs::Image new_pic){
	ROS_INFO("Take a new picture");
	picture1 = new_pic;
}
void set_last_pic(const sensor_msgs::Image new_pic){
	ROS_INFO("Store an old picture");
	picture2 = new_pic;
}
/**
 * Read input command, save initial image, execute command and read final image.
 */

void incoming_cmd(const logitech_cam::CamCmd msg)
{
	ROS_INFO("incoming_cmd");
	logitech_cam::CmdActionData actiondata;
	ROS_INFO("logitech_cam::CmdActionData actiondata;");

	actiondata.U0 = msg;
	ROS_INFO("actiondata.U0 = msg;");
	commander.publish(msg);
	ROS_INFO("Going to sleep");
	sleep(3);
	ROS_INFO("Sleeped for 3000ms?");
	takepic = 1;
	ROS_INFO("takepic = 1;");
	sleep(1);
	actiondata.Y0 = picture1;
	actiondata.Y1 = picture2;
	logger.publish(actiondata);
}
/**
 * Read input images and save them to object picture.
 */
void video_in(const sensor_msgs::Image msg){
//	ROS_INFO("video_in");
	if(takepic != 0){
//		picture1 = msg;
		set_last_pic(picture1);
		set_pic(msg);
		ROS_INFO("Picture stored");
		takepic = 0;
	}
//	if(takepic == 2){
//		picture2 = msg;
//		ROS_INFO("Picture1 stored");
//		takepic = 0;
//	}
}
int main(int argc, char **argv)
{
	takepic = 0;
	ROS_INFO("Cmd Action Logger Starting");

	ros::init(argc, argv, "CmdActionLogger");

	ros::NodeHandle n;

	// Listen to input images
	ros::Subscriber camerain = n.subscribe("/usb_cam/image_raw",1000, video_in);

	// Listen to commands on logger instructions
	ros::Subscriber instrin = n.subscribe("logger_instr",1000, incoming_cmd);

	// Send out the instruction
	commander = n.advertise<logitech_cam::CamCmd>("camera_instr", 1000);

	// Send data to logger
	logger = n.advertise<logitech_cam::CmdActionData>("LoggerData", 1000);


	ros::Rate loop_rate(10);
	ros::spin();
	return 0;
}
