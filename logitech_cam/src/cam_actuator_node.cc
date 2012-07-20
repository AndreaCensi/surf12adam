#include "ros/ros.h"
#include "std_msgs/String.h"
#include "CreeperCam.h"
#include "logitech_cam/CamCmd.h"
#include "logitech_cam/IntArray.h"
#include <cmath>

#include <sstream>
CreeperCam *camera;

ros::Publisher pub;
/*
//void receive_callback(const std_msgs::String::ConstPtr& msg)
void receive_callback(const logitech_cam::CamCmd msg)
{
	ROS_INFO("Recieved something");
	camera->TiltRelative(msg.Tvalue);
	camera->stall(abs(msg.Tvalue)*.2/100);
	camera->PanRelative(msg.Pvalue);
	//ROS_INFO("Panorate amount: ", msg.Pvalue);
	//ROS_INFO("Tilt amount: ", msg.Tvalue);
}
*/

/**
 * Callback function for recieved IntArray of cammands
 * Message is assumed to have the form [Pan, Tilt, [Zoom]] as relative motion.
 * Zoom is ignored by the camera actuator node.
 *
 * Adam 7/10/12
 */
void receive_callback(const logitech_cam::IntArray msg)
{
	ROS_INFO("Recieved something");
	#camera->stall(0.3);
	bool cmd1 = camera->TiltRelative(msg.data[1]);
	camera->stall(abs(msg.data[1])*.2/100);
	bool cmd2 = camera->PanRelative(msg.data[0]);
	if(cmd1 && cmd2){
		ROS_INFO("Published command");
		pub.publish(msg);
	}
	else{
		ROS_INFO("Ignoring command");
	}
}
int main(int argc, char **argv)
{
	int startup_time = 5;
	ros::init(argc, argv, "CameraController");

	ros::NodeHandle n;
	//ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Rate loop_rate(10);

	int count = 0;
//	CreeperCam *camera;

	camera = new CreeperCam();
	camera->Reset();
	camera->stall(startup_time);

	pub = n.advertise<logitech_cam::IntArray>("/logitech_cam/camera_executed",1000);
	ros::Subscriber sub = n.subscribe("/logitech_cam/camera_instr",1000, receive_callback);
	ROS_INFO("Camera Ready");

	ros::spin();

	camera->close();
	camera->stall(3.0);
	delete[] camera;

	return 0;
}
