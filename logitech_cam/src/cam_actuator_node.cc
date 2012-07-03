#include "ros/ros.h"
#include "std_msgs/String.h"
#include "CreeperCam.h"
#include "logitech_cam/CamCmd.h"
#include <cmath>

#include <sstream>
CreeperCam *camera;

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

int main(int argc, char **argv)
{

	ros::init(argc, argv, "CameraController");

	ros::NodeHandle n;
	//ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Subscriber sub = n.subscribe("Instructions",1000, receive_callback);
	ros::Rate loop_rate(10);

	int count = 0;
//	CreeperCam *camera;

	camera = new CreeperCam();
	camera->Reset();
	camera->stall(2.0);

	ros::spin();

	camera->close();
	camera->stall(3.0);
	delete[] camera;

	return 0;
}
