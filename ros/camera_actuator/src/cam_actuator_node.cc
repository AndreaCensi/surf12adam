#include "ros/ros.h"
#include "std_msgs/String.h"
#include "CreeperCam.h"
#include "camera_actuator/CamCmd.h"
#include "camera_actuator/IntArray.h"
#include <cmath>

#include <sstream>
CreeperCam *camera;

ros::Publisher pub;

/**
 * Callback function for recieved IntArray of cammands
 * Message is assumed to have the form [Pan, Tilt, [Zoom]] as relative motion.
 * Zoom is ignored by the camera actuator node.
 *
 * Adam 7/10/12
 */
void receive_callback(const camera_actuator::IntArray msg)
{
	ROS_INFO("Recieved something");
//	camera->stall(0.3);
	bool cmd1 = camera->TiltRelative(msg.data[1]);
	camera->stall(abs(msg.data[1])*.2/100);
	bool cmd2 = camera->PanRelative(msg.data[0]);
	if(cmd1 && cmd2){
		ROS_INFO("Published command");
		pub.publish(msg);
	}
	else{
		ROS_INFO("Ignored command");
	}
}
int main(int argc, char **argv)
{
	int startup_time = 3;
	ros::init(argc, argv, "CameraController");

	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	int count = 0;

	camera = new CreeperCam();
	camera->Reset();
	camera->stall(startup_time);

	pub = n.advertise<camera_actuator::IntArray>("/logitech_cam/camera_executed",1000);
	ros::Subscriber sub = n.subscribe("/logitech_cam/camera_instr",1000, receive_callback);
	ROS_INFO("Camera Ready to take command");

	ros::spin();

	camera->close();
	delete[] camera;

	return 0;
}
