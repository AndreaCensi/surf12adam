#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "ImageTunnel.h"
#include "logitech_cam/CamCmd.h"
#include <iostream>

class cmd_action_logger_node{
public:
	void incoming_cmd(const logitech_cam::CamCmd msg);
	void video_in(const sensor_msgs::Image msg);
	void set_pic(const sensor_msgs::Image msg);
	void set_last_pic(const sensor_msgs::Image msg);
};
