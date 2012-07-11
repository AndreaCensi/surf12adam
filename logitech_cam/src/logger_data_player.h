#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "ImageTunnel.h"
#include "logitech_cam/CamCmd.h"
#include <iostream>

class logger_data_player{
public:
	void data_in(const logitech_cam::CamCmd msg);
};
