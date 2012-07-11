#include "ros/ros.h"
#include "std_msgs/String.h"
#include "CreeperCam.h"
#include "logitech_cam/CamCmd.h"
#include <iostream>
#include "sensor_msgs/Image.h"


class ImageTunnel {
public:
	ImageTunnel(void);
	void camera_trigged(const std_msgs::String msg);
	void video_in(const sensor_msgs::Image msg);
};
