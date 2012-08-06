/*
 *  Code From Stephanie's SURF Project 2011
 *  Class declaration moved to CreeperCam.h
 */


#include <stdio.h>
#include <stdlib.h>
#include "controls.h"
#include <webcam.h>
#include <sys/time.h>
#include "CreeperCam.h"
#define STALLTIME 2.0



/* Declares a camera object. Declares a ptz camera device and a chandle for the
   device. Then it resets the pan and tilt */
CreeperCam::CreeperCam(void) {

	CResult res = c_init();
	if (res) {
		printf("Exiting, could not open\n");
		exit(-1);
	}

	// Create the device
	const char *name = "video0";
	this->ptz_handle = c_open_device(name);
	if (!this->ptz_handle) {
		printf("Could not open device - error!\n");
	}
	this->max_pan = 4000;
	this->min_pan = -4000;
	this->max_tilt = 1400;
	this->min_tilt = -1600;

	this->Reset();
	this->stall(STALLTIME);
}

/* Closes the device */ 
void CreeperCam::close(void) {
	c_close_device(this->ptz_handle);
	c_cleanup();
}

/* Reset the device's position */
void CreeperCam::Reset(void) {

	// For some straing reason, this is a pan/tilt reset...
	const char *reset_name = "Tilt Reset";

	// Object that's needed for resetting the camera
	CControlValue reset_int;
	reset_int.value = 1; 

	// Control identifier for resetting the camera
	CControlId control_id;
	control_id = get_control_id(this->ptz_handle, reset_name);

	// Now actually reset the camera
	c_set_control(this->ptz_handle, control_id, &reset_int);

	// Stall to make sure tilt reset is done
	stall(1);

	const char *reset2 = "Pan Reset";
	control_id = get_control_id(this->ptz_handle, reset2);
	c_set_control(this->ptz_handle, control_id, &reset_int);

	this->current_pan = 0.0;
	this->current_tilt = 0.0;
}
/**
 * Pan the camera a relative amount
 * 7/1/12, Adam Nilsson
 */
bool CreeperCam::PanRelative(double amount) {
	double current = this->current_pan;
	if(current + amount <= this->max_pan && current + amount >= this->min_pan){
		// Get the control ID for moving the camera
		CControlId control_id;
		const char *name = "Pan (relative)";
		control_id = get_control_id(this->ptz_handle, name);

		// Control Value for moving the camera
		CControlValue pan_value;
		pan_value.value = amount;

		c_set_control(this->ptz_handle, control_id, &pan_value);

		this->current_pan += amount;
		return true;
	}
	else {
		warn("Pan limit reached");
		return false;
	}
}
/* Pan the device a certain amount */
void CreeperCam::Pan(double target) {

	// Find how much we should be moving the camera
	double current = this->current_pan;
	double amount_to_pan = target - current;

	// Get the control ID for moving the camera
	CControlId control_id;
	const char *name = "Pan (relative)";
	control_id = get_control_id(this->ptz_handle, name);

	// Control Value for moving the camera
	CControlValue pan_value;
	pan_value.value = amount_to_pan;

	c_set_control(this->ptz_handle, control_id, &pan_value);

	this->current_pan = target;
}
/**
 * Tilt the camera a relative amount
 /* 7/1/12, Adam Nilsson/
 */
bool CreeperCam::TiltRelative(double amount) {
	double current = this->current_tilt;
	if(current + amount <= this->max_tilt && current + amount >= this->min_tilt){
		// Get the control ID for moving the camera
		CControlId control_id;
		const char *name = "Tilt (relative)";
		control_id = get_control_id(this->ptz_handle, name);

		// Control Value
		CControlValue tilt_value;
		tilt_value.value = amount;

		CResult res = c_set_control(this->ptz_handle, control_id, &tilt_value);
		printf("result: %d\n", res);

		this->current_tilt += amount;
		return true;
	}
	else{
		warn("Tilt Limit Reached");
		return false;
	}
}
/* Tilt the device to a certain amount */ 
void CreeperCam::Tilt(double target) {

	/* Get the current tilt */
	double current = this->current_tilt;
	double amount_to_tilt = target - current;

	// Get the control ID for moving the camera
	CControlId control_id;
	const char *name = "Tilt (relative)";
	control_id = get_control_id(this->ptz_handle, name);

	// Control Value
	CControlValue tilt_value;
	tilt_value.value = amount_to_tilt;

	CResult res = c_set_control(this->ptz_handle, control_id, &tilt_value);
	printf("result: %d\n", res);

	this->current_tilt = target;
}

// These next two functions together stall the program to wait for the camera
// to move.
void CreeperCam::stall(double time) {

	timeval start;
	timeval end;

	gettimeofday(&start, NULL);
	gettimeofday(&end, NULL);

	while (timediff(start, end) < time) {
		gettimeofday(&end, NULL);
	}

}
double CreeperCam::timediff(timeval start, timeval end) {
	double difference = ((double) end.tv_usec - (double) start.tv_usec) /
			((double)1000000) +
			((double) end.tv_sec - (double) start.tv_sec);

	return difference;
}
/**
 * Send a warning message
 */
 void CreeperCam::warn(char *msg){
	 printf("Warning: CreeperCam:  ", msg);
 }
