#include <stdio.h>
#include <stdlib.h>
#include "controls.h"
#include <webcam.h>
#include <sys/time.h>

class CreeperCam {
	public:
		CreeperCam(void);
		void Pan(double target_pan);
		void Tilt(double target_tilt);
		bool PanRelative(double amount);
		bool TiltRelative(double amount);
		void Reset(void);
		void close(void);
		void stall(double time);
		void warn(char *msg); // Send a warning message

	private:
		// Current Positions
		double current_pan;
		double current_tilt;
		double timediff(timeval start, timeval end);

		// Hard coded limits for pan and tilt
		double max_pan;
		double min_pan;
		double max_tilt;
		double min_tilt;

		// Device and controls
		CHandle ptz_handle;
};
