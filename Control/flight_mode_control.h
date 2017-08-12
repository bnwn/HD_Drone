#ifndef __FLIGHT_MODE_CONTROL
#define __FLIGHT_MODE_CONTROL

enum Flight_Mode {
		Stabilize = 0,
		AltHold = 1,
		OneKeyFlip = 2,
	  Acro = 3,
		Loiter = 4,
		PosHold = 5,
		Auto = 6
};

void flight_mode_update(enum Flight_Mode _mode);
#endif