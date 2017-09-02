#ifndef __FLIGHT_MODE_CONTROL
#define __FLIGHT_MODE_CONTROL
#include "attitude_control.h"

#define CONTROL_SENSE_LEVEL0 1

enum Flight_Mode {
    Stabilize = 0,
    AltHold = 1,
    OneKeyFlip = 2,
    Acro = 3,
    Loiter = 4,
    PosHold = 5,
    Auto = 6
};

#define CONTROL_LEANS_ANGLE_MAX_DEFAULT 30 // uint(degree)

bool set_flight_mode(enum Flight_Mode _mode);
void update_flight_mode(void);
void exit_mode(enum Flight_Mode _mode);
float get_smoothing_gain(void);

/* each flight mode control */
bool stabilize_init(bool ignore_checks);
void stabilize_run(void);
bool althold_init(bool ignore_checks);
void althold_run(void);
bool acro_init(bool ignore_checks);
void acro_run(void);

extern enum Flight_Mode control_mode;
extern _Target_Attitude target_attitude;

#endif
