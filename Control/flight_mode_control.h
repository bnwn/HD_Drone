#ifndef __FLIGHT_MODE_CONTROL
#define __FLIGHT_MODE_CONTROL

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

bool set_flight_mode(enum Flight_Mode _mode);
void update_flight_mode();
void exit_mode(enum Flight_Mode _mode);
float get_smoothing_gain();

/* each flight mode control */
bool stabilize_init(bool ignore_checks);
void stabilize_run();
bool althold_init(bool ignore_checks);
void althold_run();

extern enum Flight_Mode control_mode;

#endif
