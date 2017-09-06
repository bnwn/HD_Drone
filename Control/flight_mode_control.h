#ifndef __FLIGHT_MODE_CONTROL
#define __FLIGHT_MODE_CONTROL
#include "attitude_control.h"

#define CONTROL_SENSE_LEVEL0 1
#define CONTROL_LEANS_ANGLE_MAX_DEFAULT 45 // uint(degree)
#define VEHIVLE_TAKEOFF_DEFAULT_ALT 0 //uint cm

enum Flight_Mode {
    Stabilize = 0,
    AltHold = 1,
    OneKeyFlip = 2,
    Acro = 3,
    Loiter = 4,
    PosHold = 5,
    Auto = 6,
    Land = 7
};

enum AltHold_Mode_State {
    AltHold_MotorStopped = 0,
    AltHold_Takeoff,
    AltHold_Flying,
    AltHold_Landed
};


bool set_flight_mode(enum Flight_Mode _mode);
void update_flight_mode(void);
void exit_mode(enum Flight_Mode _mode);
float get_smoothing_gain(void);

/* each flight mode control */
bool stabilize_init(bool ignore_checks);
void stabilize_run(void);
bool althold_init(bool ignore_checks);
void althold_run(void);
// start takeoff to specified altitude above home in centimeters
void takeoff_timer_start(float alt_cm);
// stop takeoff
void takeoff_stop(void);
// returns pilot and takeoff climb rates
//  pilot_climb_rate is both an input and an output
//  takeoff_climb_rate is only an output
//  has side-effect of turning takeoff off when timeout as expired
void takeoff_get_climb_rates(float *pilot_climb_rate, float *takeoff_climb_rate);
bool acro_init(bool ignore_checks);
void acro_run(void);
void init_simple_bearing(void);
// update_simple_mode - rotates pilot input if we are in simple mode
void update_simple_mode(_Target_Attitude *_target_att);

extern enum Flight_Mode control_mode;
extern _Target_Attitude target_attitude;

#endif
