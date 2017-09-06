#ifndef __POSITION_CONTROL_H
#define __POSITION_CONTROL_H

// position controller default definitions
#define POSCONTROL_ACCELERATION_MIN             50.0f   // minimum horizontal acceleration in cm/s/s - used for sanity checking acceleration in leash length calculation
#define POSCONTROL_ACCEL_XY                     100.0f  // default horizontal acceleration in cm/s/s.  This is overwritten by waypoint and loiter controllers
#define POSCONTROL_ACCEL_XY_MAX                 980.0f  // max horizontal acceleration in cm/s/s that the position velocity controller will ask from the lower accel controller
                                                        // should be 1.5 times larger than POSCONTROL_ACCELERATION.
                                                        // max acceleration = max lean angle * 980 * pi / 180.  i.e. 23deg * 980 * 3.141 / 180 = 393 cm/s/s
#define POSCONTROL_STOPPING_DIST_UP_MAX         300.0f  // max stopping distance (in cm) vertically while climbing
#define POSCONTROL_STOPPING_DIST_DOWN_MAX       200.0f  // max stopping distance (in cm) vertically while descending
#define POSCONTROL_JERK_LIMIT_CMSSS             1700.0f // default jerk limit on horizontal acceleration (unit: m/s/s/s)

#define POSCONTROL_SPEED                        500.0f  // default horizontal speed in cm/s
#define POSCONTROL_SPEED_DOWN                  -150.0f  // default descent rate in cm/s
#define POSCONTROL_SPEED_UP                     250.0f  // default climb rate in cm/s
#define POSCONTROL_VEL_XY_MAX_FROM_POS_ERR      200.0f  // max speed output from pos_to_vel controller when feed forward is used

#define POSCONTROL_ACCEL_Z                      250.0f  // default vertical acceleration in cm/s/s.

#define POSCONTROL_LEASH_LENGTH_MIN             100.0f  // minimum leash lengths in cm

#define POSCONTROL_DT_50HZ                      0.02f   // time difference in seconds for 50hz update rate
#define POSCONTROL_DT_400HZ                     0.0025f // time difference in seconds for 400hz update rate

#define POSCONTROL_ACTIVE_TIMEOUT_MS            200     // position controller is considered active if it has been called within the past 0.2 seconds

#define POSCONTROL_VEL_ERROR_CUTOFF_FREQ        4.0f    // low-pass filter on velocity error (unit: hz)
#define POSCONTROL_THROTTLE_CUTOFF_FREQ         0.059f    // low-pass filter on accel error (unit: 2hz)
#define POSCONTROL_ACCEL_FILTER_HZ              2.0f    // low-pass filter on acceleration (unit: hz)
#define POSCONTROL_JERK_RATIO                   1.0f    // Defines the time it takes to reach the requested acceleration
#define POSCONTROL_OVERSPEED_GAIN_Z             2.0f    // gain controlling rate at which z-axis speed is brought back within SPEED_UP and SPEED_DOWN range

// Throttle control gains
#ifndef ALT_HOLD_P
 # define ALT_HOLD_P            1.0f
#endif

// Velocity (vertical) control gains
#ifndef VEL_Z_P
 # define VEL_Z_P       5.0f
#endif

// Accel (vertical) control gains
#ifndef ACCEL_Z_P
 # define ACCEL_Z_P     0.50f
#endif
#ifndef ACCEL_Z_I
 # define ACCEL_Z_I     1.00f
#endif
#ifndef ACCEL_Z_D
 # define ACCEL_Z_D     0.0f
#endif
#ifndef ACCEL_Z_IMAX
 # define ACCEL_Z_IMAX  800
#endif
#ifndef ACCEL_Z_FILT_HZ
 # define ACCEL_Z_FILT_HZ   20.0f
#endif

// default maximum vertical velocity and acceleration the pilot may request
#ifndef POSCONTROL_VELZ_MAX
 # define POSCONTROL_VELZ_MAX    250     // maximum vertical velocity in cm/s
#endif
#ifndef POSCONTROL_ACCEL_Z_DEFAULT
 # define POSCONTROL_ACCEL_Z_DEFAULT 250 // vertical acceleration in cm/s/s while altitude is under pilot control
#endif

// max distance in cm above or below current location that will be used for the alt target when transitioning to alt-hold mode
#ifndef ALT_HOLD_INIT_MAX_OVERSHOOT
 # define ALT_HOLD_INIT_MAX_OVERSHOOT 200
#endif
// the acceleration used to define the distance-velocity curve
#ifndef ALT_HOLD_ACCEL_MAX
 # define ALT_HOLD_ACCEL_MAX 250    // if you change this you must also update the duplicate declaration in AC_WPNav.h
#endif

#ifndef AUTO_DISARMING_DELAY
# define AUTO_DISARMING_DELAY  10
#endif


/* function prototype */
void poscontrol_init_takeoff(void);
void position_z_controller(void);
void pos_to_rate_z(void);
// rate_to_accel_z - calculates desired accel required to achieve the velocity target
void rate_to_accel_z(void);
// accel_to_throttle - alt hold's acceleration controller
void accel_to_throttle(float _accel_target_z);
void add_takeoff_climb_rate(float _climb_rate_cms, float _dt);
void set_alt_target_from_climb_rate_ff(float climb_rate_cms, float dt, bool force_descend);
void relax_alt_controller(float throttle_setting);
void calc_leash_length_z(void);
float calc_leash_length(float speed_cms, float accel_cms, float kP);
// Proportional controller with piecewise sqrt sections to constrain second derivative
float sqrt_controller(float error, float p, float second_ord_lim);
/// set_speed_z - sets maximum climb and descent rates
/// To-Do: call this in the main code as part of flight mode initialisation
///     calc_leash_length_z should be called afterwards
///     speed_down should be a negative number
void set_speed_z(float _speed_down, float _speed_up);
/// set_accel_z - set vertical acceleration in cm/s/s
void set_accel_z(float _accel_cmss);
bool is_active_z(void);
void set_alt_target_to_current_alt(void);
void set_desired_vel_z(float _vel_z_cms);

#endif
