#include "PN020Series.h"s
#include "flight_mode_control.h"
#include "motor_control.h"
#include "attitude_control.h"
#include "position_control.h"
#include "common.h"
#include "rc_channel.h"
#include "inertial_nav.h"

struct {
    bool running;
    float max_speed;
    float alt_delta;
    uint32_t start_ms;
} takeoff_state;

enum Flight_Mode control_mode, prev_control_mode;
_Target_Attitude target_attitude = {0};
float simple_cos_yaw, simple_sin_yaw;
float super_simple_last_bearing, super_simple_cos_yaw, super_simple_sin_yaw ;

bool set_flight_mode(enum Flight_Mode _mode)
{
    bool success = false;
    bool ignore_checks = (fc_status.armed == DISARMED);

    if (_mode == control_mode) {
        //prev_control_mode = control_mode;
        return true;
    }

    switch (_mode) {
        case Stabilize:
            success = stabilize_init(ignore_checks);
            break;
        case AltHold:
            success = althold_init(ignore_checks);
            break;
        case OneKeyFlip:
            break;
        case Acro:
			success = acro_init(ignore_checks);
            break;
        case Loiter:
            break;
        case PosHold:
            break;
        case Auto:
            break;
        default:
            break;
    }

    if (success) {
        exit_mode(control_mode);
        prev_control_mode = control_mode;
        control_mode = _mode;
    }
		
	return success;
}

void update_flight_mode(void)
{
    switch (control_mode) {
        case Stabilize:
            stabilize_run();
            break;
        case AltHold:
            althold_run();
            break;
        case OneKeyFlip:
            break;
        case Acro:
			acro_run();
            break;
        case Loiter:
            break;
        case PosHold:
            break;
        case Auto:
            break;
        default:
            break;
    }
}

void exit_mode(enum Flight_Mode _mode)
{

}

// stabilize_init - initialise stabilize controller
bool stabilize_init(bool _ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (fc_status.land_complete && !check_throttle_is_safe()) {
        return false;
    }

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void stabilize_run(void)
{
	static uint32_t timestamp = 0;
    float target_throttle;
	uint32_t now = sys_micro();
	float dt = (timestamp>0) ? ((float)(now-timestamp)/1000000.0f) : 0;
	timestamp = now;
	
    if (fc_status.armed == DISARMED) {
        set_motor_state(Motor_ShutDown);
		return;
    } else if (fc_status.armed == IDLED) {
        set_motor_state(Motor_Spin);
        return;
    }

    set_motor_state(Motor_Unlimited);

    target_throttle = get_desired_throttle_expo();
    get_desired_leans_angles(&target_attitude, CONTROL_LEANS_ANGLE_MAX_DEFAULT);

    attitude_angle_euler_controller(target_attitude.roll, target_attitude.pitch, target_attitude.yaw, get_smoothing_gain(), dt);
    attitude_throttle_controller(target_throttle, true, 0.0f);

    // clear landing flag
    set_land_complete(false);
}

bool acro_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (fc_status.land_complete && !check_throttle_is_safe()) {
        return false;
    }

    reset_rate_controller();

	return true;
}

void acro_run(void)
{
	float target_throttle;

    if (fc_status.armed == DISARMED) {
        set_motor_state(Motor_ShutDown);
        return;
    } else if (fc_status.armed == IDLED) {
        set_motor_state(Motor_Spin);
        return;
    }

    set_motor_state(Motor_Unlimited);

    target_throttle = get_desired_throttle_expo();
	
	get_desired_leans_angles(&target_attitude, CONTROL_LEANS_ANGLE_MAX_DEFAULT);
    update_simple_mode(&target_attitude);

    attitude_throttle_controller(target_throttle, true, 0.0f);
	attitude_target_ang_vel.roll = target_attitude.roll;
	attitude_target_ang_vel.pitch = target_attitude.pitch;
    attitude_target_ang_vel.yaw = target_attitude.yaw;
	// clear landing flag
    set_land_complete(false);
}

// althold_init - initialise althold controller
bool althold_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if ((fc_status.land_complete && !check_throttle_is_safe()) || !fc_status.inav_z_estimate_ok) {
        return false;
    }

    // initialize vertical speeds and leash lengths
    set_speed_z(-POSCONTROL_VELZ_MAX, POSCONTROL_VELZ_MAX);
    set_accel_z(POSCONTROL_ACCEL_Z_DEFAULT);

    // initialise position and desired velocity
    if (!is_active_z()) {
        set_alt_target_to_current_alt();
        set_desired_vel_z(get_inav_velocity().z);
    }

    // stop takeoff if running
    takeoff_stop();

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void althold_run(void)
{
    enum AltHold_Mode_State althold_state;
    float takeoff_climb_rate = 0.0f;
    float target_climb_rate = 0.0f;
    bool takeoff_triggered;
    static uint32_t timestamp = 0;
    uint32_t now = sys_micro();
    float dt = (timestamp>0) ? ((float)(now-timestamp)/1000000.0f) : 0;
    timestamp = now;

    get_desired_leans_angles(&target_attitude, CONTROL_LEANS_ANGLE_MAX_DEFAULT);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode(&target_attitude);

    // get desired climb rate
    target_climb_rate = get_desired_climb_rate();
    target_climb_rate = (float)constrain_float(target_climb_rate, -POSCONTROL_VELZ_MAX, POSCONTROL_VELZ_MAX);

    takeoff_triggered = fc_status.land_complete && (target_climb_rate > 0.0f);


    // Alt Hold State Machine Determination
    if (fc_status.armed == DISARMED || !fc_status.inav_z_estimate_ok || fc_status.armed == IDLED) {
        althold_state = AltHold_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        althold_state = AltHold_Takeoff;
    } else if (fc_status.land_complete) {
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:
        
		if (fc_status.armed == DISARMED) {
			set_motor_state(Motor_ShutDown);
		} else if (fc_status.armed == IDLED) {
			set_motor_state(Motor_Spin);
		}
        attitude_angle_euler_controller(target_attitude.roll, target_attitude.pitch, target_attitude.yaw, get_smoothing_gain(), dt);

        reset_rate_controller();
        target_attitude.yaw = 0.0f;
        relax_alt_controller(0.0f);   // forces throttle output to go to zero

        position_z_controller();
        break;

    case AltHold_Takeoff:
        // set motors to full range
        set_motor_state(Motor_Unlimited);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start((float)constrain_float(VEHIVLE_TAKEOFF_DEFAULT_ALT,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            poscontrol_init_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(&target_climb_rate, &takeoff_climb_rate);

        // call attitude controller
        attitude_angle_euler_controller(target_attitude.roll, target_attitude.pitch, target_attitude.yaw, get_smoothing_gain(), dt);

        // call position controller
        set_alt_target_from_climb_rate_ff(target_climb_rate, dt, false);
        add_takeoff_climb_rate(takeoff_climb_rate, dt);
        position_z_controller();
        break;

    case AltHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate <= 0.0f) {
            set_motor_state(Motor_Spin);
        } else {
            set_motor_state(Motor_Unlimited);;
        }

        attitude_angle_euler_controller(target_attitude.roll, target_attitude.pitch, target_attitude.yaw, get_smoothing_gain(), dt);

        reset_rate_controller();
        target_attitude.yaw = 0.0f;
        relax_alt_controller(0.0f);   // forces throttle output to go to zero

        position_z_controller();
        break;

    case AltHold_Flying:
        set_motor_state(Motor_Unlimited);

        // call attitude controller
        attitude_angle_euler_controller(target_attitude.roll, target_attitude.pitch, target_attitude.yaw, get_smoothing_gain(), dt);

        // adjust climb rate using rangefinder
//        if (rangefinder_alt_ok()) {
//            // if rangefinder is ok, use surface tracking
//            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
//        }

        // call position controller
        set_alt_target_from_climb_rate_ff(target_climb_rate, dt, false);
        position_z_controller();
        break;
    }
}

// start takeoff to specified altitude above home in centimeters
void takeoff_timer_start(float alt_cm)
{
    // calculate climb rate
    float speed = MIN(POSCONTROL_SPEED_UP, MAX(POSCONTROL_VELZ_MAX*2.0f/3.0f, POSCONTROL_VELZ_MAX-50.0f));

    // sanity check speed and target
    if (takeoff_state.running || speed <= 0.0f || alt_cm <= 0.0f) {
        return;
    }

    // initialise takeoff state
    takeoff_state.running = true;
    takeoff_state.max_speed = speed;
    takeoff_state.start_ms = sys_milli();
    takeoff_state.alt_delta = alt_cm;
}

// stop takeoff
void takeoff_stop()
{
    takeoff_state.running = false;
    takeoff_state.start_ms = 0;
}

// returns pilot and takeoff climb rates
//  pilot_climb_rate is both an input and an output
//  takeoff_climb_rate is only an output
//  has side-effect of turning takeoff off when timeout as expired
void takeoff_get_climb_rates(float *pilot_climb_rate, float *takeoff_climb_rate)
{
    // acceleration of 50cm/s/s
    static const float takeoff_accel = 50.0f;
    float takeoff_minspeed = MIN(50.0f,takeoff_state.max_speed);
    float time_elapsed = (sys_milli()-takeoff_state.start_ms)*1.0e-3f;
    float speed = MIN(time_elapsed*takeoff_accel+takeoff_minspeed, takeoff_state.max_speed);

    float time_to_max_speed = (takeoff_state.max_speed-takeoff_minspeed)/takeoff_accel;
    float height_gained;
	
    // return pilot_climb_rate if take-off inactive
    if (!takeoff_state.running) {
        *takeoff_climb_rate = 0.0f;
        return;
    }

    if (time_elapsed <= time_to_max_speed) {
        height_gained = 0.5f*takeoff_accel*squa(time_elapsed) + takeoff_minspeed*time_elapsed;
    } else {
        height_gained = 0.5f*takeoff_accel*squa(time_to_max_speed) + takeoff_minspeed*time_to_max_speed +
                        (time_elapsed-time_to_max_speed)*takeoff_state.max_speed;
    }

    // check if the takeoff is over
    if (height_gained >= takeoff_state.alt_delta) {
        takeoff_stop();
    }

    // if takeoff climb rate is zero return
    if (speed <= 0.0f) {
        *takeoff_climb_rate = 0.0f;
        return;
    }

    // default take-off climb rate to maximum speed
    *takeoff_climb_rate = speed;

    // if pilot's commands descent
    if (*pilot_climb_rate < 0.0f) {
        // if overall climb rate is still positive, move to take-off climb rate
        if (*takeoff_climb_rate + *pilot_climb_rate > 0.0f) {
            *takeoff_climb_rate = *takeoff_climb_rate + *pilot_climb_rate;
            *pilot_climb_rate = 0;
        } else {
            // if overall is negative, move to pilot climb rate
            *pilot_climb_rate = *pilot_climb_rate + *takeoff_climb_rate;
            *takeoff_climb_rate = 0.0f;
        }
    } else { // pilot commands climb
        // pilot climb rate is zero until it surpasses the take-off climb rate
        if (*pilot_climb_rate > *takeoff_climb_rate) {
            *pilot_climb_rate = *pilot_climb_rate - *takeoff_climb_rate;
        } else {
            *pilot_climb_rate = 0.0f;
        }
    }
}

float get_smoothing_gain(void)
{
    float sense_gain = 1;

    switch (control_mode) {
    case Stabilize:
        sense_gain = CONTROL_SENSE_LEVEL0;
        break;
    default:
        sense_gain = CONTROL_SENSE_LEVEL0;
        break;
    }

    return sense_gain;
}

void init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = cosf(AngE.Yaw);
    simple_sin_yaw = sinf(AngE.Yaw);

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_degree(ahrs.Yaw+180.0f);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;
}

// update_simple_mode - rotates pilot input if we are in simple mode
void update_simple_mode(_Target_Attitude *_target_att)
{
    // exit immediately if no new radio frame or not in simple mode
    if (fc_status.simple_mode == Normal) {
        return;
    }

    if (fc_status.simple_mode == Simple) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        _target_att->roll = _target_att->roll*simple_cos_yaw - _target_att->pitch*simple_sin_yaw;
        _target_att->pitch = _target_att->roll*simple_sin_yaw + _target_att->pitch*simple_cos_yaw;
    }else{
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        _target_att->roll = _target_att->roll*super_simple_cos_yaw - _target_att->pitch*super_simple_sin_yaw;
        _target_att->pitch = _target_att->roll*super_simple_sin_yaw + _target_att->pitch*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    _target_att->roll = _target_att->roll*cosf(AngE.Yaw) + _target_att->pitch*sinf(AngE.Yaw);
    _target_att->pitch = -_target_att->roll*sinf(AngE.Yaw) + _target_att->pitch*cosf(AngE.Yaw);
}

