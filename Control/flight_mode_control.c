#include "PN020Series.h"
#include "flight_mode_control.h"
#include "motor_control.h"
#include "attitude_control.h"
#include "position_control.h"
#include "common.h"
#include "rc_channel.h"

enum Flight_Mode control_mode, prev_control_mode;
_Target_Attitude target_attitude = {0};

bool set_flight_mode(enum Flight_Mode _mode)
{
    bool success = false;
    bool ignore_checks = true;
    // bool ignore_checks = !armed();

    if (_mode == control_mode) {
        prev_control_mode = control_mode;
        return true;
    }

    switch (_mode) {
        case Stabilize:
            success = stabilize_init(ignore_checks);
            break;
        case AltHold:
            //success = althold_init(ignore_checks);
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
            //althold_run();
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
	reset_pid_param();
    if (_ignore_checks) {
        return true;
    }

#if 0
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);
#endif

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void stabilize_run(void)
{
    float target_throttle;

	if (fc_status.armed != DISARMED) {
		return;
	}
    target_throttle = get_desired_throttle_expo();
    get_desired_leans_angles(&target_attitude, CONTROL_LEANS_ANGLE_MAX_DEFAULT);

//    target_throttle = trace_throttle;
//    attitude_angle_euler_controller(trace_attitude_ang.roll, trace_attitude_ang.pitch, trace_attitude_ang.yaw, get_smoothing_gain(), 0.0025f);

    attitude_angle_euler_controller(target_attitude.roll, target_attitude.pitch, target_attitude.yaw, get_smoothing_gain(), LOOP_DT);
    attitude_throttle_controller(target_throttle, true, 0.0f);

#if 0
    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // clear landing flag
    set_land_complete(false);
    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();
#endif
}

bool acro_init(bool ignore_checks)
{
	reset_pid_param();
	return true;
}

void acro_run(void)
{
	float target_throttle;

	if (fc_status.armed != DISARMED) {
		return;
	}
	
    target_throttle = get_desired_throttle_expo();
	
	get_desired_leans_angles(&target_attitude, CONTROL_LEANS_ANGLE_MAX_DEFAULT);
	
    attitude_throttle_controller(target_throttle, true, 0.0f);
	attitude_target_ang_vel.roll = target_attitude.roll;
	attitude_target_ang_vel.pitch = target_attitude.pitch;
	attitude_target_ang_vel.yaw = target_attitude.yaw;
}

#if 0
// althold_init - initialise althold controller
bool Copter::althold_init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter Alt Hold if the Rotor Runup is not complete
    if (!ignore_checks && !motors->rotor_runup_complete()){
        return false;
    }
#endif

    // initialize vertical speeds and leash lengths
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // stop takeoff if running
    takeoff_stop();

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Copter::althold_run()
{
    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (target_climb_rate > 0.0f) && motors->rotor_runup_complete());
#else
    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);
#endif

    // Alt Hold State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        althold_state = AltHold_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        althold_state = AltHold_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
#if FRAME_CONFIG == HELI_FRAME
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
#else
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        pos_control->update_z_controller();
        break;

    case AltHold_Takeoff:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case AltHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        avoid.adjust_roll_pitch(target_roll, target_pitch, aparm.angle_max);
#endif

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
        break;
    }
}
#endif
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
