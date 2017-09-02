#include "PN020Series.h"
#include "attitude_control.h"
#include "rc_channel.h"
#include "common.h"
#include "Algorithm_filter.h"

/* all control loop structure*/
_Ctrl ctrl_loop = {0};

_Target_Attitude attitude_target_ang_vel = {0};
_Target_Attitude attitude_target_ang = {0};
_Target_Attitude trace_attitude_ang = {0};
_Target_Attitude trace_attituce_ang_vel = {0};

void attitude_angle_rate_controller(void)
{
    _Vector_Float current_ang_vel = get_inertial_vel();

    set_motor_roll(axis_target_pid_cal(&ctrl_loop.rate.roll, (attitude_target_ang_vel.roll - current_ang_vel.x)));
    set_motor_pitch(axis_target_pid_cal(&ctrl_loop.rate.pitch, (attitude_target_ang_vel.pitch - current_ang_vel.y)));
    set_motor_yaw(axis_target_pid_cal(&ctrl_loop.rate.yaw, (attitude_target_ang_vel.yaw - current_ang_vel.z)));
//		set_motor_roll(axis_target_pid_cal(&ctrl_loop.rate.roll, trace_attituce_ang_vel.roll, current_ang_vel.x));
//    set_motor_pitch(axis_target_pid_cal(&ctrl_loop.rate.pitch, trace_attituce_ang_vel.pitch, current_ang_vel.y));
//    set_motor_yaw(axis_target_pid_cal(&ctrl_loop.rate.yaw, trace_attituce_ang_vel.yaw, current_ang_vel.z));
}

float axis_target_pid_cal(Pid_t *_pid, float _error)
{
    float rate_error_rads = Rad(_error);
	float output;
	
    set_pid_input(_pid, rate_error_rads);

//    // Compute output in range -1 ~ +1
//    float output = get_rate_pitch_pid().get_p() + integrator + get_rate_pitch_pid().get_d() + get_rate_pitch_pid().get_ff(rate_target_rads);
    output = _pid->P_Item_Output + _pid->I_Item_Output + _pid->D_Item_Output;

    return output;
}

void attitude_angle_euler_controller(float _target_roll, float _target_pitch, float _target_yaw_rate, float sense_level, float _dt)
{
    float _target_yaw = _target_yaw_rate;

    attitude_target_ang_vel.roll = sense_level * axis_target_pid_cal(&ctrl_loop.angle.roll, (_target_roll - ahrs.Roll));
    attitude_target_ang_vel.pitch = sense_level * axis_target_pid_cal(&ctrl_loop.angle.pitch, (_target_pitch - ahrs.Pitch));
    attitude_target_ang_vel.yaw = sense_level * axis_target_pid_cal(&ctrl_loop.angle.yaw, _target_yaw);
}

void attitude_throttle_controller(float _throttle, bool _use_leans_tilt, float _cutoff)
{
    if (_use_leans_tilt) {
        // Apply angle boost
        _throttle = get_throttle_boosted(_throttle);
    }
    set_motor_throttle(_throttle, _cutoff);
}

float get_throttle_boosted(float _throttle_in)
{
    // inverted_factor is 1 for tilt angles below 60 degrees
    // inverted_factor reduces from 1 to 0 for tilt angles between 60 and 90 degrees

    float cos_tilt = cos(AngE.Pitch) * cos(AngE.Roll);
    float inverted_factor = data_limit(2.0f*cos_tilt, 1.0f, 0.0f);
    float boost_factor = 1.0f / data_limit(cos_tilt, 1.0f, 0.5f);

    float throttle_out = _throttle_in*inverted_factor*boost_factor;
    data_limit(throttle_out, RC_THROTTLE_OUT_LIMIT, 0.0f);
    return throttle_out;
}

void reset_pid_param(void)
{
	reset_pid_I(&ctrl_loop.rate.roll);
	reset_pid_I(&ctrl_loop.rate.pitch);
	reset_pid_I(&ctrl_loop.rate.yaw);
	reset_pid_D(&ctrl_loop.rate.roll);
	reset_pid_D(&ctrl_loop.rate.pitch);
	reset_pid_D(&ctrl_loop.rate.yaw);
}
