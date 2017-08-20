#include "PN020Series.h"
#include "attitude_control.h"

/* all control loop structure*/
_Ctrl ctrl_loop = {0};

_Target_Attitude attitude_target_ang_vel = {0};
_Target_Attitude attitude_target_ang = {0};
_Target_Attitude trace_attitude_ang = {0};
_Target_Attitude trace_attituce_ang_vel = {0};

void attitude_angle_rate_controller(void)
{
    _Vector_Float current_ang_vel = inertial_sensor.gyro.filter;

//	printf("velx:%d, vely:%d, velz:%d\n", (int16_t)(attitude_target_ang_vel.roll * 1000), (int16_t)(attitude_target_ang_vel.pitch * 1000), (int16_t)(attitude_target_ang_vel.yaw * 1000));
//	printf("velx:%d, vely:%d, velz:%d\n", (int16_t)(trace_attituce_ang_vel.roll * 1000), (int16_t)(trace_attituce_ang_vel.pitch * 1000), (int16_t)(trace_attituce_ang_vel.yaw * 1000));
    set_motor_roll(axis_target_pid_cal(&ctrl_loop.rate.roll, attitude_target_ang_vel.roll, current_ang_vel.x));
    set_motor_pitch(axis_target_pid_cal(&ctrl_loop.rate.pitch, attitude_target_ang_vel.pitch, current_ang_vel.y));
    set_motor_yaw(axis_target_pid_cal(&ctrl_loop.rate.yaw, attitude_target_ang_vel.yaw, current_ang_vel.z));
//		set_motor_roll(axis_target_pid_cal(&ctrl_loop.rate.roll, trace_attituce_ang_vel.roll, current_ang_vel.x));
//    set_motor_pitch(axis_target_pid_cal(&ctrl_loop.rate.pitch, trace_attituce_ang_vel.pitch, current_ang_vel.y));
//    set_motor_yaw(axis_target_pid_cal(&ctrl_loop.rate.yaw, trace_attituce_ang_vel.yaw, current_ang_vel.z));
}

float axis_target_pid_cal(Pid_t *_pid, float _target, float _current)
{
    float rate_error_rads = Rad(_target - _current);
		float output;
	
    set_pid_input(_pid, rate_error_rads);

//    float integrator = get_integrator(_pid);

//    // Ensure that integrator can only be reduced if the output is saturated
//    // if (!_motors.limit.roll_pitch || ((integrator > 0 && rate_error_rads < 0) || (integrator < 0 && rate_error_rads > 0))) {
//    if ((integrator > 0 && rate_error_rads < 0) || (integrator < 0 && rate_error_rads > 0))
//        integrator = _pid->I_Item_Output;
//    }

//    // Compute output in range -1 ~ +1
//    float output = get_rate_pitch_pid().get_p() + integrator + get_rate_pitch_pid().get_d() + get_rate_pitch_pid().get_ff(rate_target_rads);
    output = _pid->P_Item_Output + _pid->I_Item_Output + _pid->D_Item_Output;

    // Constrain output
    // return constrain_float(output, -1.0f, 1.0f);
    return output;
}

void attitude_angle_euler_controller(float _target_roll, float _target_pitch, float _target_yaw_rate, float sense_level, float _dt)
{
    float _target_yaw = ahrs.Yaw + _target_yaw_rate * _dt;

//	printf("roll:%d, pitch:%d, yaw:%d\n", (int16_t)(ahrs.Roll * 100), (int16_t)(ahrs.Pitch * 100), (int16_t)(ahrs.Yaw * 100));
    attitude_target_ang_vel.roll = axis_target_pid_cal(&ctrl_loop.angle.roll, _target_roll, ahrs.Roll);
    attitude_target_ang_vel.pitch = axis_target_pid_cal(&ctrl_loop.angle.pitch, _target_pitch, ahrs.Pitch);
    attitude_target_ang_vel.yaw = axis_target_pid_cal(&ctrl_loop.angle.yaw, _target_yaw, ahrs.Yaw);
}

void attitude_throttle_controller(float _throttle)
{
    set_motor_throttle(_throttle);
}
