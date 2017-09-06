#ifndef __ATTITUDE_CONTROL_H
#define __ATTITUDE_CONTROL_H

#include "../AHRS/inertial_sensor.h"
#include "../AHRS/ahrs.h"
#include "../Algorithm/Algorithm_math/Algorithm_math.h"
#include "../Algorithm/Algorithm_pid/Algorithm_pid.h"
#include "motor_control.h"

#define CONTROL_ANGLE_LOOP_ROLL_KP 150.0f
#define CONTROL_ANGLE_LOOP_ROLL_KI 0.0f
#define OONTROL_ANGLE_LOOP_ROLL_INTEGRATOR_MAX 0.0f
#define CONTROL_ANGLE_LOOP_ROLL_KD 0.0f
#define CONTROL_ANGLE_LOOP_PITCH_KP 150.0f
#define OONTROL_ANGLE_LOOP_PITCH_INTEGRATOR_MAX 0.0f
#define CONTROL_ANGLE_LOOP_PITCH_KI 0.0f
#define CONTROL_ANGLE_LOOP_PITCH_KD 0.0f
#define CONTROL_ANGLE_LOOP_YAW_KP 100.0f
#define CONTROL_ANGLE_LOOP_YAW_KI 0.0f
#define OONTROL_ANGLE_LOOP_YAW_INTEGRATOR_MAX 0.0f
#define CONTROL_ANGLE_LOOP_YAW_KD 0.0f
#define CONTROL_RATE_LOOP_ROLL_KP 0.18f
#define CONTROL_RATE_LOOP_ROLL_KI 0.0f
#define OONTROL_RATE_LOOP_ROLL_INTEGRATOR_MAX 0.3f
#define CONTROL_RATE_LOOP_ROLL_KD 0.0095f
#define CONTROL_RATE_LOOP_PITCH_KP 0.18f
#define CONTROL_RATE_LOOP_PITCH_KI 0.0f
#define OONTROL_RATE_LOOP_PITCH_INTEGRATOR_MAX 0.3f
#define CONTROL_RATE_LOOP_PITCH_KD 0.0095f
#define CONTROL_RATE_LOOP_YAW_KP 2.0f
#define CONTROL_RATE_LOOP_YAW_KI 0.0f
#define OONTROL_RATE_LOOP_YAW_INTEGRATOR_MAX 0.3f
#define CONTROL_RATE_LOOP_YAW_KD 0.0f
#define CONTROL_ACRO_SENSITITY_LOOP_YAW_KP 1.0f

//#define CONTROL_ANGLE_LOOP_ROLL_KP 400.0f
//#define CONTROL_ANGLE_LOOP_ROLL_KI 0.0f
//#define OONTROL_ANGLE_LOOP_ROLL_INTEGRATOR_MAX 0.0f
//#define CONTROL_ANGLE_LOOP_ROLL_KD 0.0f
//#define CONTROL_ANGLE_LOOP_PITCH_KP 400.0f
//#define OONTROL_ANGLE_LOOP_PITCH_INTEGRATOR_MAX 0.0f
//#define CONTROL_ANGLE_LOOP_PITCH_KI 0.0f
//#define CONTROL_ANGLE_LOOP_PITCH_KD 0.0f
//#define CONTROL_ANGLE_LOOP_YAW_KP 400.0f
//#define CONTROL_ANGLE_LOOP_YAW_KI 0.0f
//#define OONTROL_ANGLE_LOOP_YAW_INTEGRATOR_MAX 0.0f
//#define CONTROL_ANGLE_LOOP_YAW_KD 0.0f
//#define CONTROL_RATE_LOOP_ROLL_KP 0.023f
//#define CONTROL_RATE_LOOP_ROLL_KI 0.00013f
//#define OONTROL_RATE_LOOP_ROLL_INTEGRATOR_MAX 0.5f
//#define CONTROL_RATE_LOOP_ROLL_KD 0.0013f
//#define CONTROL_RATE_LOOP_PITCH_KP 0.023f
//#define CONTROL_RATE_LOOP_PITCH_KI 0.00013f
//#define OONTROL_RATE_LOOP_PITCH_INTEGRATOR_MAX 0.5f
//#define CONTROL_RATE_LOOP_PITCH_KD 0.0013f
//#define CONTROL_RATE_LOOP_YAW_KP 0.02f
//#define CONTROL_RATE_LOOP_YAW_KI 0.0f
//#define OONTROL_RATE_LOOP_YAW_INTEGRATOR_MAX 0.5f
//#define CONTROL_RATE_LOOP_YAW_KD 0.0f
//#define CONTROL_ACRO_SENSITITY_LOOP_YAW_KP 100.0f




/* function prototype */
void attitude_angle_rate_controller(void);
float axis_target_pid_cal(Pid_t *_pid, float _error);
void attitude_angle_euler_controller(float _target_roll, float _target_pitch, float _target_yaw_rate, float sense_level, float _dt);
void attitude_throttle_controller(float _throttle, bool _use_leans_tilt, float _cutoff);
float get_throttle_boosted(float _throttle_in);
void reset_rate_controller(void);

typedef struct {
    float roll;
    float pitch;
    float yaw;
}_Target_Attitude;

typedef struct {
    Pid_t roll;
    Pid_t pitch;
    Pid_t yaw;
}_Tache;

typedef struct {
    Pid_t x;
    Pid_t y;
    Pid_t z;
}_Pid_Vector;

typedef struct {
    _Tache rate;
    _Tache angle;
    _Tache acro_sensibility;
	_Pid_Vector pos;
	_Pid_Vector pos_vel;
	_Pid_Vector pos_accel;
}_Ctrl;

extern _Ctrl ctrl_loop;
extern _Target_Attitude attitude_target_ang_vel;
extern _Target_Attitude attitude_target_ang;
extern _Target_Attitude trace_attitude_ang;
extern _Target_Attitude trace_attituce_ang_vel;

#endif
