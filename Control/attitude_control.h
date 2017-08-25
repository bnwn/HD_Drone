#ifndef __ATTITUDE_CONTROL_H
#define __ATTITUDE_CONTROL_H

#include "../AHRS/inertial_sensor.h"
#include "../AHRS/ahrs.h"
#include "../Algorithm/Algorithm_math/Algorithm_math.h"
#include "../Algorithm/Algorithm_math/mymath.h"
#include "../Algorithm/Algorithm_pid/Algorithm_pid.h"
#include "motor_control.h"

#define CONTROL_ANGLE_LOOP_ROLL_KP 500.0f
#define CONTROL_ANGLE_LOOP_ROLL_KI 0.0f
#define OONTROL_ANGLE_LOOP_ROLL_INTEGRATOR_MAX 0.0f
#define CONTROL_ANGLE_LOOP_ROLL_KD 0.0f
#define CONTROL_ANGLE_LOOP_PITCH_KP 500.0f
#define OONTROL_ANGLE_LOOP_PITCH_INTEGRATOR_MAX 0.0f
#define CONTROL_ANGLE_LOOP_PITCH_KI 0.0f
#define CONTROL_ANGLE_LOOP_PITCH_KD 0.0f
#define CONTROL_ANGLE_LOOP_YAW_KP 0.0f
#define CONTROL_ANGLE_LOOP_YAW_KI 0.0f
#define OONTROL_ANGLE_LOOP_YAW_INTEGRATOR_MAX 0.0f
#define CONTROL_ANGLE_LOOP_YAW_KD 0.0f
#define CONTROL_RATE_LOOP_ROLL_KP 0.0285f
#define CONTROL_RATE_LOOP_ROLL_KI 0.000165f
#define OONTROL_RATE_LOOP_ROLL_INTEGRATOR_MAX 0.5f
#define CONTROL_RATE_LOOP_ROLL_KD 0.00178f
#define CONTROL_RATE_LOOP_PITCH_KP 0.0285f
#define CONTROL_RATE_LOOP_PITCH_KI 0.000165f
#define OONTROL_RATE_LOOP_PITCH_INTEGRATOR_MAX 0.5f
#define CONTROL_RATE_LOOP_PITCH_KD 0.00178f
#define CONTROL_RATE_LOOP_YAW_KP 0.0f
#define CONTROL_RATE_LOOP_YAW_KI 0.0f
#define OONTROL_RATE_LOOP_YAW_INTEGRATOR_MAX 0.5f
#define CONTROL_RATE_LOOP_YAW_KD 0.0f



/* function prototype */
void attitude_angle_rate_controller(void);
float axis_target_pid_cal(Pid_t *_pid, float _target, float _current);
void attitude_angle_euler_controller(float _target_roll, float _target_pitch, float _target_yaw_rate, float sense_level, float _dt);
void attitude_throttle_controller(float _throttle);

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
    _Tache rate;
    _Tache angle;
}_Ctrl;

extern _Ctrl ctrl_loop;
extern _Target_Attitude attitude_target_ang_vel;
extern _Target_Attitude attitude_target_ang;
extern _Target_Attitude trace_attitude_ang;
extern _Target_Attitude trace_attituce_ang_vel;

#endif
