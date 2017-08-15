#ifndef __ATTITUDE_CONTROL_H
#define __ATTITUDE_CONTROL_H

#include "../AHRS/inertial_sensor.h"
#include "../AHRS/ahrs.h"
#include "../Algorithm/Algorithm_math/Algorithm_math.h"
#include "../Algorithm/Algorithm_math/mymath.h"
#include "../Algorithm/Algorithm_pid/Algorithm_pid.h"
#include "motor_control.h"

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

#endif
