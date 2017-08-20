#ifndef __MOTOR_CONTROL
#define __MOTOR_CONTROL

#include "../Driver/motor.h"
#include "../AHRS/ahrs.h"
#include "../Algorithm/Algorithm_math/mymath.h"
#include "../Algorithm/Algorithm_math/Algorithm_math.h"

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float throttle;
}Thrust;

void motors_output(void);
void throttle_control(float _dt);
void set_motor_roll(float _thrust);
void set_motor_pitch(float _thrust);
void set_motor_yaw(float _thrust);
void set_motor_throttle(float _thrust);
void set_trace_throttle(float _thr);

extern Thrust thrust;
extern float target_throttle;
extern float trace_throttle;
#endif
