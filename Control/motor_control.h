#ifndef __MOTOR_CONTROL
#define __MOTOR_CONTROL

#include "../Driver/motor.h"
#include "../AHRS/ahrs.h"
#include "../Algorithm/Algorithm_math/Algorithm_math.h"

#define IDLED_DUTY 0.1
#define THROTTLE_HOVER_DEFAULT 0.35f

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float throttle;
    float thr_cutoff;
}Thrust;

void motors_output(void);
void throttle_control(float _dt);
void set_motor_roll(float _thrust);
void set_motor_pitch(float _thrust);
void set_motor_yaw(float _thrust);
void set_motor_throttle(float _thrust, float _cutoff);
void set_trace_throttle(float _thr);
float get_throttle_hover(void);

extern Thrust thrust;
extern float target_throttle;
#endif
