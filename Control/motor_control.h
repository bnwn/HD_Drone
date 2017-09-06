#ifndef __MOTOR_CONTROL
#define __MOTOR_CONTROL

#include "../Driver/motor.h"
#include "../AHRS/ahrs.h"
#include "../Algorithm/Algorithm_math/Algorithm_math.h"

#define IDLED_DUTY 0.08f
#define THROTTLE_HOVER_DEFAULT 0.35f

enum Motor_State {
    Motor_ShutDown = 0,
    Motor_Spin = 1,
    Motor_Unlimited = 2
};


typedef struct {
    float roll;
    float pitch;
    float yaw;
    float throttle;
    float thr_cutoff;
}Thrust;

typedef struct {
    bool output_on;
    enum Motor_State state;
    Thrust thrust;
    bool limit_throttle_lower;
    bool limit_throttle_upper;
}Motor_t;

void motors_output(void);
void motor_logic(void);
void throttle_control(float _dt);
void set_motor_roll(float _thrust);
void set_motor_pitch(float _thrust);
void set_motor_yaw(float _thrust);
void set_motor_throttle(float _thrust, float _cutoff);
void set_trace_throttle(float _thr);
void set_motor_state(enum Motor_State _state);
float get_throttle_hover(void);

extern Motor_t motor;
extern float target_throttle;
#endif
