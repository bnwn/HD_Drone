#include "PN020Series.h"
#include "motor_control.h"
#include "Algorithm_filter.h"
#include "common.h"
#include "Algorithm_pid.h"

Motor_t motor = {0};
float target_throttle = 0;
float throttle_hover = THROTTLE_HOVER_DEFAULT;

/*====================================================================================================*/
/*====================================================================================================*
**函数 : motors_output(void)
**功能 : 电机控制
**输入 : None
**?出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void motors_output(void)
{
    motor_logic();

    if (motor.output_on) {
        motor_update(motor_duty);
    } else if (fc_status.armed != MOTOR_TEST) {
        motor_stop();
    }
}

void motor_logic(void)
{
    static float old_desired_throttle = 0;

    switch (motor.state) {
        case Motor_Unlimited:
            motor.limit_throttle_lower = false;
            motor.limit_throttle_upper = false;
            motor.output_on = true;

            if (motor.thrust.thr_cutoff > 0.01f) {
                motor.thrust.throttle = LPF_1st(old_desired_throttle, motor.thrust.throttle, motor.thrust.thr_cutoff);
                old_desired_throttle = motor.thrust.throttle;
            } else if (old_desired_throttle != 0.0f){
                old_desired_throttle = 0.0f;
            }
#if VEHICLE_FRAME == QUAD
            motor_duty[MOTOR1_INDEX] = motor.thrust.throttle + motor.thrust.pitch - motor.thrust.roll + motor.thrust.yaw;
            motor_duty[MOTOR2_INDEX] = motor.thrust.throttle - motor.thrust.pitch + motor.thrust.roll + motor.thrust.yaw;
            motor_duty[MOTOR3_INDEX] = motor.thrust.throttle + motor.thrust.pitch + motor.thrust.roll - motor.thrust.yaw;
            motor_duty[MOTOR4_INDEX] = motor.thrust.throttle - motor.thrust.pitch - motor.thrust.roll - motor.thrust.yaw;
    //		motor_duty[MOTOR1_INDEX] = 0.5;
    //		motor_duty[MOTOR2_INDEX] = 0.5;
    //		motor_duty[MOTOR3_INDEX] = 0.5;
    //		motor_duty[MOTOR4_INDEX] = 0.5;

#ifdef __DEVELOP__
            //printf ("%d, %d, %d, %d\n", (int16_t)(motor_duty[MOTOR1_INDEX]*1000), (int16_t)(motor_duty[MOTOR2_INDEX]*1000), (int16_t)(motor_duty[MOTOR3_INDEX]*1000), \
            //																												(int16_t)(motor_duty[MOTOR4_INDEX]*1000));
#endif
#elif VEHICLE_FRAME == HEXA
#endif
            break;
        case Motor_Spin:
            motor.limit_throttle_lower = false;
            motor.limit_throttle_upper = false;
            motor.output_on = true;

            motor_duty[MOTOR1_INDEX] = IDLED_DUTY;
            motor_duty[MOTOR2_INDEX] = IDLED_DUTY;
            motor_duty[MOTOR3_INDEX] = IDLED_DUTY;
            motor_duty[MOTOR4_INDEX] = IDLED_DUTY;
            break;
        case Motor_ShutDown:
            motor.limit_throttle_lower = true;
            motor.limit_throttle_upper = true;
            motor.output_on = false;

            motor_duty[MOTOR1_INDEX] = 0;
            motor_duty[MOTOR2_INDEX] = 0;
            motor_duty[MOTOR3_INDEX] = 0;
            motor_duty[MOTOR4_INDEX] = 0;
            break;
        default:
            break;
    }
}

void throttle_control(float dt)
{
    //thrust.throttle	= (target_throttle - 1000)/cos(ahrs.Roll/RtA)/cos(ahrs.Pitch/RtA);
		//thrust.throttle = target_throttle; 
}

void set_motor_roll(float _thrust)
{
    motor.thrust.roll = _thrust;
}

void set_motor_pitch(float _thrust)
{
    motor.thrust.pitch = _thrust;
}

void set_motor_yaw(float _thrust)
{
    motor.thrust.yaw = _thrust;
}

void set_motor_throttle(float _thrust, float _cutoff)
{
    motor.thrust.throttle = _thrust;
    motor.thrust.thr_cutoff = _cutoff;
}

void set_trace_throttle(float _thr)
{
    motor.thrust.throttle = _thr;
}

void set_motor_state(enum Motor_State _state)
{
    if (motor.state == _state) {
        return;
    }
    motor.state = _state;
}

float get_throttle_hover(void)
{
	return throttle_hover;
}
