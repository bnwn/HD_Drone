#include "PN020Series.h"
#include "motor.h"
#include "../Algorithm/Algorithm_math/Algorithm_math.h"

int32_t motor_duty[MOTOR_MAX_NUM];
uint32_t motor_min_duty_cnt;
float motor_duty_scale = 1.0f;
uint32_t motor_duty_range;

void motor_init(void)
{
    PWM_ConfigOutputChannel(PWM, 0, MOTOR_PWM_FREQ, MOTOR_MIN_PWM_DUTY);
    PWM_ConfigOutputChannel(PWM, 1, MOTOR_PWM_FREQ, MOTOR_MIN_PWM_DUTY);
    PWM_ConfigOutputChannel(PWM, 2, MOTOR_PWM_FREQ, MOTOR_MIN_PWM_DUTY);
    PWM_ConfigOutputChannel(PWM, 3, MOTOR_PWM_FREQ, MOTOR_MIN_PWM_DUTY);
#if VEHICLE_FRAME == QUAD
    PWM_EnableOutput(PWM, 0x0F);
    PWM_Start(PWM, 0x0F);
#elif VEHICLE_FRAME == HEXA
    PWM_ConfigOutputChannel(PWM, 4, MOTOR_PWM_FREQ, MOTOR_MIN_PWM_DUTY);
    PWM_ConfigOutputChannel(PWM, 5, MOTOR_PWM_FREQ, MOTOR_MIN_PWM_DUTY);
    PWM_EnableOutput(PWM, 0x3F);
    PWM_Start(PWM, 0x3F);
#endif

#if MOTOR_TYPE == BLDC
    motor_min_duty_cnt = PWM->CMPDAT0;
    motor_duty_range = MOTOR_MAX_PWM - MOTOR_MIN_PWM;
    motor_duty_scale = (motor_min_duty_cnt + 1) / MOTOR_MIN_PWM;
#elif MOTOR_TYPE == DC
    motor_duty_range = PWM->PERIOD0;
#endif
}

void motor_update(int32_t *_duty)
{
    int i = 0;

    for (; i < MOTOR_MAX_NUM; i++) {
        if (*(_duty + i) < 0) {
            *(_duty + i) = 0;
        } else if (*(_duty + i) > motor_duty_range) {
            *(_duty + i) = motor_duty_range;
        }
        PWM_SET_CMR(PWM, i, (uint32_t)((*(_duty + i) + MOTOR_MIN_PWM) * motor_duty_scale));
    }
}

