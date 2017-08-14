#include "PN020Series.h"
#include "motor.h"

int32_t motor_duty[MOTOR_MAX_NUM];
uint32_t motor_min_duty_cnt;
float motor_duty_scale = 1.0f;
uint32_t motor_duty_range;

void motor_init(void)
{
    PWM_ConfigOutputChannel(PWM, 0, 400, MOTOR_MIN_PWM_DUTY);
    PWM_ConfigOutputChannel(PWM, 1, 400, MOTOR_MIN_PWM_DUTY);
    PWM_ConfigOutputChannel(PWM, 2, 400, MOTOR_MIN_PWM_DUTY);
    PWM_ConfigOutputChannel(PWM, 3, 400, MOTOR_MIN_PWM_DUTY);
#if VEHICLE_FRAME == QUAD
    PWM_EnableOutput(PWM, 0x0F);
    PWM_Start(PWM, 0x0F);
#elif VEHICLE_FRAME == HEXA
    PWM_ConfigOutputChannel(PWM, 4, 400, MOTOR_MIN_PWM_DUTY);
    PWM_ConfigOutputChannel(PWM, 5, 400, MOTOR_MIN_PWM_DUTY);
    PWM_EnableOutput(PWM, 0x3F);
    PWM_Start(PWM, 0x3F);
#endif
    motor_min_duty_cnt = PWM->CMPDAT0;
    motor_duty_range = MOTOR_MAX_PWM - MOTOR_MIN_PWM;
    motor_duty_scale = (motor_min_duty_cnt + 1) / MOTOR_MIN_PWM;
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
