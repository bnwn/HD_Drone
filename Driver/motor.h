#ifndef __MOTOR_H
#define __MOTOR_H

#include "pwm.h"

#define BLDC 0x01
#define DC 0x02
#define MOTOR_TYPE DC // BLDC
#define VEHICLE_FRAME QUAD // QUAD HEXA OTCA

#if VEHICLE_FRAME == QUAD
#define MOTOR_MAX_NUM 4
#elif VEHICLE_FRAME == HEXA
#define MOTOR_MAX_MUM 6
#else
#define MOTOR_MAX_NUM 8
#endif

#if MOTOR_TYPE == BLDC
#define MOTOR_PWM_FREQ 400 // uint(Hz)
#define MOTOR_MAX_PWM 2000
#define MOTOR_MIN_PWM 1000
#define MOTOR_MIN_PWM_DUTY (MOTOR_PWM_FREQ * MOTOR_MIN_PWM / 10000)
#define MOTOR_MAX_PWM_DUTY (MOTOR_PWM_FREQ * MOTOR_MAX_PWM / 10000)
#elif MOTOR_TYPE == DC
#define MOTOR_PWM_FREQ 20000 // uint(20KHz)
#define MOTOR_MIN_PWM 0
#define MOTOR_MIN_PWM_DUTY 0
#endif

#define MOTOR_DISARMED PWM_Stop(PWM, 0xff);
#if VEHICLE_FRAME == QUAD
#define MOTOR_ARMED PWM_Start(PWM, 0x0f);
#elif VEHICLE_FRAME == HEXA
#define MOTOR_ARMED PWM_Start(PWM, 0x3f);
#endif

/* function prototype */
/**
 * @brief motor init
 */
void motor_init(void);

/**
 * @brief update motor pwm output
 * @param _duty
 */
void motor_update(int32_t *_duty);


extern int32_t motor_duty[MOTOR_MAX_NUM];

#endif
