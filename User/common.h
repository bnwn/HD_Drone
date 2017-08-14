#ifndef __COMMON_H
#define __COMMON_H

#include "stdio.h"
#include "../AHRS/ahrs.h"
#include "../AHRS/inertial_sensor.h"
#include "../Algorithm/Algorithm_filter/Algorithm_filter.h"
#include "../Algorithm/Algorithm_math/Algorithm_math.h"
#include "../Algorithm/Algorithm_math/mymath.h"
#include "../Algorithm/Algorithm_pid/Algorithm_pid.h"
#include "../Algorithm/Algorithm_quaternion/Algorithm_quaternion.h"
#include "../Control/attitude_control.h"
#include "../Control/flight_mode_control.h"
#include "../Control/motor_control.h"
#include "../Control/position_control.h"
#include "../Driver/bsp/timer_delay.h"
#include "../Driver/bsp/uart_console.h"
#include "../Driver/bmi160.h"
#include "../Driver/fbm320.h"
#include "../Driver/motor.h"
#include "../Scheduler/scheduler.h"

void system_init(void);

void peripheral_init(void);

#endif
