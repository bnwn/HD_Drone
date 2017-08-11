#ifndef __COMMON_H
#define __COMMON_H

#include "stdio.h"
#include "../Driver/bsp/timer_delay.h"
#include "../Driver/bsp/uart_console.h"
#include "../Driver/bmi160.h"
#include "../Driver/fbm320.h"
#include "../Driver/motor.h"
#include "../Scheduler/scheduler.h"

void system_init();

void peripheral_init();

#endif
