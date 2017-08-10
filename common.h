#ifndef __COMMON_H
#define __COMMON_H

#include "stdio.h"
#include "Driver/bsp/timer_delay.h"
#include "Driver/bsp/uart_console.h"
#include "Driver/bsp/scheduler.h"
#include "Driver/bmi160.h"
#include "Driver/fbm320.h"
#include "Driver/motor.h"

void system_init();

void driver_init();

#endif
