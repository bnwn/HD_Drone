#ifndef __TIMER_DELAY_H
#define __TIMER_DELAY_H

#include "timer.h"

#define TIMER_DELAY_MODE  TIMER_COUNTER

#if TIMER_DELAY_MODE == TIMER_COUNTER
#define delay_ms(_ms) TIMER_Delay(TIMER0, _ms * 1000)
#define delay_us(_us) TIMER_Delay(TIMER0, _us)
#else
void delay_ms(uint32_t _ms);
void delay_us(uint32_t _us);
#endif

#endif
