#include "PN020Series.h"
#include "timer_delay.h"

#if TIMER_DELAY_MODE == TIMER0_COUNTER
#else
void delay_ms(uint32_t _ms)
{
    // reserve;
}

void delay_us(uint32_t _us)
{
    // reserve
}
#endif
