#include "PN020Series.h"
#include "scheduler.h"
#include "../User/common.h"

uint32_t micro_100_counter = 0;

void scheduler_init()
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, MAIN_ISR_FREQ);

    /* enable timer1 interrupt */
    TIMER_EnableInt(TIMER1);
    NVIC_SetPriority(TMR1_IRQn, 0);

    TIMER_Start(TIMER1);
}

void TMR1_IRQHandler(void)
{
    time_slice();

    fast_loop();

    // clear timer interrupt flag
    TIMER_ClearIntFlag(TIMER0);
}

void fast_loop()
{
		
    /* fbm320 read running in 100us */
    fbm320_timer_procedure();

}

void scheduler_run()
{
    if (slice_flag.loop_1Hz) {

    }
    if (slice_flag.loop_5Hz) {

    }
    if (slice_flag.loop_10Hz) {

    }
}


void time_slice()
{
    static uint16_t count_1Hz = 0, count_5Hz = 0, count_10Hz = 0, count_20Hz = 0,\
            count_50Hz = 0, count_100Hz = 0, count_200Hz = 0, count_400Hz = 0, count_fast_loop = 0;

    count_1Hz++;
    count_5Hz++;
    count_10Hz++;
    count_20Hz++;
    count_50Hz++;
    count_100Hz++;
    count_200Hz++;
    count_400Hz++;
    count_fast_loop++;
    if (count_1Hz >= 10000) {
        slice_flag.loop_1Hz = true;
        count_1Hz = 0;
    }
    if (count_5Hz >= 2000) {
        slice_flag.loop_5Hz = true;
        count_5Hz = 0;
    }
    if (count_10Hz >= 1000) {
        slice_flag.loop_10Hz = true;
        count_10Hz = 0;
    }
    if (count_20Hz >= 500) {
        slice_flag.loop_20Hz = true;
        count_20Hz = 0;
    }
    if (count_50Hz >= 200) {
        slice_flag.loop_50Hz = true;
        count_50Hz = 0;
    }
    if (count_100Hz >= 100) {
        slice_flag.loop_100Hz = true;
        count_100Hz = 0;
    }
    if (count_200Hz >= 50) {
        slice_flag.loop_200Hz = true;
        count_200Hz = 0;
    }
    if (count_400Hz >= 25) {
        slice_flag.loop_400Hz = true;
        count_400Hz = 0;
    }
    if (count_fast_loop >= MAIN_LOOP_TIME) {
        slice_flag.fast_loop = true;
        count_fast_loop = 0;
    }
}
