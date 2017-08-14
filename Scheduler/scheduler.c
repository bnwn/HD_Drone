#include "PN020Series.h"
#include "scheduler.h"
#include "../User/common.h"

uint32_t micro_100_counter = 0;

void scheduler_init(void)
{
    slice_flag.loop_1Hz = false;
    slice_flag.loop_5Hz = false;
    slice_flag.loop_10Hz = false;
    slice_flag.loop_20Hz = false;
    slice_flag.loop_50Hz = false;
    slice_flag.loop_100Hz = false;
    slice_flag.loop_200Hz = false;
    slice_flag.loop_400Hz = false;
    slice_flag.fast_loop = false;

    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, MAIN_ISR_FREQ);

    /* enable timer1 interrupt */
    TIMER_EnableInt(TIMER1);
    NVIC_SetPriority(TMR1_IRQn, 0);

    TIMER_Start(TIMER1);
}

void TMR1_IRQHandler(void)
{
    time_slice();

    /* main loop */
    fast_loop();

    // clear timer interrupt flag
    TIMER_ClearIntFlag(TIMER0);
}

void fast_loop(void)
{
    /* fbm320 read running in 100us */
    fbm320_timer_procedure();

    if (slice_flag.fast_loop) {
        /* core sensor update */
        inertial_sensor_read();

        /* run attitude angle rate control in 400Hz, first call in loop ensure attitude stable */
        attitude_angle_rate_controller();

        motors_output();

        AHRS_Update();

        update_flight_mode();

        slice_flag.fast_loop = false;
    }
}

void scheduler_run(void)
{
    if (slice_flag.loop_1Hz) {

    }
    if (slice_flag.loop_5Hz) {

    }
    if (slice_flag.loop_10Hz) {

    }
    if (slice_flag.loop_50Hz) {
        set_flight_mode(Stabilize);

        slice_flag.loop_50Hz = false;
    }
}


void time_slice(void)
{
    static uint16_t count_1Hz = 9997, count_5Hz = 1995, count_10Hz = 993, count_20Hz = 491,\
            count_50Hz = 189, count_100Hz = 87, count_200Hz = 35, count_400Hz = 8, count_fast_loop = 49;

    if (!slice_flag.loop_1Hz) count_1Hz++;
    if (!slice_flag.loop_5Hz) count_5Hz++;
    if (!slice_flag.loop_10Hz) count_10Hz++;
    if (!slice_flag.loop_20Hz) count_20Hz++;
    if (!slice_flag.loop_50Hz) count_50Hz++;
    if (!slice_flag.loop_100Hz) count_100Hz++;
    if (!slice_flag.loop_200Hz) count_200Hz++;
    if (!slice_flag.loop_400Hz) count_400Hz++;
    if (!slice_flag.fast_loop) count_fast_loop++;

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
