#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include "timer.h"

#define MAIN_ISR_FREQ 10000 // uint(Hz)
#define MAIN_LOOP_FREQ 400 // uint(Hz)
#define MAIN_LOOP_TIME 25 // uint(0.1ms) (MAIN_ISR_FREQ / MAIN_LOOP_FREQ)

void scheduler_init();
void fast_loop();
void scheduler_run();
void time_slice();

typedef struct {
    bool loop_1Hz = false;
    bool loop_5Hz = false;
    bool loop_10Hz = false;
    bool loop_20Hz = false;
    bool loop_50Hz = false;
    bool loop_100Hz = false;
    bool loop_200Hz = false;
    bool loop_400Hz = false;
    bool fast_loop = false;
} Slice_Flag;

extern Slice_Flag slice_flag;


#endif
