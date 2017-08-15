#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include "PN020Series.h"
#include "timer.h"

#define MAIN_ISR_FREQ 10000 // uint(Hz)
#define MAIN_LOOP_FREQ 400 // uint(Hz)
#define MAIN_LOOP_TIME 25 // uint(0.1ms) (MAIN_ISR_FREQ / MAIN_LOOP_FREQ)

#define SCHEDULER_RUN  TIMER_Start(TIMER1)

void scheduler_init(void);
void fast_loop(void);
void scheduler_run(void);
void time_slice(void);

typedef struct {
    bool loop_1Hz;
    bool loop_5Hz;
    bool loop_10Hz;
    bool loop_20Hz;
    bool loop_50Hz;
    bool loop_100Hz;
    bool loop_200Hz;
    bool loop_400Hz;
    bool fast_loop;
} Slice_Flag;

extern Slice_Flag slice_flag;


#endif
