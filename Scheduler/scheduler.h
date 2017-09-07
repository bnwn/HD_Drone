#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include "PN020Series.h"
#include "timer.h"

#define MAIN_ISR_FREQ 400 // uint(Hz)
#define MAIN_LOOP_FREQ 100 // uint(Hz)
#define MAIN_LOOP_TIME 4 // (MAIN_ISR_FREQ / MAIN_LOOP_FREQ)

#define SCHEDULER_RUN  TIMER_Start(TIMER1)
#define SCHEDULER_STOP TIMER_Stop(TIMER1)

void scheduler_init(void);
void scheduler_run(void);
/* scheduler */
void fast_loop(void);
void low_priority_loop(void);
void sched_200Hzloop(void);
void sched_80Hzloop(void);
void sched_50Hzloop(void);
void sched_20Hzloop(void);
void sched_10Hzloop(void);
void sched_5Hzloop(void);
void sched_1Hzloop(void);

/* timer slice */
void time_slice(void);

typedef struct {
    bool loop_1Hz;
    bool loop_5Hz;
    bool loop_10Hz;
    bool loop_20Hz;
    bool loop_50Hz;
    bool loop_80Hz;
    bool loop_200Hz;
    bool loop_400Hz;
    bool main_loop;
} Slice_Flag;

extern Slice_Flag slice_flag;


#endif
