#include "PN020Series.h"
#include "scheduler.h"
#include "../User/common.h"

uint32_t micro_ticker = 0;
Slice_Flag slice_flag;

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
    slice_flag.main_loop = false;

    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, MAIN_ISR_FREQ);

    /* enable timer1 interrupt */
    TIMER_EnableInt(TIMER1);
		NVIC_EnableIRQ(TMR1_IRQn);
    NVIC_SetPriority(TMR1_IRQn, 0);

    TIMER_Stop(TIMER1);
}

void TMR1_IRQHandler(void)
{
		micro_ticker++;
		if (micro_ticker >= 0xFFFFFFF8) micro_ticker = 0;
	
    time_slice();

    // clear timer interrupt flag
    TIMER_ClearIntFlag(TIMER1);
}

void scheduler_run(void)
{
	  static uint32_t test = 0;
		
		//test = micro_ticker;
		/* main loop */
		fast_loop();
		//test = micro_ticker - test;
		//printf("using time:%d\n", test); 
	
		/* low priority task call in 200Hz, 100Hz, 50Hz, 20Hz, 10Hz, 5Hz, 1Hz */
		low_priority_loop();
}

void fast_loop(void)
{ 
		/* run out maximum cost 5.8ms */
		if (!slice_flag.main_loop) {
				return;
		}
		
		inertial_sensor_read();
		
		attitude_angle_rate_controller();
		
		motors_output();
		
		AHRS_Update();
		
		update_flight_mode();

		slice_flag.main_loop = false;
}

void low_priority_loop(void)
{
		sched_200Hzloop();
		sched_100Hzloop();
		sched_50Hzloop();
		sched_20Hzloop();
		sched_10Hzloop();
		sched_5Hzloop();
		sched_1Hzloop();
}

void sched_200Hzloop(void)
{
		if (!slice_flag.loop_200Hz) {
				return;
		}
		
		//printf("attitude:%d, %d, %d\n", (int16_t)(100*ahrs.Roll), (int16_t)(100*ahrs.Pitch), (int16_t)(100*ahrs.Yaw));
		slice_flag.loop_200Hz = false;
}

void sched_100Hzloop(void)
{
		if (!slice_flag.loop_100Hz) {
				return;
		}
		
		slice_flag.loop_100Hz = false;
}

void sched_50Hzloop(void)
{
		if (!slice_flag.loop_50Hz) {
				return;
		}
		
		fbm320_timer_procedure();  // update absolute altitude about 16.7Hz
		
		slice_flag.loop_50Hz = false;
}

void sched_20Hzloop(void)
{
		EulerAngle _ahrs;
		if (!slice_flag.loop_20Hz) {
				return;
		}
		
		set_flight_mode(Stabilize);
		//AHRS_Read_Attitude(&_ahrs);
		//printf("attitude:%d, %d, %d\n", (int16_t)(100*_ahrs.Roll), (int16_t)(100*_ahrs.Pitch), (int16_t)(100*_ahrs.Yaw));
		//printf("roll:%3.3f, pitch:%3.3f, yaw:%3.3f\n", (float)(_ahrs.Roll), (float)(_ahrs.Pitch), (float)(_ahrs.Yaw));
//				printf("%d, %d, %d, %d, %d, %d \n", (int16_t)inertial_sensor.accel.filter.x, (int16_t)inertial_sensor.accel.filter.y, (int16_t)inertial_sensor.accel.filter.z, \
//																	(int16_t)inertial_sensor.gyro.filter.x, (int16_t)inertial_sensor.gyro.filter.y, (int16_t)inertial_sensor.gyro.filter.z);
		//printf("attitude:%d, %d, %d\n", (int16_t)(100*ahrs.Roll), (int16_t)(100*ahrs.Pitch), (int16_t)(100*ahrs.Yaw));
		
		slice_flag.loop_20Hz = false;
}

void sched_10Hzloop(void)
{
		if (!slice_flag.loop_10Hz) {
				return;
		}
		
//		printf("%d, %d, %d, %d, %d, %d \n", (int16_t)inertial_sensor.accel.filter.x, (int16_t)inertial_sensor.accel.filter.y, (int16_t)inertial_sensor.accel.filter.z, \
//																	(int16_t)inertial_sensor.gyro.filter.x, (int16_t)inertial_sensor.gyro.filter.y, (int16_t)inertial_sensor.gyro.filter.z);
//		printf("altitude: %d cm\n", fbm320_packet.Altitude);
		slice_flag.loop_10Hz = false;
}

void sched_5Hzloop(void)
{
		if (!slice_flag.loop_200Hz) {
				return;
		}
		
		slice_flag.loop_5Hz = false;
}

void sched_1Hzloop(void)
{
		if (!slice_flag.loop_1Hz) {
				return;
		}

//	  PWM_ConfigOutputChannel(PWM, 0, MOTOR_PWM_FREQ, 10);
//    PWM_ConfigOutputChannel(PWM, 1, MOTOR_PWM_FREQ, 10);
//    PWM_ConfigOutputChannel(PWM, 2, MOTOR_PWM_FREQ, 10);
//    PWM_ConfigOutputChannel(PWM, 3, MOTOR_PWM_FREQ, 10);
//		PWM_ConfigOutputChannel(PWM, 7, MOTOR_PWM_FREQ, 10);
//    PWM_EnableOutput(PWM, 0xFF);
//    PWM_Start(PWM, 0xFF);
		slice_flag.loop_1Hz = false;
}


void time_slice(void)
{
    static uint16_t count_1Hz = 397, count_5Hz = 75, count_10Hz = 33, count_20Hz = 11,\
            count_50Hz = 7, count_100Hz = 2, count_200Hz = 1, count_main_loop = MAIN_LOOP_TIME;

//    if (!slice_flag.loop_1Hz) count_1Hz++;
		count_1Hz++;
    count_5Hz++;
    count_10Hz++;
    count_20Hz++;
		count_50Hz++;
    count_100Hz++;
    count_200Hz++;
    count_main_loop++;

    if (count_1Hz >= 400) {
        slice_flag.loop_1Hz = true;
        count_1Hz = 0;
    }
    if (count_5Hz >= 80) {
        slice_flag.loop_5Hz = true;
        count_5Hz = 0;
    }
    if (count_10Hz >= 40) {
        slice_flag.loop_10Hz = true;
        count_10Hz = 0;
    }
    if (count_20Hz >= 20) {
        slice_flag.loop_20Hz = true;
        count_20Hz = 0;
    }
    if (count_50Hz >= 8) {
        slice_flag.loop_50Hz = true;
        count_50Hz = 0;
    }
    if (count_100Hz >= 4) {
        slice_flag.loop_100Hz = true;
        count_100Hz = 0;
    }
    if (count_200Hz >= 2) {
        slice_flag.loop_200Hz = true;
        count_200Hz = 0;
    }
    if (count_main_loop >= MAIN_LOOP_TIME) {
        slice_flag.main_loop = true;
        count_main_loop = 0;
    }
}
