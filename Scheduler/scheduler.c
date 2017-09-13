#include "PN020Series.h"
#include "common.h"
#include "stdio.h"
#include "../AHRS/ahrs.h"
#include "../AHRS/inertial_sensor.h"
#include "../AHRS/inertial_nav.h"
#include "../Algorithm/Algorithm_filter/Algorithm_filter.h"
#include "../Algorithm/Algorithm_math/Algorithm_math.h"
#include "../Algorithm/Algorithm_pid/Algorithm_pid.h"
#include "../Algorithm/Algorithm_quaternion/Algorithm_quaternion.h"
#include "../Control/attitude_control.h"
#include "../Control/flight_mode_control.h"
#include "../Control/motor_control.h"
#include "../Control/position_control.h"
#include "../Driver/bsp/timer_delay.h"
#include "../Driver/bsp/uart_console.h"
#include "../Driver/bmi160.h"
#include "../Driver/fbm320.h"
#include "../Driver/motor.h"
#include "../Scheduler/scheduler.h"
#include "../RC/rc_channel.h"
#include "adc_det.h"

static uint32_t main_isr_ticker = 0;
Slice_Flag slice_flag;

#ifdef __DEVELOP__
extern _Target_Attitude attitude_target_ang_vel;
extern float accel_NED[3];
extern float acc_bias[3];	// body frame ,
#endif

void scheduler_init(void)
{
    slice_flag.loop_1Hz = false;
    slice_flag.loop_5Hz = false;
    slice_flag.loop_10Hz = false;
    slice_flag.loop_20Hz = false;
    slice_flag.loop_50Hz = false;
    slice_flag.loop_80Hz = false;
    slice_flag.loop_200Hz = false;
    slice_flag.loop_400Hz = false;
    slice_flag.main_loop = false;

	if (systick_config(CyclesPerUs*1000) == 0)  printf ("systick init success\n");
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, MAIN_ISR_FREQ);

    /* enable timer1 interrupt */
    TIMER_EnableInt(TIMER1);
	NVIC_EnableIRQ(TMR1_IRQn);
    NVIC_SetPriority(TMR1_IRQn, 2);

    TIMER_Stop(TIMER1);
}   

void TMR1_IRQHandler(void)
{
	main_isr_ticker++;
	if (main_isr_ticker >= 0xFFFFFFF8) main_isr_ticker = 0;
	
    time_slice();

    // clear timer interrupt flag
    TIMER_ClearIntFlag(TIMER1);
}

void scheduler_run(void)
{
	/* main loop */
	fast_loop();

	/* low priority task call in 200Hz, 100Hz, 50Hz, 20Hz, 10Hz, 5Hz, 1Hz */
	low_priority_loop();
}

void fast_loop(void)
{ 	
	static uint32_t loop_tick = 0;	
	loop_tick = sys_micro();
	/* run out maximum cost 5.8ms */
	if (!slice_flag.main_loop) {
		return;
	}
	
	inertial_sensor_read();
	
	attitude_angle_rate_controller();
	
	motors_output();
	
	AHRS_Update();
	
	inertial_nav_update();
	
	update_flight_mode();
	
	slice_flag.main_loop = false;
	
#ifdef __DEVELOP__
	loop_tick = sys_micro() - loop_tick;
	//printf("using time:%d\n", sys_micro()-loop_tick);
	//using time:3798
	//using time:1479
	//261,213,208,-200,-200,-200,
	//using time:4081
	//using time:2718
	//262,213,208,-200,-200,-200,
	//using time:5827
	//using time:2839
	//262,213,207,-200,-200,-200,
	//using time:4340
	//using time:4334
	//262,212,208,-200,-200,-200,
	//using time:3840
	//using time:2972
#endif
}

void low_priority_loop(void)
{
//		sched_200Hzloop();
	sched_80Hzloop();
	sched_50Hzloop();
	sched_20Hzloop();
	sched_10Hzloop();
//		sched_5Hzloop();
	sched_1Hzloop();
}

void sched_200Hzloop(void)
{
	if (!slice_flag.loop_200Hz) {
		return;
	}

	slice_flag.loop_200Hz = false;
}

void sched_80Hzloop(void)
{
	if (!slice_flag.loop_80Hz) {
		return;
	}
	
	fbm320_timer_procedure();  // update absolute altitude about 26.7Hz
	
	slice_flag.loop_80Hz = false;
}

void sched_50Hzloop(void)
{
	if (!slice_flag.loop_50Hz) {
		return;
	}
			
	rc_channel_read();

	slice_flag.loop_50Hz = false;
}

void sched_20Hzloop(void)
{
    _Vector_Float current_ang_vel = get_inertial_vel();
	EulerAngle _ahrs;
	float bat_vol, pwm_det[4];
	if (!slice_flag.loop_20Hz) {
		return;
	}	
	
	update_home_pos();
	switch_handle();
	adc_convert();
#ifdef __DEVELOP__	
	bat_vol = get_battery_vol();
	motor_pwm_detect(pwm_det);
	
//	if ((fc_status.printf_flag == 0) || (fc_status.printf_flag == 255))
		printf("%d,%d,%d,%d,%d,%d,\n", (int16_t)(ahrs.Roll*2+200), (int16_t)(ahrs.Pitch*2+200), (int16_t)(ahrs.Yaw*2+200), (int16_t)(target_attitude.roll*2-200), (int16_t)(target_attitude.pitch*2-200), (int16_t)(target_attitude.yaw*2-200));
//	else if (fc_status.printf_flag == 1)
//		printf("%d,%d,%d,%d,%d,%d,\n", (int16_t)(current_ang_vel.x*10), (int16_t)(current_ang_vel.y*10), (int16_t)(current_ang_vel.z*10), \
//										(int16_t)(attitude_target_ang_vel.roll*10), (int16_t)(attitude_target_ang_vel.pitch*10), (int16_t)(attitude_target_ang_vel.yaw*10));
//	else if (fc_status.printf_flag == 2)
//		printf("%d,%d,%d,%d,%d,%d,\n", (int16_t)((inertial_sensor.accel.average.x-inertial_sensor.accel.quiet.x)*_accel_scale*30+200), (int16_t)((inertial_sensor.accel.average.y-inertial_sensor.accel.quiet.y)*_accel_scale*30+200), \
//										(int16_t)((inertial_sensor.accel.average.z-inertial_sensor.accel.quiet.z)*_accel_scale*10), \
//										(int16_t)(inertial_sensor.accel.filter.x*30-200), (int16_t)(inertial_sensor.accel.filter.y*30-200), (int16_t)(inertial_sensor.accel.filter.z*10));
//	else if (fc_status.printf_flag == 3)
//		printf("%d,%d,%d,%d,%d,%d,\n", (int16_t)(inertial_sensor.gyro.average.x*_gyro_scale+200), (int16_t)(inertial_sensor.gyro.average.y*_gyro_scale+200), (int16_t)(inertial_sensor.gyro.average.z*_gyro_scale+200), \
//										(int16_t)(inertial_sensor.gyro.filter.x-200), (int16_t)(inertial_sensor.gyro.filter.y-200), (int16_t)(inertial_sensor.gyro.filter.z-200));
//	else if (fc_status.printf_flag == 4)
//		printf ("%d, %d, %d, %d,\n", (int16_t)(motor_duty[MOTOR1_INDEX]*500-300), (int16_t)(motor_duty[MOTOR2_INDEX]*500-300), (int16_t)(motor_duty[MOTOR3_INDEX]*500-300), \
//																														(int16_t)(motor_duty[MOTOR4_INDEX]*500-300));
//	else if (fc_status.printf_flag == 5)
//		printf ("%d, %d, %d, %d,\n", (int16_t)(pwm_det[0]*100), (int16_t)(pwm_det[1]*100), (int16_t)(pwm_det[2]*100), (int16_t)(pwm_det[3]*100));
//	else if (fc_status.printf_flag == 6)
//		printf ("%d, %d, %d, %d,\n", (int16_t)(rc_channels[0].rc_in/5), (int16_t)(rc_channels[1].rc_in/5), (int16_t)(rc_channels[2].rc_in/5), (int16_t)(rc_channels[3].rc_in/5));
//	else if (fc_status.printf_flag == 7)
//		printf("%d, %d, %d, %d, %d, %d, %d, %d, %d\n", (int16_t)((fbm320_packet.altitude-home_absolute_pos.z)*100), (int16_t)(get_inav_alt()), (int16_t)(get_inav_velocity().z), \
//											(int16_t)(get_inav_accel().z), (int16_t)(accel_NED[2]*100), (int16_t)((inertial_sensor.accel.filter.z-GRAVITY_MSS)*100), (int16_t)(home_absolute_pos.z*100), \
//												(int16_t)(bat_vol * 100), (int16_t)(acc_bias[2]*100));
//	else if (fc_status.printf_flag == 8)
//		printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", (int16_t)(pos_target.z), (int16_t)(pos_target.vz), (int16_t)(pos_target.az), \
//											(int16_t)(get_inav_alt()), (int16_t)(get_inav_velocity().z), (int16_t)(get_inav_accel().z), \
//											(int16_t)(pos_error.z), (int16_t)(pos_error.vz), (int16_t)(pos_error.az), \
//											(int16_t)(bat_vol * 100), (int16_t)(motor.thrust.throttle*1000));
#endif	
#ifdef __DEVELOP__
	//AHRS_Read_Attitude(&_ahrs);
	//printf("attitude:%d, %d, %d\n", (int16_t)(100*_ahrs.Roll), (int16_t)(100*_ahrs.Pitch), (int16_t)(100*_ahrs.Yaw));
	//printf("roll:%3.3f, pitch:%3.3f, yaw:%3.3f\n", (float)(_ahrs.Roll), (float)(_ahrs.Pitch), (float)(_ahrs.Yaw));
//	printf("%d, %d, %d \n", (int16_t)inertial_sensor.accel.average.x, (int16_t)inertial_sensor.accel.average.y, (int16_t)inertial_sensor.accel.average.z);
//	printf("%d, %d, %d \n", (int16_t)inertial_sensor.accel.relative.x, (int16_t)inertial_sensor.accel.relative.y, (int16_t)inertial_sensor.accel.relative.z);
//		printf("%d, %d, %d \n", (int16_t)inertial_sensor.gyro.filter.x, (int16_t)inertial_sensor.gyro.filter.y, (int16_t)inertial_sensor.gyro.filter.z);
//		printf("attitude:%d, %d, %d\n", (int16_t)(100*ahrs.Roll), (int16_t)(100*ahrs.Pitch), (int16_t)(100*ahrs.Yaw));
//		printf ("%d, %d, %d, %d\n", (int16_t)(motor_duty[MOTOR1_INDEX]*1000), (int16_t)(motor_duty[MOTOR2_INDEX]*1000), (int16_t)(motor_duty[MOTOR3_INDEX]*1000), \
																													(int16_t)(motor_duty[MOTOR4_INDEX]*1000));
	
//	printf("ch1:%d, ch2:%d, ch3:%d, ch4:%d, ch5:%d, ch6:%d, ch7:%d, ch8:%d, ch9:%d, ch10:%d, ch11:%d, ch12:%d\n", rc_channels[0].rc_in, rc_channels[1].rc_in, rc_channels[2].rc_in, \
//																																		rc_channels[3].rc_in, rc_switchs[0].rc_in, rc_switchs[1].rc_in, rc_switchs[2].rc_in, rc_switchs[3].rc_in, \
//																																		rc_switchs[4].rc_in, rc_switchs[5].rc_in, rc_switchs[6].rc_in, rc_switchs[7].rc_in);
//        printf("altitude: %d m\n", (int16)(100*fbm320_packet.altitude));
#endif
	slice_flag.loop_20Hz = false;
}

void sched_10Hzloop(void)
{
	if (!slice_flag.loop_10Hz) {
		return;
	}
	
	check_motor_armed();
	
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
	static float trace_throttle = 0.0f;
	if (!slice_flag.loop_1Hz) {
		return;
	}

//	if (fc_status.printf_flag == 255) {
//		fc_status.armed = MOTOR_TEST;
//		trace_throttle += 0.1;
//		if (trace_throttle > 1.0f)
//		{
//			trace_throttle = 0.0f;
//			fc_status.motor_control_Hz += 1000;
//			#ifdef __DEVELOP__
//			printf("current motor PWM freq:%d\n", fc_status.motor_control_Hz);
//			#endif
//			PWM_ConfigOutputChannel(PWM, 0, fc_status.motor_control_Hz, MOTOR_MIN_PWM_DUTY);	
//			PWM_ConfigOutputChannel(PWM, 1, fc_status.motor_control_Hz, MOTOR_MIN_PWM_DUTY);
//			PWM_ConfigOutputChannel(PWM, 2, fc_status.motor_control_Hz, MOTOR_MIN_PWM_DUTY);
//			PWM_ConfigOutputChannel(PWM, 3, fc_status.motor_control_Hz, MOTOR_MIN_PWM_DUTY);
//			
//			PWM_EnableOutput(PWM, 0x0F);
//			PWM_Start(PWM, 0x0F);
//			
//			motor_duty_range = PWM->PERIOD0;
//		}
//		motor_duty[MOTOR1_INDEX] = trace_throttle;
//		motor_duty[MOTOR2_INDEX] = trace_throttle;
//		motor_duty[MOTOR3_INDEX] = trace_throttle;
//		motor_duty[MOTOR4_INDEX] = trace_throttle;
//		
//		motor_update(motor_duty);
//	} else {
//		trace_throttle = 0.0f;
//	}
	slice_flag.loop_1Hz = false;
}


void time_slice(void)
{
    static uint16_t count_1Hz = 397, count_5Hz = 75, count_10Hz = 33, count_20Hz = 11,\
            count_50Hz = 7, count_80Hz = 3, count_200Hz = 1, count_main_loop = MAIN_LOOP_TIME;

    count_1Hz++;
    count_5Hz++;
    count_10Hz++;
    count_20Hz++;
    count_50Hz++;
    count_80Hz++;
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
    if (count_80Hz >= 5) {
        slice_flag.loop_80Hz = true;
        count_80Hz = 0;
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