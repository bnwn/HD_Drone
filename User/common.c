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

_Status_t fc_status;
static uint32_t milli_ticker = 0;

void system_init(void)
{
   /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();
    CLK_InitHIRC();
    //CLK_InitHXT();
    //CLK_InitHXTPLL();
    SystemCoreClockUpdate();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP |= SYS_MFP_P00_UART0_TXD|SYS_MFP_P01_UART0_RXD;
    GPIO_ENABLE_DIGITAL_PATH(P0,(1<<1));
    CLK_EnableModuleClock(UART0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init Timer Clock                                                                                        */
    /*---------------------------------------------------------------------------------------------------------*/
    /* timer delay configure */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HCLK);
//    NVIC_SetPriority(TMR0_IRQn, 3);
    /* timer perioic configure */
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HCLK);
		
		/* Set P2 multi-function pin for PWM Channel 0  */
    CLK_EnableModuleClock(PWMCH01_MODULE);
    CLK_EnableModuleClock(PWMCH23_MODULE);
    CLK_EnableModuleClock(PWMCH67_MODULE);
    SYS->P2_MFP = SYS_MFP_P22_PWM0_CH0 | SYS_MFP_P23_PWM0_CH1 | SYS_MFP_P24_PWM0_CH2 | SYS_MFP_P25_PWM0_CH3;
    SYS->P5_MFP = SYS_MFP_P57_PWM0_CH7;

    /* Set P5.7 and P5.6 for I2C SDA and SCL */
    CLK_EnableModuleClock(I2C0_MODULE);
    SYS->P3_MFP |=SYS_MFP_P35_I2C0_SCL|SYS_MFP_P34_I2C0_SDA;
    P3->DINOFF &= ~ ( GP_DINOFF_DINOFF4_Msk |GP_DINOFF_DINOFF5_Msk);
		
    /* spi gpio setting */
    SYS->P0_MFP |= SYS_MFP_P03_SPI1_SS;  //SPI1SS
    SYS->P1_MFP |= SYS_MFP_P11_SPI1_CLK | SYS_MFP_P16_SPI1_MOSI | SYS_MFP_P17_SPI1_MISO ;//SPI1 MOSI1 MISO1   
    GPIO_ENABLE_DIGITAL_PATH(P0,(1<<3));
    GPIO_ENABLE_DIGITAL_PATH(P1,(1<<1));
    GPIO_ENABLE_DIGITAL_PATH(P1,(1<<6));
    GPIO_ENABLE_DIGITAL_PATH(P1,(1<<7));
    CLK_EnableModuleClock(SPI1_MODULE);  

    SYS_LockReg();
}

void peripheral_init(void)
{
    /* uart device setting */    
#ifdef __DEVELOP__
    uart_console_init(UART0, CONSOLE_BAUDRATE);
    printf("console init success!\n");
#endif
    /* I2C Bus device init */
    I2C_Init(I2C0, I2C_CLOCK_FREQ);
	
    if (!fbm320_init()) {

    }
#ifdef __DEVELOP__
    printf("fbm320 init success!\n");
#endif
    if (!bmi160_init()) {
				
    }
#ifdef __DEVELOP__
    printf("bmi160 init success!\n");
#endif

    /* servo output init */
    motor_init();

    /* task scheduler timer init */
    scheduler_init();
		
    /* rf init */
    rc_channel_init();

    param_load();

#ifdef __DEVELOP__
    printf("HD_Drone init success!\n");

    printf("sensor collect offset...\n");
#endif
    gyro_offset();
    accel_offset();
	inertial_sensor_init();
	inertial_nav_init();
#ifdef __DEVELOP__
    printf("collect complete\n");
#endif
}

void param_load(void)
{
	set_pid_param(&ctrl_loop.angle.pitch, CONTROL_ANGLE_LOOP_PITCH_KP, CONTROL_ANGLE_LOOP_PITCH_KI, \
																				CONTROL_ANGLE_LOOP_PITCH_KD, OONTROL_ANGLE_LOOP_PITCH_INTEGRATOR_MAX, 0, LOOP_DT);

	set_pid_param(&ctrl_loop.angle.roll, CONTROL_ANGLE_LOOP_ROLL_KP, CONTROL_ANGLE_LOOP_ROLL_KI, \
																				CONTROL_ANGLE_LOOP_ROLL_KD, OONTROL_ANGLE_LOOP_ROLL_INTEGRATOR_MAX, 0, LOOP_DT);

	set_pid_param(&ctrl_loop.angle.yaw, CONTROL_ANGLE_LOOP_YAW_KP, CONTROL_ANGLE_LOOP_YAW_KI, \
																				CONTROL_ANGLE_LOOP_YAW_KD, OONTROL_ANGLE_LOOP_YAW_INTEGRATOR_MAX, 0, LOOP_DT);

	set_pid_param(&ctrl_loop.rate.pitch, CONTROL_RATE_LOOP_PITCH_KP, CONTROL_RATE_LOOP_PITCH_KI, \
																				CONTROL_RATE_LOOP_PITCH_KD, OONTROL_RATE_LOOP_PITCH_INTEGRATOR_MAX, 20, LOOP_DT);

	set_pid_param(&ctrl_loop.rate.roll, CONTROL_RATE_LOOP_ROLL_KP, CONTROL_RATE_LOOP_ROLL_KI, \
																				CONTROL_RATE_LOOP_ROLL_KD, OONTROL_RATE_LOOP_ROLL_INTEGRATOR_MAX, 20, LOOP_DT);

	set_pid_param(&ctrl_loop.rate.yaw, CONTROL_RATE_LOOP_YAW_KP, CONTROL_RATE_LOOP_YAW_KI, \
																				CONTROL_RATE_LOOP_YAW_KD, OONTROL_RATE_LOOP_YAW_INTEGRATOR_MAX, 0.5, LOOP_DT);

	set_pid_param(&ctrl_loop.acro_sensibility.yaw, CONTROL_ACRO_SENSITITY_LOOP_YAW_KP, 0, \
																			0, 0, 0, LOOP_DT);
	
	set_pid_param(&ctrl_loop.pos.z, ALT_HOLD_P, 0, \
																			0, 0, 0, LOOP_DT);
																			
	set_pid_param(&ctrl_loop.pos.z, VEL_Z_P, 0, \
																			0, 0, 0, LOOP_DT);
																			
	set_pid_param(&ctrl_loop.pos.z, ACCEL_Z_P, ACCEL_Z_I, \
																			ACCEL_Z_D, ACCEL_Z_FILT_HZ, ACCEL_Z_IMAX, LOOP_DT);																			
}

void fc_status_reset(void)
{
    fc_status.armed = DISARMED;
    fc_status.simple_mode = Normal;
	fc_status.land_complete = true;
	fc_status.code_matched = false;
    fc_status.radio_lost = true;
    fc_status.altitude_updated = false;
	fc_status.home_abs_alt_updated = false;
    fc_status.accel_updated = false;
    fc_status.baro_collect_ok = false;
    fc_status.inav_z_estimate_ok = false;
	fc_status.printf_flag = 6;
	fc_status.motor_control_Hz = 2000;
}

void set_land_complete(bool _b)
{
    if (fc_status.land_complete != _b)
    {
        fc_status.land_complete = _b;
    }

    if (fc_status.land_complete && ((control_mode == Land) || (control_mode == Auto))) {
        fc_status.armed = DISARMED;
    }
}

void set_radio_lost(bool _b)
{
    if (fc_status.radio_lost != _b)
    {
        fc_status.radio_lost = _b;
    }

    if (fc_status.radio_lost) {
        fc_status.armed = DISARMED;
        //set_flight_mode(Land);
    }
}

uint32_t systick_config(uint32_t ticks)
{
  if (ticks > SysTick_LOAD_RELOAD_Msk)  return (1);            /* Reload value impossible */

  SysTick->LOAD  = (ticks & SysTick_LOAD_RELOAD_Msk) - 1;      /* set reload register */
  NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority for Systick Interrupt */
  SysTick->VAL   = 0;                                          /* Load the SysTick Counter Value */
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_TICKINT_Msk   |
                   SysTick_CTRL_ENABLE_Msk;                    /* Enable SysTick IRQ and SysTick Timer */
  return (0);                                                  /* Function successful */
}

void SysTick_Handler(void)
{
	if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
		milli_ticker++;	
}

uint32_t sys_micro(void)
{
	return (uint32_t)(milli_ticker * 1000 + SysTick->VAL / CyclesPerUs);
}

uint32_t sys_milli(void)
{
	return milli_ticker;
}

