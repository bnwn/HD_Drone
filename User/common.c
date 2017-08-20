#include "PN020Series.h"
#include "common.h"

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

    SYS_LockReg();
}

void peripheral_init(void)
{
    /* uart device setting */
    uart_console_init(UART0, CONSOLE_BAUDRATE);
		printf("console init success!\n");
	
    /* I2C Bus device init */
    I2C_Init(I2C0, I2C_CLOCK_FREQ);
    if (!fbm320_init()) {

    }
		printf("fbm320 init success!\n");
    if (!bmi160_init()) {
				
    }
		printf("bmi160 init success!\n");

    /* servo output init */
    motor_init();

    /* task scheduler timer init */
    scheduler_init();
		
		param_load();
}

void param_load(void)
{
		set_pid_param(&ctrl_loop.angle.pitch, CONTROL_ANGLE_LOOP_PITCH_KP, CONTROL_ANGLE_LOOP_PITCH_KI, \
																					CONTROL_ANGLE_LOOP_PITCH_KD, OONTROL_ANGLE_LOOP_PITCH_INTEGRATOR_MAX, 0, 0.01f);
	
		set_pid_param(&ctrl_loop.angle.roll, CONTROL_ANGLE_LOOP_ROLL_KP, CONTROL_ANGLE_LOOP_ROLL_KI, \
																					CONTROL_ANGLE_LOOP_ROLL_KD, OONTROL_ANGLE_LOOP_ROLL_INTEGRATOR_MAX, 0, 0.01f);
	
		set_pid_param(&ctrl_loop.angle.yaw, CONTROL_ANGLE_LOOP_YAW_KP, CONTROL_ANGLE_LOOP_YAW_KI, \
																					CONTROL_ANGLE_LOOP_YAW_KD, OONTROL_ANGLE_LOOP_YAW_INTEGRATOR_MAX, 0, 0.01f);
	
		set_pid_param(&ctrl_loop.rate.pitch, CONTROL_RATE_LOOP_PITCH_KP, CONTROL_RATE_LOOP_PITCH_KI, \
																					CONTROL_RATE_LOOP_PITCH_KD, OONTROL_RATE_LOOP_PITCH_INTEGRATOR_MAX, 20, 0.01f);
	
		set_pid_param(&ctrl_loop.rate.roll, CONTROL_RATE_LOOP_ROLL_KP, CONTROL_RATE_LOOP_ROLL_KI, \
																					CONTROL_RATE_LOOP_ROLL_KD, OONTROL_RATE_LOOP_ROLL_INTEGRATOR_MAX, 20, 0.01f);
	
		set_pid_param(&ctrl_loop.rate.yaw, CONTROL_RATE_LOOP_YAW_KP, CONTROL_RATE_LOOP_YAW_KI, \
																					CONTROL_RATE_LOOP_YAW_KD, OONTROL_RATE_LOOP_YAW_INTEGRATOR_MAX, 0.5, 0.01f);
}
