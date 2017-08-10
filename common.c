#include "PN020Series.h"
#include "common.h"

void system_init()
{
   /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();
    CLK_InitHXTPLL();
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
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HCLK);
    NVIC_SetPriority(TMR0_IRQn, 0);

    /* Set P5.7 and P5.6 for I2C SDA and SCL */
    CLK_EnableModuleClock(I2C0_MODULE);
    SYS->P3_MFP |=SYS_MFP_P35_I2C0_SCL|SYS_MFP_P34_I2C0_SDA;
    P3->DINOFF &= ~ ( GP_DINOFF_DINOFF4_Msk |GP_DINOFF_DINOFF5_Msk);

    SYS_LockReg();
}

void driver_init()
{
    /* I2C Bus device init */
    I2C_Init(I2C0, 100000);
    if (!bmi160_init()) {

    }
    if (!fbm320_init()) {

    }

    /* servo output init */
    motor_init();

    /* uart device setting */
    uart_console_init(UART0, CONSOLE_BAUDRATE);
}
