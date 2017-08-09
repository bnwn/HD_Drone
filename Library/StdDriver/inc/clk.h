/**************************************************************************//**
 * @file     clk.h
 * @version  V1.00
 * $Revision: 3$
 * $Date: 16/02/29 14:50 $ 
 * @brief    PN020 series CLK driver header file
 *
 * @note
 * Copyright (C) 2016 Shanghai Panchip Microelectronics Co., Ltd.   All rights reserved.
 *****************************************************************************/ 
#ifndef __CLK_H__
#define __CLK_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup PN020_Device_Driver PN020 Device Driver
  @{
*/

/** @addtogroup PN020_CLK_Driver CLK Driver
  @{
*/



/** @addtogroup PN020_CLK_EXPORTED_CONSTANTS CLK Exported Constants
  @{
*/

#define FREQ_25MHZ         25000000
#define FREQ_50MHZ         50000000
#define FREQ_72MHZ         72000000
#define FREQ_100MHZ        100000000
#define FREQ_200MHZ        200000000
#define FREQ_250MHZ        250000000
#define FREQ_500MHZ        500000000


/*---------------------------------------------------------------------------------------------------------*/
/*  PWRCTL constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_PWRCTL_XTL16M             0x01UL /*!< Setting External Crystal Oscillator as 16MHz         */
#define CLK_PWRCTL_XTLEN_HXT          0x01UL /*!< Setting External Crystal Oscillator as 16MHz         */
#define CLK_PWRCTL_XTL32K             0x02UL /*!< Setting External Crystal Oscillator as 32KHz         */
#define CLK_PWRCTL_XTLEN_LXT          0x02UL /*!< Setting External Crystal Oscillator as 32KHz         */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLk STATUS constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define	CLK_STATUS_XTLSTB				0x01UL
#define	CLK_STATUS_IBGSTB				0x02UL
#define	CLK_STATUS_PLLSTB				0x04UL
#define	CLK_STATUS_LIRCSTB			0x08UL
#define	CLK_STATUS_HIRCSTB			0x10UL

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL0 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL0_HCLKSEL_HIRC          0x00UL /*!< Setting clock source as internal High speed RC clock */
#define CLK_CLKSEL0_HCLKSEL_PLL           0x01UL
#define CLK_CLKSEL0_HCLKSEL_LIRC          0x02UL
#define CLK_CLKSEL0_HCLKSEL_XTAL          0x03UL


/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL1 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL1_WDTSEL_HCLK_DIV2048  0x00000000UL /*!< Setting WDT clock source as HCLK/2048 */
#define CLK_CLKSEL1_WDTSEL_LIRC           0x00000001UL /*!< Setting WDT clock source as internal 10KHz RC clock */

#define CLK_CLKSEL1_TMR0SEL_HCLK         0x00000000UL /*!< Setting Timer 0 clock source as HCLK */
#define CLK_CLKSEL1_TMR0SEL_LIRC         0x00000100UL /*!< Setting Timer 0 clock source as internal 10KHz RC clock */
#define CLK_CLKSEL1_TMR0SEL_TM0          0x00000300UL /*!< Setting Timer 0 clock source as external trigger */

#define CLK_CLKSEL1_TMR1SEL_HCLK         0x00000000UL /*!< Setting Timer 1 clock source as HCLK */
#define CLK_CLKSEL1_TMR1SEL_LIRC         0x00001000UL /*!< Setting Timer 1 clock source as internal 10KHz RC clock */
#define CLK_CLKSEL1_TMR1SEL_TM1          0x00003000UL /*!< Setting Timer 1 clock source as external trigger */

#define CLK_CLKSEL1_TMR2SEL_HCLK         0x00000000UL /*!< Setting Timer 2 clock source as HCLK */
#define CLK_CLKSEL1_TMR2SEL_LIRC         0x00010000UL /*!< Setting Timer 2 clock source as internal 10KHz RC clock */
#define CLK_CLKSEL1_TMR2SEL_TM2          0x00030000UL /*!< Setting Timer 2 clock source as external trigger */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL2 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL2_CLKOSEL_HXT         	0x00000000UL /*!< Setting CLKODIV clock source as HXT, PLL or HIRC divide 8 */ 
#define CLK_CLKSEL2_CLKOSEL_LIRC        	0x00000004UL /*!< Setting CLKODIV clock source as LIRC */ 

#define CLK_CLKSEL2_WWDTSEL_HCLK_DIV2048   	0x00020000UL /*!< Setting WWDT clock source as HCLK/2048 */ 
#define CLK_CLKSEL2_WWDTSEL_LIRC           	0x00030000UL /*!< Setting WWDT clock source as internal RC clock */ 
 

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKDIV constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

#define CLK_CLKDIV_HCLK1  				(0x0UL)        /*!< hclk div = 1 */ 
#define CLK_CLKDIV_HCLK2  				(0x1UL)        /*!< hclk div = 2 */ 
#define CLK_CLKDIV_HCLK4  				(0x2UL)        /*!< hclk div = 4 */ 
#define CLK_CLKDIV_HCLK8  				(0x3UL)        /*!< hclk div = 8 */ 

/*---------------------------------------------------------------------------------------------------------*/
/*  MODULE constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/ 
#define MODULE_APBCLK(x)                   ((x >>31) & 0x1)    /*!< Calculate APBCLK offset on MODULE index */ 
#define MODULE_CLKSEL(x)                   ((x >>29) & 0x3)    /*!< Calculate CLKSEL offset on MODULE index */ 
#define MODULE_CLKSEL_Msk(x)               ((x >>25) & 0xf)    /*!< Calculate CLKSEL mask offset on MODULE index */ 
#define MODULE_CLKSEL_Pos(x)               ((x >>20) & 0x1f)   /*!< Calculate CLKSEL position offset on MODULE index */ 
#define MODULE_CLKDIV(x)                   ((x >>18) & 0x3)    /*!< Calculate APBCLK CLKDIV on MODULE index */ 
#define MODULE_CLKDIV_Msk(x)               ((x >>10) & 0xff)   /*!< Calculate CLKDIV mask offset on MODULE index */ 
#define MODULE_CLKDIV_Pos(x)               ((x >>5 ) & 0x1f)   /*!< Calculate CLKDIV position offset on MODULE index */ 
#define MODULE_IP_EN_Pos(x)                ((x >>0 ) & 0x1f)   /*!< Calculate APBCLK offset on MODULE index */ 
#define MODULE_NoMsk                       0x0                 /*!< Not mask on MODULE index */ 
#define NA                                 MODULE_NoMsk        /*!< Not Available */

#define MODULE_APBCLK_ENC(x)        (((x) & 0x01) << 31)   /*!< MODULE index, 0x0:AHBCLK, 0x1:APBCLK */
#define MODULE_CLKSEL_ENC(x)        (((x) & 0x03) << 29)   /*!< CLKSEL offset on MODULE index, 0x0:CLKSEL0, 0x1:CLKSEL1 0x3 CLKSEL2*/
#define MODULE_CLKSEL_Msk_ENC(x)    (((x) & 0x0f) << 25)   /*!< CLKSEL mask offset on MODULE index */
#define MODULE_CLKSEL_Pos_ENC(x)    (((x) & 0x1f) << 20)   /*!< CLKSEL position offset on MODULE index */
#define MODULE_CLKDIV_ENC(x)        (((x) & 0x03) << 18)   /*!< APBCLK CLKDIV on MODULE index, 0x0:CLKDIV */
#define MODULE_CLKDIV_Msk_ENC(x)    (((x) & 0xff) << 10)   /*!< CLKDIV mask offset on MODULE index */
#define MODULE_CLKDIV_Pos_ENC(x)    (((x) & 0x1f) <<  5)   /*!< CLKDIV position offset on MODULE index */
#define MODULE_IP_EN_Pos_ENC(x)     (((x) & 0x1f) <<  0)   /*!< APBCLK offset on MODULE index */
/*-------------------------------------------------------------------------------------------------------------------------------*/        
/*   APBCLK(1) | CLKSEL(2) | CLKSEL_Msk(4) |    CLKSEL_Pos(5)    | CLKDIV(2) | CLKDIV_Msk(8) |     CLKDIV_Pos(5)  |  IP_EN_Pos(5)*/
/*-------------------------------------------------------------------------------------------------------------------------------*/
#define ISP_MODULE       (( 0UL<<31)|( 0<<29)|( MODULE_NoMsk<<25)|( 0<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 2<<0)) /*!< ISP Module  \hideinitializer */
#define WDT_MODULE       (( 1UL<<31)|( 1<<29)|(            3<<25)|( 0<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 0<<0)) /*!< WDT/WWDT Module   \hideinitializer */
#define WWDT_MODULE       (( 1UL<<31)|( 3<<29)|(            3<<25)|( 0<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 0<<0)) /*!< WDT/WWDT Module   \hideinitializer */
#define TMR0_MODULE      (( 1UL<<31)|( 1<<29)|(            7<<25)|( 8<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 2<<0)) /*!< TMR0 Module  \hideinitializer */
#define TMR1_MODULE      (( 1UL<<31)|( 1<<29)|(            7<<25)|(12<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 3<<0)) /*!< TMR1 Module  \hideinitializer */
#define	TMR2_MODULE		(( 1UL<<31)|( 1<<29)|(            7<<25)|(16<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 4<<0)) /*!< TMR2 Module  \hideinitializer */	
#define CLKO_MODULE      (( 1UL<<31)|( 3<<29)|(            3<<25)|( 2<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 6<<0)) /*!< CLKO Module  \hideinitializer */
#define I2C0_MODULE      (( 1UL<<31)|( 0<<29)|( MODULE_NoMsk<<25)|( 0<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 8<<0)) /*!< I2C0 Module  \hideinitializer */
#define I2C1_MODULE      (( 1UL<<31)|( 0<<29)|( MODULE_NoMsk<<25)|( 0<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 9<<0)) /*!< I2C1 Module  \hideinitializer */
#define SPI0_MODULE      (( 1UL<<31)|( 1<<29)|(            3<<25)|( 4<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|(12<<0)) /*!< SPI Module  \hideinitializer */
#define SPI1_MODULE      (( 1UL<<31)|( 1<<29)|(            3<<25)|( 6<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|(13<<0)) /*!< SPI Module  \hideinitializer */
#define UART0_MODULE     (( 1UL<<31)|( 1<<29)|(            3<<25)|(24<<20)|( 0<<18)|(          0xF<<10)|( 8<<5)|(16<<0)) /*!< UART0 Module  \hideinitializer */
#define UART1_MODULE     (( 1UL<<31)|( 1<<29)|(            3<<25)|(24<<20)|( 0<<18)|(          0xF<<10)|( 8<<5)|(17<<0)) /*!< UART1 Module  \hideinitializer */
#define PWMCH01_MODULE   (( 1UL<<31)|( 1<<29)|(            3<<25)|(28<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|(20<<0)) /*!< PWMCH01 Module  \hideinitializer */
#define PWMCH23_MODULE   (( 1UL<<31)|( 1<<29)|(            3<<25)|(30<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|(21<<0)) /*!< PWMCH23 Module  \hideinitializer */
#define PWMCH45_MODULE   (( 1UL<<31)|( 3<<29)|(            3<<25)|( 4<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|(22<<0)) /*!< PWMCH45 Module  \hideinitializer */
#define PWMCH67_MODULE   (( 1UL<<31)|( 3<<29)|(            3<<25)|( 6<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|(23<<0)) /*!< PWMCH45 Module  \hideinitializer */
#define ADC_MODULE       (( 1UL<<31)|( 1<<29)|(            3<<25)|( 2<<20)|( 0<<18)|(         0xFF<<10)|(16<<5)|(28<<0)) /*!< ADC Module  \hideinitializer */
#define ACMP_MODULE      (( 1UL<<31)|( 0<<29)|( MODULE_NoMsk<<25)|( 0<<20)|( 0<<18)|( MODULE_NoMsk<<10)|( 0<<5)|(30<<0)) /*!< ACMP Module  \hideinitializer */




/*@}*/ /* end of group PN020_CLK_EXPORTED_CONSTANTS */


/** @addtogroup PN020_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/



__STATIC_INLINE uint32_t CLK_GetPLLClockFreq(void)
{
   uint32_t u32PllReg;
   u32PllReg = CLK->PLLCTL;
    
   if(u32PllReg & (CLK_PLLCTL_PD_Msk ))
       return 0; 

   return 48000000;
}
void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetHXTFreq(void);
uint32_t CLK_GetEXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_DisableSysTick(void);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
void CLK_SysTickDelay(uint32_t us);
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq);
void CLK_DisablePLL(void);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);
void CLK_InitHIRC(void);
void CLK_InitLIRC(void);
void CLK_InitHXT(void);
void CLK_InitHXTPLL(void);
void CLK_InitExtOSC4MPLL(void);
void CLK_InitExtOSC8MPLL(void);
/*@}*/ /* end of group PN020_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group PN020_CLK_Driver */

/*@}*/ /* end of group PN020_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__CLK_H__

/*** (C) COPYRIGHT 2016 Shanghai Panchip Microelectronics Co., Ltd.   ***/
