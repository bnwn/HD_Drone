/**************************************************************************//**
 * @file     clk.c
 * @version  V1.01
 * $Date: 16/05/25 9:39a $
 * @brief    PN020 series CLK driver source file
 *
 * @note
 * Copyright (C) 2016 Shanghai Panchip Microelectronics Co., Ltd.   All rights reserved.
 *****************************************************************************/
#include "PN020Series.h"
/** @addtogroup PN020_Device_Driver PN020 Device Driver
  @{
*/

/** @addtogroup PN020_CLK_Driver CLK Driver
  @{
*/


/** @addtogroup PN020_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief  This function let system enter to Power-down mode.
  * @return None
  */
void CLK_PowerDown(void)
{
	SCB->SCR = SCB_SCR_SLEEPDEEP_Msk;
	CLK->WAKEUPCTL	&= ~(0x3f<<24 )  ;
	CLK->WAKEUPCTL |= 1<<26 ;
    CLK->PWRCTL |= (CLK_PWRCTL_PDEN_Msk | CLK_PWRCTL_PDWKIF_Msk);

    __WFI();
}

/**
  * @brief  This function let system enter to Idle mode
  * @return None
  */
void CLK_Idle(void)
{
	CLK->WAKEUPCTL	&= ~(0x3f<<24 )  ;
	CLK->WAKEUPCTL |= 1<<26 ;
    CLK->PWRCTL |= (CLK_PWRCTL_PDEN_Msk | CLK_PWRCTL_PDWKIF_Msk);
    __WFI();
}

/**
  * @brief  This function get external high frequency crystal frequency. The frequency unit is Hz.
  * @return None
  */
uint32_t CLK_GetHXTFreq(void)
{
    if((CLK->PWRCTL & CLK_PWRCTL_PDXTL_Msk)==CLK_PWRCTL_PDXTL_Msk)
        return 0;
    else
        return __HXT;
}

/**
  * @brief  This function get external  frequency input frequency. The frequency unit is Hz.
  * @return LXT frequency
  */
uint32_t CLK_GetEXTFreq(void)
{

        return __EXT;

}

/**
  * @brief  This function get HCLK frequency. The frequency unit is Hz.
  * @return HCLK frequency
  */
uint32_t CLK_GetHCLKFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}


/**
  * @brief  This function get CPU frequency. The frequency unit is Hz.
  * @return CPU frequency
  */
uint32_t CLK_GetCPUFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}



/**
  * @brief  This function set HCLK clock source and HCLK clock divider
  * @param[in]  u32ClkSrc is HCLK clock source. Including :
  *                  - \ref CLK_CLKSEL0_HCLKSEL_XTAL
  *                  - \ref CLK_CLKSEL0_HCLKSEL_LIRC
  *                  - \ref CLK_CLKSEL0_HCLKSEL_HIRC
  * @param[in]  u32ClkDiv is HCLK clock divider. Including :
  *                  - \ref CLK_CLKDIV_HCLK(x)
  * @return None
  */
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    /* Apply new Divider */
    CLK->CLKDIV = (CLK->CLKDIV & ~CLK_CLKDIV_HCLKDIV_Msk) | u32ClkDiv;

    /* Switch HCLK to new HCLK source */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | u32ClkSrc;

    /* Update System Core Clock */
    SystemCoreClockUpdate();
}

/**
  * @brief  This function set selected module clock source and module clock divider
  * @param[in]  u32ModuleIdx is module index.
  * @param[in]  u32ClkSrc is module clock source.
  * @return None
  * @details Valid parameter combinations listed in following table:
  *
  * |Module index             |Clock source                                           
  * | :----------------  | :------------------------------------------ |
  * |\ref WDT_MODULE     |\ref CLK_CLKSEL1_WDTSEL_HCLK_DIV2048     
  * |\ref WDT_MODULE     |\ref CLK_CLKSEL1_WDTSEL_LIRC                  

  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_LIRC                 
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_HCLK                
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0SEL_TM0                 
  
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_LIRC                 
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_HCLK                
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1SEL_TM1                 

  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_LIRC                 
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_HCLK                
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2SEL_TM1                 

  * |\ref WWDT_MODULE   |\ref CLK_CLKSEL2_WWDTSEL_HCLK_DIV2048 
  * |\ref WWDT_MODULE   |\ref CLK_CLKSEL2_WWDTSEL_LIRC    

  * |\ref CLKO_MODULE    |\ref CLK_CLKSEL2_CLKOSEL_HXT        
  * |\ref CLKO_MODULE    |\ref CLK_CLKSEL2_CLKOSEL_LIRC       

  */


void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc)
{
    uint32_t u32tmp=0,u32sel=0;

    if(MODULE_CLKSEL_Msk(u32ModuleIdx)!=MODULE_NoMsk) {
        u32sel = (uint32_t)&CLK->CLKSEL0+((MODULE_CLKSEL(u32ModuleIdx))*4);
        u32tmp = *(volatile uint32_t *)(u32sel);
        u32tmp = ( u32tmp & ~(MODULE_CLKSEL_Msk(u32ModuleIdx)<<MODULE_CLKSEL_Pos(u32ModuleIdx)) ) | u32ClkSrc;
        *(volatile uint32_t *)(u32sel) = u32tmp;
    }
}



/**
  * @brief      Disable System Tick counter
  * @return     None
  * @details    This function disable System Tick counter.
  */
void CLK_DisableSysTick(void)
{
    SysTick->CTRL = 0;    /* Set System Tick counter disabled */
}


/**
  * @brief  This function enable module clock
  * @param[in]  u32ModuleIdx is module index. Including :
  *                  - \ref WDT_MODULE
  *                  - \ref TMR0_MODULE
  *                  - \ref TMR1_MODULE
  *                  - \ref TMR2_MODULE
  *                  - \ref CLKO_MODULE
  *                  - \ref I2C0_MODULE
  *                  - \ref I2C1_MODULE
  *                  - \ref SPI0_MODULE
  *                  - \ref SPI1_MODULE
  *                  - \ref UART0_MODULE
  *                  - \ref UART1_MODULE
  *                  - \ref PWMCH01_MODULE
  *                  - \ref PWMCH23_MODULE
  *                  - \ref PWMCH45_MODULE
  *                  - \ref PWMCH67_MODULE
  *                  - \ref ADC_MODULE
  *                  - \ref ACMP_MODULE
  * @return None
  */


void CLK_EnableModuleClock(uint32_t u32ModuleIdx)
{
    *(volatile uint32_t *)((uint32_t)&CLK->AHBCLK+(MODULE_APBCLK(u32ModuleIdx)*4))  |= 1<<MODULE_IP_EN_Pos(u32ModuleIdx);
}

/**
  * @brief  This function disable module clock
  * @param[in]  u32ModuleIdx is module index
  *                  - \ref WDT_MODULE
  *                  - \ref TMR0_MODULE
  *                  - \ref TMR1_MODULE
  *                  - \ref CLKO_MODULE
  *                  - \ref I2C0_MODULE
  *                  - \ref I2C1_MODULE
  *                  - \ref SPI0_MODULE
  *                  - \ref SPI1_MODULE
  *                  - \ref UART0_MODULE
  *                  - \ref UART1_MODULE
  *                  - \ref PWMCH01_MODULE
  *                  - \ref PWMCH23_MODULE
  *                  - \ref PWMCH45_MODULE
  *                  - \ref PWMCH67_MODULE
  *                  - \ref ADC_MODULE
  *                  - \ref ACMP_MODULE
  * @return None
  */
void CLK_DisableModuleClock(uint32_t u32ModuleIdx)
{
    *(volatile uint32_t *)((uint32_t)&CLK->AHBCLK+(MODULE_APBCLK(u32ModuleIdx)*4))  &= ~(1<<MODULE_IP_EN_Pos(u32ModuleIdx));
}

  

/**
  * @brief      Disable PLL
  * @param      None
  * @return     None
  * @details    This function set PLL in Power-down mode.
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_DisablePLL(void)
{
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;
}

/**
  * @brief  This function execute delay function.
  * @param[in]  us  Delay time. The Max value is 2^24 / CPU Clock(MHz). Ex:
  *                             50MHz => 335544us, 48MHz => 349525us, 28MHz => 699050us ...
  * @return None
  * @details    Use the SysTick to generate the delay time and the UNIT is in us.
  *             The SysTick clock source is from HCLK, i.e the same as system core clock.
  */
void CLK_SysTickDelay(uint32_t us)
{
    SysTick->LOAD = us * CyclesPerUs;
    SysTick->VAL  =  (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    SysTick->CTRL = 0;
}

/**
  * @brief  This function check selected clock source status
  * @param[in]  u32ClkMask is selected clock source. Including
  *                  - \ref CLK_STATUS_CLKSFAIL_Msk
  *                  - \ref CLK_STATUS_HIRCSTB_Msk
  *                  - \ref CLK_STATUS_LIRCSTB_Msk
  *                  - \ref CLK_STATUS_XTLSTB_Msk
  *
  * @return   0  clock is not stable
  *           1  clock is stable
  *
  * @details  To wait for clock ready by specified CLKSTATUS bit or timeout (~5ms)
  */
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask)
{
    int32_t i32TimeOutCnt = 2160000;

    while((CLK->STATUS & u32ClkMask) != u32ClkMask) {
        if(i32TimeOutCnt-- <= 0)
            return 0;
    }
    return 1;
}


/**
  * @brief      Init the Internal High Speed oscillator (48Mhz).
  * @param    None
  * @return    None

  */
void CLK_InitHIRC(void)
{	
			uint32_t u32pwrctl = 0;
			CLK->PWRCTL &=~(0x0f<<12);
			CLK->PWRCTL |= (0x03<<14);  
			SYS->TESTCTL = 0X300; 
			FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;   
			FMC->ISPCMD = 0x04;
			FMC->ISPADDR = 0x00 ;
			FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
			while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;
	 
			u32pwrctl= (~(FMC->ISPDAT))&(0xff<<16);
			CLK->PWRCTL &= ~(0xff<<16);
			CLK->PWRCTL |=u32pwrctl; 	
	
      CLK->PWRCTL &= ~0x000003FC;
      while((CLK->STATUS & CLK_STATUS_HIRCSTB) == 0);
      CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_HIRC;     
}


/**
  * @brief      Init the Internal Low Speed crystal (10Khz).
  * @param    None
  * @return    None

  */
void CLK_InitLIRC(void)
{	
     
	CLK->PWRCTL &= ~0x000003FC; 											
	while((CLK->STATUS & CLK_STATUS_LIRCSTB) == 0);
	CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_LIRC;  
}


/**
  * @brief      Init the External High Speed crystal (16Mhz).
  * @param    None
  * @return    None

  */

void CLK_InitHXT(void)
{
    CLK->PWRCTL &= ~0x000003FF; 
	SYS->P5_MFP |= SYS_MFP_P50_XT1_IN | SYS_MFP_P51_XT1_OUT;
	while((CLK->STATUS & CLK_STATUS_XTLSTB) == 0);
	CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_XTAL;	
	

}


/**
  * @brief      Init the PLL (48Mhz)  clock source from external 16M crystal 
  * @param    None
  * @return    None
  */

void CLK_InitHXTPLL(void)
{
    CLK->PWRCTL &= ~0x000003FF; 
	SYS->P5_MFP |= SYS_MFP_P50_XT1_IN | SYS_MFP_P51_XT1_OUT;

	CLK->PLLCTL &= ~(CLK_PLLCTL_PLLINFSEL_Msk | CLK_PLLCTL_PLLSRC_Msk | CLK_PLLCTL_PD_Msk);		// use  external 16Mhz  crystal
														
	while((CLK->STATUS & CLK_STATUS_PLLSTB) == 0);
	CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_PLL;
	
	
}


/**
  * @brief      Init the PLL (48Mhz)  clock source from external 4M  oscillator
  * @param    None
  * @return    None
  */

void CLK_InitExtOSC4MPLL(void)
{
	CLK->PWRCTL &= ~0x000003FF; 
    SYS->P2_MFP |=SYS_MFP_P21_EXTCLK;
    P2->DINOFF  &= ~ ( GP_DINOFF_DINOFF1_Msk ); 
	CLK->PLLCTL &= ~(CLK_PLLCTL_PLLINFSEL_Msk | CLK_PLLCTL_PLLSRC_Msk | CLK_PLLCTL_PD_Msk);
	CLK->PLLCTL |= CLK_PLLCTL_PLLSRC_Msk;				// use ext  4M clk;	
	while((CLK->STATUS & CLK_STATUS_PLLSTB) == 0);	
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_PLL;

}


/**
  * @brief      Init the PLL (48Mhz)  clock source from external 8M  oscillator
  * @param    None
  * @return    None
  */

void CLK_InitExtOSC8MPLL(void)
{
	CLK->PWRCTL &= ~0x000003FF; 
    SYS->P2_MFP |=SYS_MFP_P21_EXTCLK;
    P2->DINOFF  &= ~ ( GP_DINOFF_DINOFF1_Msk ); 
	CLK->PLLCTL &= ~(CLK_PLLCTL_PLLINFSEL_Msk | CLK_PLLCTL_PLLSRC_Msk | CLK_PLLCTL_PD_Msk);
	CLK->PLLCTL |= (CLK_PLLCTL_PLLSRC_Msk|CLK_PLLCTL_PLLINFSEL_Msk);				// use ext  8M clk;	
	while((CLK->STATUS & CLK_STATUS_PLLSTB) == 0);	
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_PLL;

}



/*@}*/ /* end of group PN020_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group PN020_CLK_Driver */

/*@}*/ /* end of group PN020_Device_Driver */

/*** (C) COPYRIGHT 2016 Shanghai Panchip Microelectronics Co., Ltd.   ***/
