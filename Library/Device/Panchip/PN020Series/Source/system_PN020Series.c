/**************************************************************************//**
 * @file     system_PN020Series.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/02/26 9:13 $
 * @brief    PN020 series system clock init code and assert handler
 *
 * @note
 * Copyright (C) 2016 Shanghai Panchip Microelectronics Co., Ltd.   All rights reserved.
 *****************************************************************************/

#include <stdint.h>
#include "PN020Series.h"


/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t __HSI = __HIRC;                 /*!< Factory Default is internal high speed RC 44.2368M divided by 2 */
uint32_t SystemCoreClock;                /*!< System Clock Frequency (Core Clock) */
uint32_t PllClock = __HIRC;              /*!< PLL Output Clock Frequency         */
uint32_t CyclesPerUs;                    /*!< Cycles per micro second */

/**
 *  @brief  Check HIRC clock rate feed to HCLK
 *
 *  @return none
 */

void SystemInit (void)
{


}

/**
  * @brief  This function is used to update the variable SystemCoreClock
  *   and must be called whenever the core clock is changed.
  * @param  None.
  * @retval None.
  */

void SystemCoreClockUpdate (void)
{
    uint32_t u32CoreFreq, u32ClkSrc;

    /* Update PLL Clock */
    PllClock = CLK_GetPLLClockFreq();

    u32ClkSrc = CLK->CLKSEL0 & CLK_CLKSEL0_HCLKSEL_Msk;

    if (u32ClkSrc == 0)
        u32CoreFreq = __HIRC;       /* External crystal clock */
    else if (u32ClkSrc ==1)
        u32CoreFreq = __PLL;       	/* PLL clock */
    else if (u32ClkSrc == 2)
        u32CoreFreq = __IRC10K;     
    else if (u32ClkSrc ==  3)
        u32CoreFreq = __HXT;      
    else
        u32CoreFreq = __HIRC;      

    SystemCoreClock = (u32CoreFreq/((0x1ul << (CLK->CLKDIV & CLK_CLKDIV_HCLKDIV_Msk))));
    CyclesPerUs = (SystemCoreClock + 500000) / 1000000;
}



#if USE_ASSERT

/**
 * @brief      Assert Error Message
 *
 * @param[in]  file  the source file name
 * @param[in]  line  line number
 *
 * @return     None
 *
 * @details    The function prints the source file name and line number where
 *             the ASSERT_PARAM() error occurs, and then stops in an infinite loop.
 */
void AssertError(uint8_t * file, uint32_t line)
{

//    printf("[%s] line %d : wrong parameters.\r\n", file, line);

    /* Infinite loop */
    while(1) ;
}
#endif

/*** (C) COPYRIGHT 2016 Shanghai Panchip Microelectronics Co., Ltd.   ***/
