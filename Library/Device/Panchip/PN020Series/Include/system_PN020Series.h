/**************************************************************************//**
 * @file     system_PN020Series.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/02/26 10:19a $
 * @brief    PN020 series system clock definition file
 *
 * @note
 * Copyright (C) 2016 Shanghai Panchip Microelectronics Co., Ltd.   All rights reserved.
 *****************************************************************************/


#ifndef __SYSTEM_PN020SERIES_H__
#define __SYSTEM_PN020SERIES_H__

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Macro Definition                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
  Define SYSCLK
 *----------------------------------------------------------------------------*/

#define __XTAL4M        	(4000000UL)
 #define __XTAL16M        	(16000000UL)   
#define __XTAL32K        	(32768UL)
#define __IRC10K         	(10000UL)
#define __XTAL            	__XTAL4M

#define	__EXT				__XTAL4M
#define __HXT             	__XTAL16M
#define __HIRC           	(48000000ul)
#define	__LIRC				__IRC10K
#define	__PLL				(48000000UL)


extern uint32_t __HSI;
extern uint32_t SystemCoreClock;        /*!< System Clock Frequency (Core Clock) */
extern uint32_t CyclesPerUs;            /*!< Cycles per micro second */
extern uint32_t PllClock;               /*!< PLL Output Clock Frequency          */
/**
 * Update SystemCoreClock variable
 *
 * @param  None
 * @return None
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from CPU registers.
 */

extern void SystemCoreClockUpdate (void);
extern void SystemInit (void);

#ifdef __cplusplus
}
#endif

#endif  //__SYSTEM_PN020SERIES_H__


/*** (C) COPYRIGHT 2016 Shanghai Panchip Microelectronics Co., Ltd.   ***/
