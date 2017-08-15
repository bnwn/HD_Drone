/**************************************************************************//**
* @file     uart.c
* @version  V1.00
* $Revision: 2 $
* $Date: 16/02/25 14:25 $
* @brief    PN020 series UART driver source file
*
* @note
* Copyright (C) 2016 Shanghai Panchip Microelectronics Co., Ltd.   All rights reserved.
*****************************************************************************/

//#include <stdio.h>
#include "PN020Series.h"

/** @addtogroup PN020_Device_Driver PN020 Device Driver
  @{
*/

/** @addtogroup PN020_UART_Driver UART Driver
  @{
*/


/** @addtogroup PN020_UART_EXPORTED_FUNCTIONS UART Exported Functions
  @{
*/

/**
 *    @brief The function is used to clear UART specified interrupt flag.
 *
 *    @param[in] uart                The base address of UART module.
 *    @param[in] u32InterruptFlag    The specified interrupt of UART module.
 *
 *    @return None
 */
void UART_ClearIntFlag(UART_T* uart , uint32_t u32InterruptFlag)
{

    if(u32InterruptFlag & UART_INTSTS_RLSINT_Msk) { /* clear Receive Line Status Interrupt */
        uart->FIFOSTS = UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk;
    }

    if(u32InterruptFlag & UART_INTSTS_BUFERRINT_Msk) { /* clear Buffer Error Interrupt */
        uart->FIFOSTS = UART_FIFOSTS_RXOVIF_Msk | UART_FIFOSTS_TXOVIF_Msk;
    }


}


/**
 *  @brief The function is used to disable UART.
 *
 *  @param[in] uart        The base address of UART module.
 *
 *  @return None
 */
void UART_Close(UART_T* uart)
{
    uart->INTEN = 0;
}




/**
 *    @brief    The function is used to disable UART specified interrupt and disable NVIC UART IRQ.
 *
 *    @param[in]    uart                The base address of UART module.
 *    @param[in]    u32InterruptFlag    The specified interrupt of UART module.
 *                                - \ref UART_INTEN_TOCNTEN_Msk        : Rx Time Out interrupt
 *                                - \ref UART_INTEN_BUFERRIEN_Msk      : Buffer Error interrupt
 *                                - \ref UART_INTEN_RXTOIEN_Msk        : Rx time-out interrupt
 *                                - \ref UART_INTEN_RLSIEN_Msk         : Rx Line status interrupt
 *                                - \ref UART_INTEN_THREIEN_Msk        : Tx empty interrupt
 *                                - \ref UART_INTEN_RDAIEN_Msk         : Rx ready interrupt
 *
 *    @return    None
 */
void UART_DisableInt(UART_T*  uart, uint32_t u32InterruptFlag )
{
    uart->INTEN &= ~ u32InterruptFlag;
}



/**
 *    @brief    The function is used to enable UART specified interrupt and disable NVIC UART IRQ.
 *
 *    @param[in]    uart                The base address of UART module.
 *    @param[in]    u32InterruptFlag    The specified interrupt of UART module:
 *                                - \ref UART_INTEN_TOCNTEN_Msk        : Rx Time Out interrupt
 *                                - \ref UART_INTEN_BUFERRIEN_Msk      : Buffer Error interrupt
 *                                - \ref UART_INTEN_RXTOIEN_Msk        : Rx time-out interrupt
 *                                - \ref UART_INTEN_RLSIEN_Msk         : Rx Line status interrupt
 *                                - \ref UART_INTEN_THREIEN_Msk        : Tx empty interrupt
 *                                - \ref UART_INTEN_RDAIEN_Msk         : Rx ready interrupt
 *
 *    @return None
 */
 
void UART_EnableInt(UART_T*  uart, uint32_t u32InterruptFlag )
{
    uart->INTEN |= u32InterruptFlag;
}


/**
 *    @brief    This function use to enable UART function and set baud-rate.
 *
 *    @param[in]    uart    The base address of UART module.
 *    @param[in]    u32baudrate    The baudrate of UART module.
 *
 *    @return    None
 */
void UART_Open(UART_T* uart, uint32_t u32baudrate)
{
    uint32_t u32Clk = 0;
    uint32_t u32Baud_Div;
    
    uart->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    uart->FIFO = UART_FIFO_RFITL_1BYTE ;

	u32Clk = CLK_GetHCLKFreq();
    if(u32baudrate != 0) 
	{
		u32Baud_Div = ((u32Clk-u32baudrate) / ((u32baudrate) << 1));
    
		if(u32Baud_Div > 0xFFFF)
			u32Baud_Div = 0xFFFF;
		uart->BAUD = u32Baud_Div;
    }
}


/**
 *    @brief    The function is used to read Rx data from RX FIFO and the data will be stored in pu8RxBuf.
 *
 *    @param[in]    uart            The base address of UART module.
 *    @param[in]    pu8RxBuf        The buffer to receive the data of receive FIFO.
 *    @param[in]    u32ReadBytes    The the read bytes number of data.
 *
 *  @return     u32Count: Receive byte count
 *
 */
uint32_t UART_Read(UART_T* uart, uint8_t *pu8RxBuf, uint32_t u32ReadBytes)
{
    uint32_t  u32Count, u32delayno;

    for(u32Count=0; u32Count < u32ReadBytes; u32Count++) {
        u32delayno = 0;

        while(uart->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) { /* Check RX empty => failed */
            u32delayno++;
            if( u32delayno >= 0x40000000 )
                return FALSE;
        }
        pu8RxBuf[u32Count] = uart->DAT;    /* Get Data from UART RX  */
    }

    return u32Count;

}


/**
 *    @brief    This function use to config UART line setting.
 *
 *    @param[in]    uart            The base address of UART module.
 *    @param[in]    u32baudrate     The register value of baudrate of UART module.
 *                              if u32baudrate = 0, UART baudrate will not change.
 *    @param[in]    u32data_width   The data length of UART module.
 *    @param[in]    u32parity       The parity setting (odd/even/none) of UART module.
 *    @param[in]    u32stop_bits    The stop bit length (1/1.5 /2bit) of UART module.
 *
 *    @return    None
 */
void UART_SetLine_Config(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits)
{

    uint32_t u32Clk = 0;
    uint32_t u32Baud_Div;
	u32Clk = CLK_GetHCLKFreq();
    if(u32baudrate != 0) 
	{
		u32Baud_Div = ((u32Clk-u32baudrate) / ((u32baudrate) << 1));
    
		if(u32Baud_Div > 0xFFFF)
			u32Baud_Div = 0xFFFF;
		uart->BAUD = u32Baud_Div;
    }

    uart->LINE = u32data_width | u32parity | u32stop_bits;
}


/**
 *    @brief    This function use to set Rx timeout count.
 *
 *    @param[in]    uart      The base address of UART module.
 *    @param[in]    u32TOC    Rx timeout counter.
 *
 *    @return    None
 */
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC)
{
    uart->TOUT = (uart->TOUT & ~UART_TOUT_TOIC_Msk)| (u32TOC);
    uart->INTEN |= UART_INTEN_TOCNTEN_Msk;
}



/**
 *    @brief    The function is to write data into TX buffer to transmit data by UART.
 *
 *    @param[in]    uart            The base address of UART module.
 *    @param[in]    pu8TxBuf        The buffer to send the data to UART transmission FIFO.
 *    @param[in]    u32WriteBytes   The byte number of data.
 *
 *  @return u32Count: transfer byte count
 */
uint32_t UART_Write(UART_T* uart,uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    uint32_t  u32Count, u32delayno;

    for(u32Count=0; u32Count != u32WriteBytes; u32Count++) {
        u32delayno = 0;
        while((uart->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) == 0) { /* Wait Tx empty and Time-out manner */
            u32delayno++;
            if( u32delayno >= 0x40000000 )
                return FALSE;
        }
        uart->DAT = pu8TxBuf[u32Count];    /* Send UART Data from buffer */
    }

    return u32Count;

}


/*@}*/ /* end of group PN020_UART_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group PN020_UART_Driver */

/*@}*/ /* end of group PN020_Device_Driver */

/*** (C) COPYRIGHT 2016 Shanghai Panchip Microelectronics Co., Ltd.   ***/



