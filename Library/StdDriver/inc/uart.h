/**************************************************************************//**
 * @file     UART.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/02/19 14:25 $ 
 * @brief    PN020 series UART driver header file
 *
 * @note
 * Copyright (C) 2016 Shanghai Panchip Microelectronics Co., Ltd.   All rights reserved.
 *****************************************************************************/ 
#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup PN020_Device_Driver PN020 Device Driver
  @{
*/

/** @addtogroup PN020_UART_Driver UART Driver
  @{
*/

/** @addtogroup PN020_UART_EXPORTED_CONSTANTS UART Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* UART_FIFO constants definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/

#define UART_FIFO_RFITL_1BYTE        (0x0 << UART_FIFO_RFITL_Pos)   /*!< UART_FIFO setting to set RX FIFO Trigger Level to 1 bit */
#define UART_FIFO_RFITL_2BYTES       (0x1 << UART_FIFO_RFITL_Pos)   /*!< UART_FIFO setting to set RX FIFO Trigger Level to 2 bits */
#define UART_FIFO_RFITL_4BYTES       (0x2 << UART_FIFO_RFITL_Pos)   /*!< UART_FIFO setting to set RX FIFO Trigger Level to 4 bits */
#define UART_FIFO_RFITL_6BYTES      (0x3 << UART_FIFO_RFITL_Pos)   /*!< UART_FIFO setting to set RX FIFO Trigger Level to 6 bits */



/*---------------------------------------------------------------------------------------------------------*/
/* UART_LINE constants definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define UART_WORD_LEN_5        (0) /*!< UART_LINE setting to set UART word length to 5 bits */
#define UART_WORD_LEN_6        (1) /*!< UART_LINE setting to set UART word length to 6 bits */
#define UART_WORD_LEN_7        (2) /*!< UART_LINE setting to set UART word length to 7 bits */
#define UART_WORD_LEN_8        (3) /*!< UART_LINE setting to set UART word length to 8 bits */

#define UART_PARITY_NONE    (0x0 << UART_LINE_PBE_Pos) /*!< UART_LINE setting to set UART as no parity   */
#define UART_PARITY_ODD     (0x1 << UART_LINE_PBE_Pos) /*!< UART_LINE setting to set UART as odd parity  */
#define UART_PARITY_EVEN    (0x3 << UART_LINE_PBE_Pos) /*!< UART_LINE setting to set UART as even parity */
#define UART_PARITY_MARK    (0x5 << UART_LINE_PBE_Pos) /*!< UART_LINE setting to keep parity bit as '1'  */
#define UART_PARITY_SPACE   (0x7 << UART_LINE_PBE_Pos) /*!< UART_LINE setting to keep parity bit as '0'  */

#define UART_STOP_BIT_1     (0x0 << UART_LINE_NSB_Pos) /*!< UART_LINE setting for one stop bit  */
#define UART_STOP_BIT_1_5   (0x1 << UART_LINE_NSB_Pos) /*!< UART_LINE setting for 1.5 stop bit when 5-bit word length  */
#define UART_STOP_BIT_2     (0x1 << UART_LINE_NSB_Pos) /*!< UART_LINE setting for two stop bit when 6, 7, 8-bit word length */



/*@}*/ /* end of group PN020_UART_EXPORTED_CONSTANTS */


/** @addtogroup PN020_UART_EXPORTED_FUNCTIONS UART Exported Functions
  @{
*/



/**
 *    @brief    Write Data to Tx data register
 *
 *    @param[in]    uart        The base address of UART module.
 *    @param[in]    u8Data  Data byte to transmit 
 *                          
 *    @return    None
 */
#define UART_WRITE(uart, u8Data)    (uart->DAT = (u8Data))


/**
 *    @brief    Read Rx data register
 *
 *    @param[in]    uart        The base address of UART module. 
 *
 *    @return    The oldest data byte in RX FIFO 
 */
#define UART_READ(uart)    (uart->DAT)


/**
 *    @brief    Get Tx empty register value. 
 *
 *    @param[in]   uart        The base address of UART module  
 *
 *    @return    Tx empty register value. 
 */
#define UART_GET_TX_EMPTY(uart)    (uart->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) 


/**
 *    @brief    Get Rx empty register value. 
 *
 *    @param[in]    uart        The base address of UART module 
 * 
 *    @return    Rx empty register value.
 */
#define UART_GET_RX_EMPTY(uart)    (uart->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) 

/**
 *    @brief    Check specified uart port transmission is over. 
 *
 *    @param[in]    uart        The base address of UART module 
 * 
 *    @return    TE_Flag.
 */
#define UART_IS_TX_EMPTY(uart)    ((uart->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) >> UART_FIFOSTS_TXEMPTYF_Pos) 


/**
 *    @brief    Wait specified uart port transmission is over 
 *
 *    @param[in]    uart        The base address of UART module 
 *
 *    @return    None 
 */
#define UART_WAIT_TX_EMPTY(uart)    while(!(((uart->FIFOSTS) & UART_FIFOSTS_TXEMPTYF_Msk) >> UART_FIFOSTS_TXEMPTYF_Pos))

                                     
/**
 *    @brief    Check RDA_IF is set or not
 *
 *    @param[in]    uart        The base address of UART module 
 *
 *    @return     
 *            0 : The number of bytes in the RX FIFO is less than the RFITL  
 *            1 : The number of bytes in the RX FIFO equals or larger than RFITL
 */
#define UART_IS_RX_READY(uart)    ((uart->INTSTS & UART_INTSTS_RDAIF_Msk)>>UART_INTSTS_RDAIF_Pos)


/**
 *    @brief    Check TX FIFO is full or not 
 *
 *    @param[in]    uart        The base address of UART module 
 *
 *    @return     
 *            1 = TX FIFO is full
 *            0 = TX FIFO is not full
 */
#define UART_IS_TX_FULL(uart)    ((uart->FIFOSTS & UART_FIFOSTS_TXFULL_Msk)>>UART_FIFOSTS_TXFULL_Pos)

/**
 *    @brief    Check RX FIFO is full or not 
 *
 *    @param[in]    uart        The base address of UART module  
 *
 *    @return
 *            1 = RX FIFO is full
 *            0 = RX FIFO is not full
 *
 */
#define UART_IS_RX_FULL(uart)    ((uart->FIFOSTS & UART_FIFOSTS_RXFULL_Msk)>>UART_FIFOSTS_RXFULL_Pos)


/**
 *    @brief    Get Tx full register value 
 *
 *    @param[in]    uart        The base address of UART module 
 *
 *    @return    Tx full register value 
 */
#define UART_GET_TX_FULL(uart)    (uart->FIFOSTS & UART_FIFOSTS_TXFULL_Msk)


/**
 *    @brief    Get Rx full register value 
 *
 *    @param[in]    uart        The base address of UART module 
 *
 *    @return    Rx full register value 
 */
#define UART_GET_RX_FULL(uart)    (uart->FIFOSTS & UART_FIFOSTS_RXFULL_Msk)


/**
 *    @brief    Enable specified interrupt
 *
 *    @param[in]    uart        The base address of UART module  
 *    @param[in]    u32eIntSel    Interrupt type select 
 *                        - \ref UART_INTEN_TOCNTEN_Msk        : Rx Time Out interrupt
 *                        - \ref UART_INTEN_BUFERRIEN_Msk      : Buffer Error interrupt   
 *                        - \ref UART_INTEN_RXTOIEN_Msk        : Rx time-out interrupt       
 *                        - \ref UART_INTEN_RLSIEN_Msk         : Rx Line status interrupt                              
 *                        - \ref UART_INTEN_THREIEN_Msk        : Tx empty interrupt          
 *                        - \ref UART_INTEN_RDAIEN_Msk         : Rx ready interrupt           
 *
 *    @return    None                                                    
 */
#define UART_ENABLE_INT(uart, u32eIntSel)    (uart->INTEN |= (u32eIntSel))


/**
 *    @brief    Disable specified interrupt  
 *
 *    @param[in]    uart        The base address of UART module 
 *    @param[in]    u32eIntSel    Interrupt type select 
 *                        - \ref UART_INTEN_TOCNTEN_Msk        : Rx Time Out interrupt
 *                        - \ref UART_INTEN_BUFERRIEN_Msk      : Buffer Error interrupt   
 *                        - \ref UART_INTEN_RXTOIEN_Msk        : Rx time-out interrupt           
 *                        - \ref UART_INTEN_RLSIEN_Msk         : Rx Line status interrupt                              
 *                        - \ref UART_INTEN_THREIEN_Msk        : Tx empty interrupt          
 *                        - \ref UART_INTEN_RDAIEN_Msk         : Rx ready interrupt          
 *    @return    None                                                    
 */
#define UART_DISABLE_INT(uart, u32eIntSel)    (uart->INTEN &= ~ (u32eIntSel))


/**
 *    @brief    Get specified interrupt flag/status
 *
 *    @param[in]    uart            The base address of UART module 
 *    @param[in]    u32eIntTypeFlag    Interrupt Type Flag,should be   
 *                            - \ref UART_INTSTS_BUFERRINT_Msk     : Buffer Error Interrupt Indicator
 *                            - \ref UART_INTSTS_RXTOINT_Msk       : Time-out Interrupt Indicator 
 *                            - \ref UART_INTSTS_RLSINT_Msk        : Receive Line Status Interrupt 
 *                            - \ref UART_INTSTS_THREINT_Msk       : Transmit Holding Register Empty Interrupt Indicator
 *                            - \ref UART_INTSTS_RDAINT_Msk        : Receive Data Available Interrupt Indicator 
 *                            - \ref UART_INTSTS_BUFERRIF_Msk      : Buffer Error Interrupt Flag   
 *                            - \ref UART_INTSTS_RXTOIF_Msk        : Rx time-out interrupt Flag        
 *                            - \ref UART_INTSTS_RLSIF_Msk         : Rx Line status interrupt Flag                              
 *                            - \ref UART_INTSTS_THREIF_Msk        : Tx empty interrupt Flag
 *                            - \ref UART_INTSTS_RDAIF_Msk         : Rx ready interrupt Flag 
 *
 *    @return
 *            0 = The specified interrupt is not happened. 
 *            1 = The specified interrupt is happened.                                                      
 */                                                                                      
#define UART_GET_INT_FLAG(uart,u32eIntTypeFlag)    ((uart->INTSTS & (u32eIntTypeFlag))?1:0)


                                                                                                                       


void UART_ClearIntFlag(UART_T* uart , uint32_t u32InterruptFlag);
void UART_Close(UART_T* uart );
void UART_DisableInt(UART_T*  uart, uint32_t u32InterruptFlag );
void UART_EnableInt(UART_T*  uart, uint32_t u32InterruptFlag );
void UART_Open(UART_T* uart, uint32_t u32baudrate);
uint32_t UART_Read(UART_T* uart, uint8_t *pu8RxBuf, uint32_t u32ReadBytes);
void UART_SetLine_Config(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits);
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC);
uint32_t UART_Write(UART_T* uart,uint8_t *pu8TxBuf, uint32_t u32WriteBytes);


/*@}*/ /* end of group PN020_UART_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group PN020_UART_Driver */

/*@}*/ /* end of group PN020_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__UART_H__

/*** (C) COPYRIGHT 2016 Shanghai Panchip Microelectronics Co., Ltd.   ***/








