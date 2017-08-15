/**************************************************************************//**
 * @file     spi.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/02/23 15:33 $ 
 * @brief    PN020 series SPI driver header file
 *
 * @note
 * Copyright (C) 2016 Shanghai Panchip Microelectronics Co., Ltd.   All rights reserved.
 *****************************************************************************/ 
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup PN020_Device_Driver PN020 Device Driver
  @{
*/

/** @addtogroup PN020_SPI_Driver SPI Driver
  @{
*/

/** @addtogroup PN020_SPI_EXPORTED_CONSTANTS SPI Exported Constants
  @{
*/

#define SPI_MODE_0        (SPI_CTL_TXNEG_Msk)                            /*!< CLKP=0; RX_NEG=0; TX_NEG=1 */
#define SPI_MODE_1        (SPI_CTL_RXNEG_Msk)                            /*!< CLKP=0; RX_NEG=1; TX_NEG=0 */
#define SPI_MODE_2        (SPI_CTL_CLKPOL_Msk | SPI_CTL_RXNEG_Msk)       /*!< CLKP=1; RX_NEG=1; TX_NEG=0 */
#define SPI_MODE_3        (SPI_CTL_CLKPOL_Msk | SPI_CTL_TXNEG_Msk)       /*!< CLKP=1; RX_NEG=0; TX_NEG=1 */

#define SPI_SLAVE         (SPI_CTL_SLAVE_Msk)                             /*!< Set as slave */
#define SPI_MASTER        (0x0)                                             /*!< Set as master */

#define SPI_SS                (SPI_SSCTL_SS_Msk)                             /*!< Set SS0 */
#define SPI_SS_ACTIVE_HIGH    (SPI_SSCTL_SSACTPOL_Msk)                          /*!< SS active high */
#define SPI_SS_ACTIVE_LOW     (0x0)                                         /*!< SS active low */

#define SPI_IE_MASK                        (0x01)                           /*!< Interrupt enable mask */
#define SPI_SSTA_SSINAIEN_MASK             (0x02)                             /*!<  Slave Select Inactive  mask */        
#define SPI_FIFO_RXOV_INTEN_MASK           (0x04)                           /*!< FIFO RX overrun interrupt mask */


/*@}*/ /* end of group PN020_SPI_EXPORTED_CONSTANTS */


/** @addtogroup PN020_SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/



/**
  * @brief  Clear the unit transfer interrupt flag.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
#define SPI_CLR_UNIT_TRANS_INT_FLAG(spi) ( (spi)->STATUS = SPI_STATUS_UNITIF_Msk )

/**
  * @brief  Clear the unit transfer flag.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
#define SPI_CLR_UNIT_TRANS_FLAG(spi) ( (spi)->STATUS = SPI_STATUS_UNITF_Msk )



/**
  * @brief  Get the datum read from Rx FIFO.
  * @param[in]  spi is the base address of SPI module.
  * @return Data in Rx buffer
  */
#define SPI_READ_RX(spi) ( (spi)->RX )

/**
  * @brief  Write datum to TX register.
  * @param[in]  spi is the base address of SPI module.
  * @param[in]  u32TxData is the datum which user attempt to transfer through SPI bus.
  * @return none
  */
#define SPI_WRITE_TX(spi, u32TxData) ( (spi)->TX = u32TxData )



/**
  * @brief  Disable automatic slave select function and set SPI_SS pin to high state.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
static __INLINE void SPI_SET_SS_ACTIVE(SPI_T *spi)
{
  spi->SSCTL &= ~SPI_SSCTL_AUTOSS_Msk;  
  spi->SSCTL |= ( SPI_SSCTL_SS_Msk);
}

/**
  * @brief  Disable automatic slave select function and set SPI_SS pin to low state.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
static __INLINE void SPI_SET_SS_INACTIVE(SPI_T *spi)
{
  spi->SSCTL &= ~SPI_SSCTL_AUTOSS_Msk;
  spi->SSCTL &= ~SPI_SSCTL_SS_Msk;
}


/**
  * @brief  Set the SPI transfer sequence with LSB first.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
#define SPI_SET_LSB_FIRST(spi) ( (spi)->CTL |= SPI_CTL_LSB_Msk )

/**
  * @brief  Set the SPI transfer sequence with MSB first.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
#define SPI_SET_MSB_FIRST(spi) ( (spi)->CTL &= ~SPI_CTL_LSB_Msk )

/**
  * @brief  Set the data width of a SPI transaction.
  * @param[in]  spi is the base address of SPI module.
  * @param[in]  u32Width is the bit width of transfer data.  
  * @return none
  */
static __INLINE void SPI_SET_DATA_WIDTH(SPI_T *spi, uint32_t u32Width)
{
   if(u32Width == 32)
        u32Width = 0;
        
   spi->CTL = (spi->CTL & ~SPI_CTL_DWIDTH_Msk) | (u32Width << SPI_CTL_DWIDTH_Pos);
}

/**
  * @brief  Get the SPI busy state.
  * @param[in]  spi is the base address of SPI module.
  * @return SPI busy status
  * @retval 0 SPI module is not busy
  * @retval 1 SPI module is busy
  */
#define SPI_IS_BUSY(spi) ( ((spi)->CTL & SPI_CTL_SPIEN_Msk) == SPI_CTL_SPIEN_Msk ? 1:0 )

/**
  * @brief  Set the GO_BUSY bit to trigger SPI transfer.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
#define SPI_TRIGGER(spi) ( (spi)->CTL |= SPI_CTL_SPIEN_Msk )

uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void SPI_Close(SPI_T *spi);
void SPI_DisableAutoSS(SPI_T *spi, uint32_t u32ActiveLevel);
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock);
uint32_t SPI_GetBusClock(SPI_T *spi);
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask);
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask);

/*@}*/ /* end of group PN020_SPI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group PN020_SPI_Driver */

/*@}*/ /* end of group PN020_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SPI_H__

/*** (C) COPYRIGHT 2016 Shanghai Panchip Microelectronics Co., Ltd.   ***/
