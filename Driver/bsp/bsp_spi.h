#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include "PN020Series.h"
#include "spi.h"

#define SS1_HIGH SPI1->SSCTL = 0 
#define SS1_LOW  SPI1->SSCTL = 1
#define         _SPI                 SPI1  
#define         CSN_HIGH             SS1_HIGH  
#define         CSN_LOW              SS1_LOW 

#define SPI_CLOCK_FREQ  100000

/*
 * Define functions prototype
 */
void SPI1_Init(void);
__INLINE uint8_t SPI_RW(uint8_t R_REG);
void SPI_WriteReg( uint8_t reg,  uint8_t wdata);
void SPI_WriteBuf( uint8_t reg, uint8_t *pBuf, uint8_t length);
uint8_t SPI_ReadReg( uint8_t reg);
void SPI_ReadBuf( uint8_t reg, unsigned char *pBuf,  uint8_t length);

#endif
