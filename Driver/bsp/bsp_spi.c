#include "bsp_spi.h"
#include "timer_delay.h"

void SPI1_Init(void)
{
     SPI_Open(SPI1, SPI_MASTER, SPI_MODE_0, 8, SPI_CLOCK_FREQ);    
     delay_ms(1000);
}

__INLINE uint8_t SPI_RW(uint8_t R_REG)
{
    SPI_WRITE_TX(_SPI, R_REG);
    SPI_TRIGGER(_SPI);
    while(SPI_IS_BUSY(_SPI));
    R_REG = SPI_READ_RX(_SPI);
    return(R_REG);           		  // return read byte
}

void SPI_WriteReg( uint8_t reg,  uint8_t wdata)
{
    CSN_LOW;  
    SPI_RW(reg);
    SPI_RW(wdata);
    CSN_HIGH;  

}

void SPI_WriteBuf( uint8_t reg, uint8_t *pBuf, uint8_t length)
{
    uint8_t i;
    CSN_LOW;
  	 SPI_RW(reg);    // Select register to write to and read status byte
  	for(i=0; i<length; i++) // then write all byte in buffer(*pBuf)    
            SPI_RW(*pBuf++);
	 CSN_HIGH; 

}

uint8_t SPI_ReadReg( uint8_t reg)
{
    uint8_t temp;
     CSN_LOW;  
    SPI_RW(reg); 
    temp =SPI_RW(0);
   CSN_HIGH;      
    return temp ;
}

void SPI_ReadBuf( uint8_t reg, unsigned char *pBuf,  uint8_t length)
{
    uint8_t i;  
    CSN_LOW;     
    SPI_RW(reg);       		                                                		
    for(i=0;i<length;i++)
    	pBuf[i] = SPI_RW(0);
    CSN_HIGH;                                                                   		
}
