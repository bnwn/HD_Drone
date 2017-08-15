#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include "PN020Series.h"
#include "i2c.h"

#define PAGE_SIZE 1024

/*
 * Define functions prototype
 */
extern void I2C_Init(I2C_T *_i2c, uint32_t _bus_clock);
extern void I2C_WriteByte(uint8_t _slave_addr, uint32_t _addr, uint8_t _data);
extern uint8_t I2C_ReadByte(uint8_t _slave_addr, uint32_t _addr);
extern void I2C_PageWrite(uint8_t _salve_addr, uint32_t _addr, uint8_t *_buf);
extern uint32_t I2C_SequentialRead(uint8_t _slave_addr, uint32_t _addr, uint8_t *_buf, uint32_t _size);

#endif
