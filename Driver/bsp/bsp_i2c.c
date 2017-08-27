#include "PN020Series.h"
#include "bsp_i2c.h"
#include <stdio.h>
#include "common.h"

/**
 * @brief I2C interface initial
 * @param _i2c
 * @param _bus_clock
 */
void I2C_Init(I2C_T *_i2c, uint32_t _bus_clock)
{
    I2C_Open(_i2c, _bus_clock);
}


/**
 * @brief I2C write one byte data
 * @param _addr
 * @param _data
 */
void I2C_WriteByte(uint8_t _slave_addr, uint32_t _addr, uint8_t _data)
{
    int32_t i32Err;

    do {
        i32Err = 0;

        /* Send start */
         I2C_SET_CONTROL_REG(I2C,  I2C_SI|I2C_STA);
        I2C_WAIT_READY(I2C);

        /* Send control byte */
        I2C_SET_DATA(I2C, _slave_addr);
        I2C_SET_CONTROL_REG(I2C, I2C_SI);
        I2C_WAIT_READY(I2C);

        if(I2C_GET_STATUS(I2C) == 0x18) {


            /* Send address */

             I2C_SET_DATA(I2C, _addr & 0xFFUL); // address
            I2C_SET_CONTROL_REG(I2C, I2C_SI);
            I2C_WAIT_READY(I2C){};

               if(I2C_GET_STATUS(I2C) == 0x28) {
                    /* ACK */

                    /* Send data */
                    I2C_SET_DATA(I2C, _data); // data
                    I2C_SET_CONTROL_REG(I2C, I2C_SI);
                    I2C_WAIT_READY(I2C);
                    if(I2C_GET_STATUS(I2C) == 0x28) {
                        /* ACK */

                        /* Send stop */
                        I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI);

                    } else {
                        /* NACK */

                        /* Send data error */
                        i32Err = 4;
                    }

            } else {
                /* NACK */

                /* Send high address error */
                i32Err = 2;
            }
        } else {
            /* NACK */

            /* Send control error */
            i32Err = 1;
        }

        if(i32Err) {
#ifdef __DEVELOP__
						printf("Error: %d£¬ I2C status: %X\n", i32Err, I2C_GET_STATUS(I2C));
#endif
            /* Send stop */
            I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI);

            CLK_SysTickDelay(100);
        }

    } while(i32Err);

}

/**
 * @brief I2C read one byte data
 * @param _slave_write_addr
 * @param _slave_read_addr
 * @param _addr
 * @return
 */
uint8_t I2C_ReadByte(uint8_t _slave_addr, uint32_t _addr)
{
    int32_t i32Err;
    uint8_t u8Data;

    u8Data = 0;
	
    do {
        i32Err = 0;
        TIMER_Delay(TIMER0,100);

        /* Send start */
        I2C_SET_CONTROL_REG(I2C,  I2C_SI|I2C_STA);
        I2C_WAIT_READY(I2C);

        /* Send control byte */
        I2C_SET_DATA(I2C, _slave_addr);
        I2C_SET_CONTROL_REG(I2C, I2C_SI);
        I2C_WAIT_READY(I2C);
				if (I2C_GET_STATUS(I2C) == 0x18) {
        //if(I2C_GET_STATUS(I2C) == 0x18) {
            /* ACK */
            /* Send address */
             I2C_SET_DATA(I2C, _addr & 0xFFUL); //  address
            I2C_SET_CONTROL_REG(I2C, I2C_SI);
            I2C_WAIT_READY(I2C){};


                if(I2C_GET_STATUS(I2C) == 0x28) {
                    /* ACK */

                    /* Send data */
                    I2C_SET_CONTROL_REG(I2C, I2C_STA | I2C_SI);
                    I2C_WAIT_READY(I2C);
                    if(I2C_GET_STATUS(I2C) == 0x10) {
                        /* ACK */

                        /* Send control byte */
                        I2C_SET_DATA(I2C, (_slave_addr | 0x01));
                        I2C_SET_CONTROL_REG(I2C, I2C_SI);
                        I2C_WAIT_READY(I2C);
                        if(I2C_GET_STATUS(I2C) == 0x40) {
                            I2C_SET_CONTROL_REG(I2C, I2C_SI);
                            I2C_WAIT_READY(I2C);

                            /* Read data */
                            u8Data = I2C_GET_DATA(I2C);
                            if(I2C_GET_STATUS(I2C) == 0x58) {
                                /* NACK */
                                /* Send stop */
                                I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI);
                            } else {
                                /* ACK */

                                /* read data error */
                                i32Err = 6;
                            }
                        } else {
                            /* NACK */

                            /* Send control read error */
                            i32Err = 5;
                        }
                    } else {
                        /* NACK */

                        /* Send start error */
                        i32Err = 4;
                    }
                } else {
                    /* NACK */

                    /* Send low address error */
                    i32Err = 3;
                }

        } else {
            /* NACK */

            /* Send control write error */
            i32Err = 1;

        }

        if(i32Err) {
#ifdef __DEVELOP__
					printf("Error: %d£¬ I2C status: %X\n", i32Err, I2C_GET_STATUS(I2C));
#endif
            /* Send stop */
            I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI);

            CLK_SysTickDelay(10);
        }

    } while(i32Err);

    return u8Data;
}

/**
 * @brief I2C read data from slave using sequential method
 * @param _slave_write_addr
 * @param _slave_read_addr
 * @param _addr
 * @param _buf
 * @param _size
 * @return
 */
uint32_t I2C_SequentialRead(uint8_t _slave_addr, uint32_t _addr, uint8_t *_buf, uint32_t _size)
{
    int32_t i32Err;
    int32_t i;

    do {
        i32Err = 0;

        /* Send start */
        I2C_SET_CONTROL_REG(I2C,  I2C_SI|I2C_STA);
        I2C_WAIT_READY(I2C);

        /* Send control byte */
        I2C_SET_DATA(I2C, _slave_addr);
        I2C_SET_CONTROL_REG(I2C, I2C_SI);
        I2C_WAIT_READY(I2C);
        if(I2C_GET_STATUS(I2C) == 0x18) {
            /* Send  address */
              I2C_SET_DATA(I2C, _addr & 0xFFUL); // address
            I2C_SET_CONTROL_REG(I2C, I2C_SI);
            I2C_WAIT_READY(I2C){};

                if(I2C_GET_STATUS(I2C) == 0x28) {
                    /* ACK */

                    /* Send data */
                    I2C_SET_CONTROL_REG(I2C, I2C_STA | I2C_SI);
                    I2C_WAIT_READY(I2C);
                    if(I2C_GET_STATUS(I2C) == 0x10) {
                        /* ACK */

                        /* Send control byte */
                        I2C_SET_DATA(I2C, (_slave_addr | 0x01));
                        I2C_SET_CONTROL_REG(I2C, I2C_SI);
                        I2C_WAIT_READY(I2C);
                        if(I2C_GET_STATUS(I2C) == 0x40) {
                            for(i=0; i<_size-1; i++) {
                                I2C_SET_CONTROL_REG(I2C, I2C_SI | I2C_AA);
                                I2C_WAIT_READY(I2C);

                                /* Read data */
                                _buf[i] = I2C_GET_DATA(I2C);
                            }

                            I2C_SET_CONTROL_REG(I2C, I2C_SI);
                            I2C_WAIT_READY(I2C);
                            _buf[i] = I2C_GET_DATA(I2C);

                            /* Send stop */
                            I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI);
                        } else {
                            /* NACK */

                            /* Send control read error */
                            i32Err = 5;
                        }
                    } else {
                        /* NACK */

                        /* Send start error */
                        i32Err = 4;
                    }
                } else {
                    /* NACK */

                    /* Send low address error */
                    i32Err = 3;
                }

        } else {
            /* NACK */

            /* Send control write error */
            i32Err = 1;

        }

        if(i32Err) {
            /* Send stop */
            I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI);

            CLK_SysTickDelay(100);
        }

    } while(i32Err);

    return _size;
}

/**
 * @brief I2C write a page data
 * @param _slave_write_addr
 * @param _addr
 * @param _buf
 */
void I2C_PageWrite(uint8_t _slave_addr, uint32_t _addr, uint8_t *_buf)
{
    int32_t i32Err;
    int32_t i;

    do {
        i32Err = 0;

        /* Send start */
         I2C_SET_CONTROL_REG(I2C,  I2C_SI|I2C_STA);
        I2C_WAIT_READY(I2C);

        /* Send control byte */
        I2C_SET_DATA(I2C, _slave_addr);
        I2C_SET_CONTROL_REG(I2C, I2C_SI);
        I2C_WAIT_READY(I2C);
        if(I2C_GET_STATUS(I2C) == 0x18) {
            /* Send  address */
              I2C_SET_DATA(I2C, _addr & 0xFFUL); // address
            I2C_SET_CONTROL_REG(I2C, I2C_SI);
            I2C_WAIT_READY(I2C){};

                if(I2C_GET_STATUS(I2C) == 0x28) {
                    /* ACK */

                    for(i=0; i<PAGE_SIZE; i++) {
                        /* Send data */
                        I2C_SET_DATA(I2C, _buf[i]); // data
                        I2C_SET_CONTROL_REG(I2C, I2C_SI);
                        I2C_WAIT_READY(I2C);
                        if(I2C_GET_STATUS(I2C) == 0x30) {
                            /* NACK */

                            /* Send data error */
                            i32Err = 4;
                        }
                    }

                    /* Send stop when no any error */
                    if(i32Err == 0) {
                        /* Send stop */
                        I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI);
                    }
                } else {
                    /* NACK */

                    /* Send low address error */
                    i32Err = 3;
                }
        } else {
            /* NACK */

            /* Send control error */
            i32Err = 1;
        }

        if(i32Err) {
            /* Send stop */
            I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI);

            CLK_SysTickDelay(100);
        }

    } while(i32Err);

}
