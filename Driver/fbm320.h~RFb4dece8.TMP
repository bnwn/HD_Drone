#ifndef __FBM320_H
#define __FBM320_H

#include "bsp/bsp_i2c.h"
#include "bsp/timer_delay.h"

/* Register define */
#define FBM320_SLAVE_ADDRESS 0xD8
#define FBM320_REG_CHIPID 0x6B
#define FBM320_CHIPID 0x42
#define FBM320_REG_CMD 0xF4
#define FBM320_REG_COEFF_A 0xAA
#define FBM320_REG_COEFF_D 0xD0
#define FBM320_REG_COEFF_F 0xF1
#define FBM320_REG_SOFTRESET 0xE0
#define FBM320_REG_DATA_LSB 0xF8
#define FBM320_REG_DATA_CSB 0xF7
#define FBM320_REG_DATA_MSB 0xF6

/* Pressure osr define */
#define FBM320_OSR_8192 0xC0
#define FBM320_OSR_4096 0x80
#define FBM320_OSR_2048 0x40
#define FBM320_OSR_1024 0x00
#define FBM320_PRESSURE_OSR 0xF4 // FBM320_READ_PRESSURE | FBM320_OSR_8192

/* CMD */
#define FBM320_READ_TEMPERATURE 0x2E
#define FBM320_READ_PRESSURE FBM320_PRESSURE_OSR

/* function prototype */
/**
 * @brief fbm320 init
 * @return
 */
bool fbm320_init(void);

/**
 * @brief read data register from fbm320
 * @return 3 byte data
 */
int32_t fbm320_read_long_data(void);

/**
 * @brief timer procedure, call in 100us
 */
void fbm320_timer_procedure(void);

void coefficient(void);

void calculate(int32_t UP, int32_t UT);		//Calculate Real Pressure & Temperautre

int32_t abs_altitude(int32_t Press); 	//Calculate absolute altitude, unit: mm

/* variance define */
typedef struct FMTI_Sensor
{
    uint8_t Version;																					//Sensor version
    uint8_t RPC;
    int32_t UP;
    int32_t UT;
    int32_t RP;																								//Unit: Pa
    int32_t RT;																								//Unit: 0.01 degree
    int16_t Altitude;																						//Unit: centi meter
    uint16_t C0, C1, C2, C3, C6, C8, C9, C10, C11, C12;
    uint32_t C4, C5, C7;
}FMTI_Sensor;

extern FMTI_Sensor fbm320_packet;
#endif
