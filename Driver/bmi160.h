#ifndef __BMI160_H
#define __BMI160_H

#include "bsp/bsp_i2c.h"
#include "bsp/timer_delay.h"
#include "../AHRS/inertial_sensor.h"

#define BMI160_SLAVE_ADDRESS 0xD0 // SDO connect to GND, slave address is 0x68
//#define BMI160_SLAVE_ADDRESS // 0x69

/* Registers and bits definitions. The indented ones are the bits for the upper
 * register. */
#define BMI160_REG_CHIPID 0x00
#define BMI160_CHIPID 0xD1
#define BMI160_REG_ERR_REG 0x02
#define BMI160_REG_FIFO_LENGTH 0x22
#define BMI160_FIFO_MSB_MASK 0x07
#define BMI160_REG_FIFO_DATA 0x24
#define BMI160_REG_ACC_CONF 0x40
#define BMI160_REG_ACC_RANGE 0x41
            /* For convenience, use log2(range) - 1 instead of bits defined in
             * the datasheet. See _configure_accel(). */
#define BMI160_ACC_RANGE_16G 3
#define BMI160_ACC_RANGE_4G 1
#define BMI160_REG_GYR_CONF 0x42
#define BMI160_REG_GYR_RANGE 0x43
#define BMI160_GYR_RANGE_2000DPS 0x00
#define BMI160_GYR_RANGE_1000DPS 0x01
#define BMI160_REG_FIFO_CONFIG_0 0x46
#define BMI160_REG_FIFO_CONFIG_1 0x47
#define BMI160_FIFO_ACC_EN 0x40
#define BMI160_FIFO_GYR_EN 0x80
#define BMI160_REG_INT_EN_1 0x51
#define BMI160_INT_FWM_EN 0x40
#define BMI160_REG_INT_OUT_CTRL 0x53
#define BMI160_INT1_LVL 0x02
#define BMI160_INT1_OUTPUT_EN 0x08
#define BMI160_REG_INT_MAP_1 0x56
#define BMI160_INT_MAP_INT1_FWM 0x40
#define BMI160_REG_CMD 0x7E
#define BMI160_CMD_ACCEL_NORMAL_POWER_MODE 0x11
#define BMI160_CMD_GYRO_NORMAL_POWER_MODE 0x15
#define BMI160_CMD_FIFO_FLUSH 0xB0
#define BMI160_CMD_SOFTRESET 0xB6

#define BMI160_OSR_NORMAL 0x20
#define BMI160_ODR_1600HZ 0x0C

/* Datasheet says that the device powers up in less than 10ms, so waiting for
 * 10 ms before initialization is enough. */
#define BMI160_POWERUP_DELAY_MSEC 10
/* TODO: Investigate this. The delay below is way too high and with that
 * there's still at least 1% of failures on initialization. Lower values
 * increase that percentage. */
#define BMI160_SOFTRESET_DELAY_MSEC 100
/* Define a little bit more than the maximum value in the datasheet's timing
 * table. The datasheet recommends adding 300 us to the time for startup
 * occasions. */
#define BMI160_ACCEL_NORMAL_POWER_MODE_DELAY_MSEC 4
#define BMI160_GYRO_NORMAL_POWER_MODE_DELAY_MSEC 81

#define BMI160_OSR BMI160_OSR_NORMAL
#define BMI160_ODR BMI160_ODR_1600HZ
#define BMI160_ACC_RANGE BMI160_ACC_RANGE_4G
#define BMI160_GYR_RANGE BMI160_GYR_RANGE_1000DPS

/* By looking at the datasheet, the accel range i (as defined by the macros
 * BMI160_ACC_RANGE_*G) maps to the range bits by the function f defined:
 *     f(0) = 3; f(i) = f(i - 1) + i + 1
 * Which can be written as the closed formula:
 *     f(i) = (i * (i + 3)) / 2 + 3 */
#define BMI160_ACC_RANGE_BITS \
    (BMI160_ACC_RANGE * (BMI160_ACC_RANGE + 3) / 2 + 3)

/* The rate in Hz based on the ODR bits can be calculated with
 * 100 / (2 ^ (8 - odr) */
#define BMI160_ODR_TO_HZ(odr_) \
    (uint16_t)(odr_ > 8 ? 100 << (odr_ - 8) : 100 >> (8 - odr_))

/* This number of samples should provide only one read burst operation on the
 * FIFO most of the time (99.99%). */
#define BMI160_MAX_FIFO_SAMPLES 20
#define BMI160_MAX_FIFO_FRAME 256

#define BMI160_READ_FLAG 0x80
#define BMI160_HARDWARE_INIT_MAX_TRIES 5

// acceleration due to gravity in m/s/s
#define GRAVITY_MSS     9.80665f

/**
 * @brief bmi160 configuration
 * @return
 */
bool bmi160_init(void);

/**
 * @brief read accel and gyro raw from bmi160
 * @param _acc
 * @param _gyro
 */
void bmi160_read_raw(Inertial_Sensor *_sensor);

/**
 * Configure accelerometer sensor. The device semaphore must already be
 * taken before calling this function.
 *
 * @return true on success, false otherwise.
 */
bool configure_accel(void);

/**
 * Configure gyroscope sensor. The device semaphore must already be
 * taken before calling this function.
 *
 * @return true on success, false otherwise.
 */
bool configure_gyro(void);

/**
 * Configure FIFO.
 *
 * @return true on success, false otherwise.
 */
bool configure_fifo(void);

/**
 * Read samples from fifo.
 */
bool read_fifo(Inertial_Sensor *_sensor);

/* variance define */
extern float _accel_scale;
extern float _gyro_scale;

#endif
