#include "PN020Series.h"
#include "bmi160.h"

/* variance define */
float _accel_scale;
float _gyro_scale;

struct RawData {
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    }accel;
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    }gyro;
};

/**
 * @brief bmi160 init
 */
bool bmi160_init()
{
    bool ret = false;
    uint8_t tmp;
    uint8_t tries_times = 0;


    // delay_ms(BMI160_POWERUP_DELAY_MSEC);
    for (; tries_times < BMI160_HARDWARE_INIT_MAX_TRIES; i++) {

        /* reset bmi160 */
        I2C_WriteByte(BMI160_SLAVE_ADDRESS, BMI160_REG_CMD, BMI160_CMD_SOFTRESET);
        delay_ms(BMI160_SOFTRESET_DELAY_MSEC);

        /* read chip ID */
        tmp = I2C_ReadByte(BMI160_SLAVE_ADDRESS, BMI160_REG_CHIPID);;
        if (tmp != BMI160_CHIPID) {
            continue;
        }

        /* set accel and gyro work on normal mode */
        I2C_WriteByte(BMI160_SLAVE_ADDRESS, BMI160_REG_CMD, BMI160_CMD_ACCEL_NORMAL_POWER_MODE);
        delay_ms(BMI160_CMD_ACCEL_NORMAL_POWER_MODE_DELAY_MSEC);

        I2C_WriteByte(BMI160_SLAVE_ADDRESS, BMI160_REG_CMD, BMI160_CMD_GYRO_NORMAL_POWER_MODE);
        delay_ms(BMI160_CMD_GYRO_NORMAL_POWER_MODE_DELAY_MSEC);

        if (!configure_accel()) {
            continue;
        }

        if (!configure_gyro()) {
            continue;
        }

        if (!configure_fifo()) {
            continue;
        }

        ret = true;
        break;
    }

    return ret;
}

/**
 * @brief read vehicle origin accel and gyro from bmi160
 * @param _acc
 * @param _gyro
 */
void bmi160_read_raw(int16_t _acc[], int16_t _gyro[])
{

}

bool configure_accel()
{
    // bool ret = false;

    I2C_WriteByte(BMI160_SLAVE_ADDRESS, BMI160_REG_ACC_CONF, BMI160_OSR | BMI160_ODR);
    delay_ms(1);

    I2C_WriteByte(BMI160_SLAVE_ADDRESS, BMI160_REG_ACC_RANGE, BMI160_ACC_RANGE_BITS);
    delay_ms(1);
    r = _dev->write_register(BMI160_REG_ACC_RANGE, BMI160_ACC_RANGE_BITS);

    _accel_scale = GRAVITY_MSS / (1 << (14 - BMI160_ACC_RANGE));

    return true;
}

bool configure_gyro()
{
    // bool ret = false;

    I2C_WriteByte(BMI160_SLAVE_ADDRESS, BMI160_REG_GYR_CONF, BMI160_OSR | BMI160_ODR);
    delay_ms(1);

    I2C_WriteByte(BMI160_SLAVE_ADDRESS, BMI160_REG_GYR_RANGE, BMI160_GYR_RANGE);
    delay_ms(1);

    /* The sensitivity in LSb/degrees/s a gyro range i can be calculated with:
     *     2 ^ 16 / (2 * 2000 / 2 ^ i) = 2 ^ (14 + i) / 1000
     * The scale is the inverse of that. */
    _gyro_scale = radians(1000.f / (1 << (14 + BMI160_GYR_RANGE)));

    return true;
}


bool configure_fifo()
{
    // bool ret = false;

    I2C_WriteByte(BMI160_SLAVE_ADDRESS, BMI160_REG_FIFO_CONFIG_1, BMI160_FIFO_ACC_EN | BMI160_FIFO_GYR_EN);
    delay_ms(1);

    I2C_WriteByte(BMI160_SLAVE_ADDRESS, BMI160_REG_CMD, BMI160_CMD_FIFO_FLUSH);

    return true;
}

bool read_fifo()
{
    struct RawData raw_data[BMI160_MAX_FIFO_SAMPLES];
    uint8_t buf[BMI160_MAX_FIFO_FRAME];
    uint16_t num_bytes;
    uint8_t num_byte[2] = {0};
    uint16_t excess;
    uint8_t num_samples = 0;
    uint8_t sample_index = 0;
    uint16_t buf_index = 0;
    bool ret = true;

    I2C_SequentialRead(BMI160_SLAVE_ADDRESS, BMI160_REG_FIFO_LENGTH, num_byte, sizeof(num_bytes));
    num_bytes = (num_byte[1] << 8) | num_byte[0];

    if (!num_bytes) {
        ret = false;
        goto read_fifo_end;
    }

read_fifo_read_data:
    if (num_bytes > sizeof(raw_data)) {
        excess = num_bytes - sizeof(raw_data);
        num_bytes = sizeof(raw_data);
    } else {
        excess = 0;
    }

    I2C_SequentialRead(BMI160_SLAVE_ADDRESS, BMI160_REG_FIFO_DATA, buf, num_bytes);

    /* Read again just once */
    if (excess && num_samples) {
//        hal.console->printf("BMI160: dropping %u samples from fifo\n",
//                            (uint8_t)(excess / sizeof(struct RawData)));
        I2C_WriteByte(BMI160_SLAVE_ADDRESS, BMI160_REG_CMD, BMI160_CMD_FIFO_FLUSH);
        excess = 0;
    }

    num_samples = num_bytes / sizeof(struct RawData);
    for (; sample_index < num_samples; sample_index++) {
        raw_data[i].accel.x = (buf[buf_index + 1] << 8) | buf[buf_index + 0];
        raw_data[i].accel.y = (buf[buf_index + 3] << 8) | buf[buf_index + 2];
        raw_data[i].accel.z = (buf[buf_index + 5] << 8) | buf[buf_index + 4];
        raw_data[i].accel.x = (buf[buf_index + 7] << 8) | buf[buf_index + 6];
        raw_data[i].accel.x = (buf[buf_index + 9] << 8) | buf[buf_index + 8];
        raw_data[i].accel.x = (buf[buf_index + 11] << 8) | buf[buf_index + 10];


        buf_index += 12;
//        raw_data *= _accel_scale;
//        gyro *= _gyro_scale;
    }

    if (excess) {
        num_bytes = excess;
        goto read_fifo_read_data;
    }

read_fifo_end:
//    if (!ret) {
//        hal.console->printf("BMI160: error on reading FIFO\n");
//    }
    return ret;
}
