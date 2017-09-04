#ifndef __INERTIAL_SENSOR_H
#define __INERTIAL_SENSOR_H

#include "Algorithm_filter.h"

#define SENSOR_BMI160 0

#define SENSOR_TYPE SENSOR_BMI160

#if SENSOR_TYPE == SENSOR_BMI160
#define IMU_SENSOR_X_FACTOR  -1
#define IMU_SENSOR_Y_FACTOR  1
#define IMU_SENSOR_Z_FACTOR  -1
#endif

/*----------------ÍÓÂÝÒÇ²É¼¯ÏÞ·ù--------------------*/
#define GYRO_GATHER   100
#define FILTER_LPF2P 0
#define FILTER_IIR_I 1
#define SENSOR_FILTER FILTER_LPF2P

#define IMU_SAMPLE_RATE 100.0f
#define IMU_FILTER_CUTOFF_FREQ 30.0f

#define IMU_ACC_RAW_RANGE 8192

typedef struct {
    float x;
    float y;
    float z;
}_Vector_Float; 

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
}_Vector_Int16;

typedef struct {
    LPF2p_t x;
    LPF2p_t y;
    LPF2p_t z;
}_Vector_LPF2p;

typedef struct {
    _Vector_Int16 latest;
    _Vector_Int16 average;
    _Vector_Int16 relative;
    _Vector_Float filter;
    _Vector_Float history;
    _Vector_Int16 quiet;
	_Vector_LPF2p flit_lpf2p;
}_Vector_Packet;

typedef struct {
    _Vector_Packet accel;
    _Vector_Packet gyro;
}Inertial_Sensor;

void inertial_sensor_init(void); 
void inertial_sensor_read(void);
void gyro_caloffest(float x,float y,float z,uint16_t amount);
void gyro_offset(void);
void accel_offset(void);
_Vector_Float get_inertial_vel(void);


extern Inertial_Sensor inertial_sensor;

#endif
