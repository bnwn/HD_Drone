#ifndef __INERTIAL_SENSOR_H
#define __INERTIAL_SENSOR_H

 #define SENSOR_BMI160 0

#define SENSOR_TYPE SENSOR_BMI160

/*----------------ÍÓÂÝÒÇ²É¼¯ÏÞ·ù--------------------*/
#define GYRO_GATHER   70

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
    _Vector_Int16 latest;
    _Vector_Int16 average;
    _Vector_Int16 relative;
    _Vector_Float filter;
    _Vector_Float history;
    _Vector_Float radian;
    _Vector_Int16 quiet;
}_Vector_Packet;

typedef struct {
    _Vector_Packet accel;
    _Vector_Packet gyro;
}Inertial_Sensor;

void inertial_sensor_read(void);
void gyro_caloffest(float x,float y,float z,uint16_t amount);
void gyro_offset(void);
void accel_offset(void);
_Vector_Float get_inertial_vel(void);


extern Inertial_Sensor inertial_sensor;

#endif
