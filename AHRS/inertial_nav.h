#ifndef __INERTIAL_NAV_H
#define __INERTIAL_NAV_H

#include "common.h"
#include "inertial_sensor.h"

typedef struct  {
    float x;
    float y;
    float z; // altitude (uint:m)
    float vx;
    float vy;
    float vz;
    float ax;
    float ay;
    float az;
} _Nav_t;

void inertial_nav_init(void);
void inertial_nav_update(void);
void position_z_update();
float get_nav_altitude(void);
void update_home_pos(void);
//Combine Filter to correct err
static void inertial_filter_predict(float dt, float *x, float acc);
static void inertial_filter_correct(float e, float dt, float *x, int i, float w);
float get_inav_alt(void);
_Vector_Float get_inav_velocity(void);
_Vector_Float get_inav_accel(void);

extern _Nav_t nav;
extern _Vector_Float home_absolute_pos;

#endif

