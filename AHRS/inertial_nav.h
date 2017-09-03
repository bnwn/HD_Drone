#ifndef __INERTIAL_NAV_H
#define __INERTIAL_NAV_H

#include "common.h"
#include "inertial_sensor.h"

typedef struct  {
    float x;
    float y;
    float z; // altitude (uint:cm)
    float vx;
    float vy;
    float vz;
    float ax;
    float ay;
    float az;
} _Nav_t;

void inertial_nav_update(void);
void position_z_update(float dt);
float get_nav_altitude(void);
//Combine Filter to correct err
static void inertial_filter_predict(float dt, float x[3]);
static void inertial_filter_correct(float e, float dt, float x[3], int i, float w);

extern _Nav_t nav;
extern _Vector_Float home_absolute_pos;

#endif
