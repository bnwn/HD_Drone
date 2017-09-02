#ifndef __INERTIAL_NAV_H
#define __INERTIAL_NAV_H

#include "common.h"

typedef struct  {
    float x;
    float y;
    float z; // altitude (uint:cm)
} Position;

void inertial_nav_update(void);
void position_z_update(void);
float get_nav_altitude(void);

extern Position nav_pos;
extern Position nav_pos_vel;
extern Position home_absolute_pos;

#endif
