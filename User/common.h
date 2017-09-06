#ifndef __COMMON_H
#define __COMMON_H

/* only develop debug output info through uart */
#define __DEVELOP__

#define LOOP_DT 0.01f

void system_init(void);

void peripheral_init(void);

void param_load(void);

void fc_status_reset(void);

void set_land_complete(bool _b);

void set_radio_lost(bool _b);

uint32_t systick_config(uint32_t ticks);
uint32_t sys_micro(void);
uint32_t sys_milli(void);

enum Armed_t{
    DISARMED = 0,
	IDLED = 1,
    ARMED = 2,
	MOTOR_TEST = 3
};

enum Simple_Mode_t {
    Normal = 0,
    Simple = 1,
    Super_Simple = 2
};

/* velhicle status */
typedef struct {
	enum Armed_t armed;
    enum Simple_Mode_t simple_mode;
	bool land_complete;
	bool code_matched;
    bool radio_lost;
    bool altitude_updated;
	bool home_abs_alt_updated;
    bool accel_updated;
    bool baro_collect_ok;
    bool inav_z_estimate_ok;
	bool baro_initialize;
	uint8_t printf_flag;
	uint16_t motor_control_Hz;
}_Status_t;

extern _Status_t fc_status;

#endif
