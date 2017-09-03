#ifndef __COMMON_H
#define __COMMON_H

/* only develop debug output info through uart */
#define __DEVELOP__

#define LOOP_DT 0.01f

void system_init(void);

void peripheral_init(void);

void param_load(void);

void fc_status_reset(void);

enum Armed_t{
	ARMED = 0,
	IDLED = 1,
	DISARMED = 2,
	MOTOR_TEST = 3
};

/* velhicle status */
typedef struct {
	enum Armed_t armed;
	bool land_complete;
	bool code_matched;
    bool altitude_updated;
    bool accel_updated;
	uint8_t printf_flag;
	uint16_t motor_control_Hz;
}_Status_t;

extern _Status_t fc_status;

#endif
