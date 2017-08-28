#ifndef __COMMON_H
#define __COMMON_H

/* only develop debug output info through uart */
#define __DEVELOP__

#define LOOP_DT 0.01f

void system_init(void);

void peripheral_init(void);

void param_load(void);

/* velhicle status */
typedef struct {
	bool armed;
}_Status_t;

extern _Status_t fc_status;

#endif
