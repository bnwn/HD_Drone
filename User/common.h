#ifndef __COMMON_H
#define __COMMON_H

/* only develop debug output info through uart */
#define __DEBUG__

void system_init(void);

void peripheral_init(void);

void param_load(void);

#endif
