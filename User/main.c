#include "PN020Series.h"
#include "common.h"
#include "scheduler.h"

int main(void)
{
    system_init();
	
	fc_status_reset();
	
    peripheral_init();

    SCHEDULER_RUN;
	
    while(1) {
        /* task scheduler */
        scheduler_run();
    }

	  return 0;
}
