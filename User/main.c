#include "PN020Series.h"
#include "common.h"

int main(void)
{
    system_init();
	
    peripheral_init();

    printf("HD_Drone init success!\n");
	
		printf("sensor collect offset...\n");
		gyro_offset();
		accel_offset();
		printf("collect complete\n");
		SCHEDULER_RUN;	
	
    while(1) {
        /* task scheduler */
        scheduler_run();
    }

	  return 0;
}
