#include "PN020Series.h"
#include "common.h"

int main()
{
    system_init();

    peripheral_init();

    printf("HD_Drone init success!\n");

    while(1) {
        /* low priority task scheduler */
        scheduler_run();
    }

	return 0;
}
