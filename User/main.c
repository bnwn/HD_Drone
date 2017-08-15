#include "PN020Series.h"
#include "common.h"

int main(void)
{
    system_init();

	  PWM_ConfigOutputChannel(PWM, 0, MOTOR_PWM_FREQ, 10);
    PWM_ConfigOutputChannel(PWM, 1, MOTOR_PWM_FREQ, 10);
    PWM_ConfigOutputChannel(PWM, 2, MOTOR_PWM_FREQ, 10);
    PWM_ConfigOutputChannel(PWM, 3, MOTOR_PWM_FREQ, 10);
		PWM_ConfigOutputChannel(PWM, 7, MOTOR_PWM_FREQ, 10);
    PWM_EnableOutput(PWM, 0xFF);
    PWM_Start(PWM, 0xFF);
	
    peripheral_init();

    printf("HD_Drone init success!\n");

    while(1) {
        /* low priority task scheduler */
        scheduler_run();
    }

	  return 0;
}
