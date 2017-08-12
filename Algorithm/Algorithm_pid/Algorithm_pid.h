#ifndef __ALGORITHM_PID_H
#define __ALGORITHM_PID_H

typedef struct {
		float kp;
		float ki;
		float kd;
		float integrator;
		float imax;
		float derivative;
		float dt;
		float P_Item_Output;
		float I_Iten_Output;
		float D_Item_Output;
}Pid_t;

#endif
