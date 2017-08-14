#ifndef __ALGORITHM_PID_H
#define __ALGORITHM_PID_H

#define PID_FILT_MIN_HZ

typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float imax;
    float derivative;
    float dt;
    float filt_hz;
    float input;
    float P_Item_Output;
    float I_Item_Output;
    float D_Item_Output;
}Pid_t;

/* function prototype */
void set_pid_param(Pid_t *_pid, float _kp, float _ki, float _kd, float _imax, float _filt_hz, float dt);
void set_pid_filt_hz(Pid_t *_pid, float _hz);
void set_pid_input(Pid_t *_pid, float _input);
void set_input_filter_all(Pid_t *_pid, float _input);
void get_pid_output(Pid_t *_pid);
void reset_pid_I(Pid_t *_pid);
void reset_pid_D(Pid_t *_pid);
float get_integrator(Pid_t *_pid);
// calc_filt_alpha - recalculate the input filter alpha
float get_filt_alpha(Pid_t _pid);

#endif
