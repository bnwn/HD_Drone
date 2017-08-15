#include "PN020Series.h"
#include "Algorithm_pid.h"
#include "../Algorithm_math/Algorithm_math.h"
#include "../Algorithm_math/mymath.h"

void set_pid_param(Pid_t *_pid, float _kp, float _ki, float _kd, float _imax, float _filt_hz, float dt)
{
    _pid->kp = _kp;
    _pid->ki = _ki;
    _pid->kd = _kd;
    _pid->imax = _imax;
    _pid->dt = dt;
    _pid->integrator = 0;
    _pid->derivative = 0;

    set_pid_filt_hz(_pid, _filt_hz);
}

// filt_hz - set input filter hz
void set_pid_filt_hz(Pid_t *_pid, float _hz)
{
    _pid->filt_hz = _hz;

    // sanity check _filt_hz
    _pid->filt_hz = MAX(_pid->filt_hz, PID_FILT_MIN_HZ);
}

void set_pid_input(Pid_t *_pid, float _input)
{
    // update filter and calculate derivative
    if (_pid->dt > 0.0f) {
        float derivative = (_input - _pid->input) / _pid->dt;
        _pid->derivative = _pid->derivative + get_filt_alpha(_pid) * (derivative - _pid->derivative);
    }

    _pid->input = _input;

    get_pid_output(_pid);
}

// set_input_filter_all - set input to PID controller
//  input is filtered before the PID controllers are run
//  this should be called before any other calls to get_p, get_i or get_d
void set_input_filter_all(Pid_t *_pid, float _input)
{
    // update filter and calculate derivative
    float input_filt_change = get_filt_alpha(_pid) * (_input - _pid->input);
    _pid->input = _pid->input +input_filt_change;
    if (_pid->dt > 0.0f) {
        _pid->derivative = input_filt_change / _pid->dt;
    }
    _pid->input = _input;

    get_pid_output(_pid);
}

void get_pid_output(Pid_t *_pid)
{
    _pid->P_Item_Output = _pid->kp * _pid->input;

    if((_pid->ki != 0) && (_pid->dt != 0)) {
        _pid->integrator += ((float)_pid->input * _pid->ki) * _pid->dt;
        if (_pid->integrator < -_pid->imax) {
            _pid->integrator = -_pid->imax;
        } else if (_pid->integrator > _pid->imax) {
            _pid->integrator = _pid->imax;
        }
        _pid->I_Item_Output = _pid->integrator;
    }

    // derivative component
    _pid->D_Item_Output = _pid->kd * _pid->derivative;
}

void reset_pid_I(Pid_t *_pid)
{
    _pid->integrator = 0;
}

void reset_pid_D(Pid_t *_pid)
{
    _pid->derivative = 0;
}

float get_integrator(Pid_t *_pid)
{
    return (_pid->integrator);
}

// calc_filt_alpha - recalculate the input filter alpha
float get_filt_alpha(Pid_t *_pid)
{
    float rc;
    if (_pid->filt_hz == 0.0f) {
        return 1.0f;
    }

    // calculate alpha
    rc = 1 / (M_2PI * _pid->filt_hz);
    return (float)(_pid->dt / (_pid->dt + rc));
}
