#ifndef __Algorithm_filter_H
#define	__Algorithm_filter_H

double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na);
float LPF_1st(float oldData, float newData, float lpf_factor);
#endif /* __Algorithm_filter_H */

#ifndef __FILTER_H
#define __FILTER_H


//float Moving_Average(uint8_t item,uint8_t width_num,float in);
void Moving_Average(float in, float moavarray[], uint16_t len, uint16_t fil_cnt[2], float *out);
float Moving_Median(uint8_t item, uint8_t width_num, float in);
float kalmanUpdate(const float gyro_m,const float incAngle);

#endif
