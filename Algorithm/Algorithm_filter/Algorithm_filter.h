#ifndef __Algorithm_filter_H
#define	__Algorithm_filter_H

typedef struct {
	float cutoff_freq;
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;
	float delay_element_1;        // buffered sample -1
	float delay_element_2;        // buffered sample -2
} LPF2p_t;

void LPF2pSetCutoffFreq(LPF2p_t *_lpf2p, float _sample_freq, float _cutoff_freq);
float LPF2pApply(LPF2p_t *_lpf2p, float _sample);
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
