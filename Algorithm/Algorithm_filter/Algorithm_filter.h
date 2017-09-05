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
float Moving_Average(float _filter_arr[], uint8_t _width, float _data);
#endif /* __Algorithm_filter_H */
