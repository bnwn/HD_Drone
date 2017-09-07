#include "PN020Series.h"
#include "Algorithm_filter.h"
#include "Algorithm_math.h"
#include "math.h"

void LPF2pSetCutoffFreq(LPF2p_t *_lpf2p, float _sample_freq, float _cutoff_freq)
{
    float fr =0;
    float ohm =0;
    float c =0;

    fr= _sample_freq/_cutoff_freq;
    ohm=tanf(M_PI/fr);
    c=1.0f+2.0f*cosf(M_PI/4.0f)*ohm + ohm*ohm;

    _lpf2p->cutoff_freq = _cutoff_freq;
    if (_lpf2p->cutoff_freq > 0.0f)
    {
        _lpf2p->b0 = ohm*ohm/c;
        _lpf2p->b1 = 2.0f*_lpf2p->b0;
        _lpf2p->b2 = _lpf2p->b0;
        _lpf2p->a1 = 2.0f*(ohm*ohm-1.0f)/c;
        _lpf2p->a2 = (1.0f-2.0f*cosf(M_PI/4.0f)*ohm+ohm*ohm)/c;
    }
	_lpf2p->delay_element_1 = 0.0f;
	_lpf2p->delay_element_2 = 0.0f;
}

float LPF2pApply(LPF2p_t *_lpf2p, float _sample)
{

    float delay_element_0 = 0, output=0;
    if (_lpf2p->cutoff_freq <= 0.0f) {
        // no filtering
        return _sample;
    }
    else
    {
        delay_element_0 = _sample - _lpf2p->delay_element_1 * _lpf2p->a1 -_lpf2p->delay_element_2 * _lpf2p->a2;
        // do the filtering
        if (isnan(delay_element_0) || isinf(delay_element_0)) {
            // don't allow bad values to propogate via the filter
            delay_element_0 = _sample;
        }
        output = delay_element_0 * _lpf2p->b0 + _lpf2p->delay_element_1 * _lpf2p->b1 + _lpf2p->delay_element_2 * _lpf2p->b2;

        _lpf2p->delay_element_2 = _lpf2p->delay_element_1;
        _lpf2p->delay_element_1 = delay_element_0;

        // return the value.  Should be no need to check limits
        return output;
    }
}


/*====================================================================================================*/
/*====================================================================================================*
** 函数名称: IIR_I_Filter
** 功能描述: IIR直接I型滤波器
** 输    入: InData 为当前数据
**           *x     储存未滤波的数据
**           *y     储存滤波后的数据
**           *b     储存系数b
**           *a     储存系数a
**           nb     数组*b的长度
**           na     数组*a的长度
**           LpfFactor
** 输    出: OutData         
** 说    明: 无
** 函数原型: y(n) = b0*x(n) + b1*x(n-1) + b2*x(n-2) -
                    a1*y(n-1) - a2*y(n-2)
**====================================================================================================*/
/*====================================================================================================*/
//double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na)
//{
//  double z1,z2;
//  short i;
//  double OutData;
//  
//  for(i=nb-1; i>0; i--)
//  {
//    x[i]=x[i-1];
//  }
//  
//  x[0] = InData;
//  
//  for(z1=0,i=0; i<nb; i++)
//  {
//    z1 += x[i]*b[i];
//  }
//  
//  for(i=na-1; i>0; i--)
//  {
//    y[i]=y[i-1];
//  }
//  
//  for(z2=0,i=1; i<na; i++)
//  {
//    z2 += y[i]*a[i];
//  }
//  
//  y[0] = z1 - z2; 
//  OutData = y[0];
//    
//  return OutData;
//}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : LPF_1st
**功能 : 一阶低通滤波
**输入 :  
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
float LPF_1st(float oldData, float newData, float lpf_factor)
{
	return oldData * (1 - lpf_factor) + newData * lpf_factor;
}


float Moving_Average(float _filter_arr[], uint8_t _width, float _data, bool _extre)
{
	uint8_t i;
	float sum = 0.0f;
	float max_data = _data, min_data = _data;
	
	for (i=1; i<_width; i++) {
		if (_filter_arr[i] > max_data) {
			max_data = _filter_arr[i];
		} else if (_filter_arr[i] < min_data) {
			min_data = _filter_arr[i];
		}
		
		sum += _filter_arr[i];
		_filter_arr[i-1] = _filter_arr[i];
	}
	_filter_arr[_width-1] = _data;
	sum += _data;
	
	if (_extre)
		return (float)((sum-max_data-min_data)/(_width-2));
	else 
		return (float)(sum/_width);
}

