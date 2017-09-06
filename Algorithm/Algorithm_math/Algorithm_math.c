#include "PN020Series.h"
#include "Algorithm_math.h"

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Q_rsqrt
**功能 : 快速计算 1/Sqrt(x) 
**输入 : number  
**输出 : 结果
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration （第一次牛顿迭代）
	return y;
} 

/***********************************************
  * @brief  可变增益自适应参数
  * @param  None
  * @retval None
************************************************/
float VariableParameter(float error)
{	
	float  result = 0;
	return 1.0f;
	if(error < 0.0f)
	{
	   error = -error;
	}
  if(error >0.6f)
	{
	   error = 0.6f;
	}
	result = 1 - 1.667 * error;
	if(result < 0)
	{
	   result = 0;
	}
	return result;
}
/**************************实现函数********************************************
*函数原型:    rad(double angle)
*功　　能:    角度转化为弧度
输入参数：    角度
输出参数：    弧度
*******************************************************************************/
float Rad(float angle)
{
    return (angle * M_PI / 180.0);
}
/**************************实现函数********************************************
*函数原型:    degree(double rad)
*功　　能:    弧度转化为角度	
输入参数：    弧度
输出参数：    角度
*******************************************************************************/
float Degree(float rad)
{
    return (rad / M_PI * 180.0);
}

float wrap_360_degree(float _angle)
{
    float ret = fmod(_angle, 360.0f);

    if (ret < 0) {
        ret += 360.0f;
    }

    return ret;
}

double constrain(double inputvalue, double limitmin, double limitmax)
{
    if (inputvalue>limitmax) {
        inputvalue=limitmax;
    }
    else
    if (inputvalue<limitmin) {
        inputvalue=limitmin;
    }
		return inputvalue;
}

float constrain_float(float inputvalue, float limitmin, float limitmax)
{
    if (inputvalue>limitmax) {
        inputvalue=limitmax;
    }
    else
    if (inputvalue<limitmin) {
        inputvalue=limitmin;
    }
        return inputvalue;
}
