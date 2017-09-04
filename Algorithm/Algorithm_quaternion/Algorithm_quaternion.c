#include "PN020Series.h"
#include "Algorithm_quaternion.h"
#include "../Algorithm_math/Algorithm_math.h"
#include "../AHRS/inertial_sensor.h"

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Quaternion_vectorGravity
**功能 : 提取等效旋转矩阵中的重力分量 
**输入 : *pNumQ
**输出 : 重力分量
**使用 : Quaternion_vectorGravity( Quaternion *pNumQ );
**====================================================================================================*/
/*====================================================================================================*/
_Vector_Float Quaternion_vectorGravity( Quaternion *pNumQ )
{
	_Vector_Float G;
  G.x = 2*(pNumQ->q1*pNumQ->q3 - pNumQ->q0*pNumQ->q2);								
  G.y = 2*(pNumQ->q0*pNumQ->q1 + pNumQ->q2*pNumQ->q3);						  
  G.z = 1-2*(pNumQ->q1*pNumQ->q1 + pNumQ->q2*pNumQ->q2);
	
	return G;
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : Quaternion_ToNumQ
**功能 : 欧拉角转四元数
**输入 : *pNumQ, *pAngE
**输出 : None
**使用 : Quaternion_ToNumQ(&NumQ, &AngE);
**====================================================================================================*/
/*====================================================================================================*/
void Quaternion_ToNumQ( Quaternion *pNumQ, EulerAngle *pAngE )
{
  float halfP = pAngE->Pitch/2.0f;
  float halfR = pAngE->Roll/2.0f;
  float halfY = pAngE->Yaw/2.0f;

#ifndef USE_ARM_MATH_LIB
  float sinP = sinf(halfP);
  float cosP = cosf(halfP);
  float sinR = sinf(halfR);
  float cosR = cosf(halfR);
  float sinY = sinf(halfY);
  float cosY = cosf(halfY);
#else
  float sinP = arm_sin_f32(halfP);
  float cosP = arm_cos_f32(halfP);
  float sinR = arm_sin_f32(halfR);
  float cosR = arm_cos_f32(halfR);
  float sinY = arm_sin_f32(halfY);
  float cosY = arm_cos_f32(halfY);
#endif

  pNumQ->q0 = cosY*cosR*cosP + sinY*sinR*sinP;
  pNumQ->q1 = cosY*cosR*sinP - sinY*sinR*cosP;
  pNumQ->q2 = cosY*sinR*cosP + sinY*cosR*sinP;
  pNumQ->q3 = sinY*cosR*cosP - cosY*sinR*sinP;
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : Quaternion_ToAngE
**功能 : 四元数转欧拉角
**输入 : *pNumQ, *pAngE
**输出 : None
**使用 : Quaternion_ToAngE(&NumQ, &AngE);
**====================================================================================================*/
/*====================================================================================================*/
void Quaternion_ToAngE( Quaternion *pNumQ, EulerAngle *pAngE, float *_matrix)
{
	Quaternion_ToMatrix(pNumQ, _matrix);

	pAngE->Pitch = -asinf(_matrix[2]);
	pAngE->Roll    = atan2f(_matrix[5], _matrix[8]);
	pAngE->Yaw    = atan2f(_matrix[1], _matrix[0]);
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : Quaternion_Multiply
**功能 : 四元数乘法
**输入 : NowQ, OldQ
**输出 : NewQ
**使用 : NewQ = Quaternion_Multiply(NowQ, OldQ);
**====================================================================================================*/
/*====================================================================================================*/
Quaternion Quaternion_Multiply( Quaternion NowQ, Quaternion OldQ )
{
  Quaternion NewQ;

  NewQ.q0 = NowQ.q0*OldQ.q0 - NowQ.q1*OldQ.q1 - NowQ.q2*OldQ.q2 - NowQ.q3*OldQ.q3;
  NewQ.q1 = NowQ.q0*OldQ.q1 + NowQ.q1*OldQ.q0 + NowQ.q2*OldQ.q3 - NowQ.q3*OldQ.q2;
  NewQ.q2 = NowQ.q0*OldQ.q2 - NowQ.q1*OldQ.q3 + NowQ.q2*OldQ.q0 + NowQ.q3*OldQ.q1;
  NewQ.q3 = NowQ.q0*OldQ.q3 + NowQ.q1*OldQ.q2 - NowQ.q2*OldQ.q1 + NowQ.q3*OldQ.q0;

  Quaternion_Normalize(&NewQ);

  return NewQ;
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : Quaternion_Normalize
**功能 : 四元数归一化
**输入 : *pNumQ
**输出 : None
**使用 : Quaternion_Normalize(&NewQ);
**====================================================================================================*/
/*====================================================================================================*/
void Quaternion_Normalize( Quaternion *pNumQ )
{
  float Normalize = 0.0f;

	Normalize = Q_rsqrt(squa(pNumQ->q0) + squa(pNumQ->q1) + squa(pNumQ->q2) + squa(pNumQ->q3));

  pNumQ->q0 = pNumQ->q0 * Normalize;
  pNumQ->q1 = pNumQ->q1 * Normalize;
  pNumQ->q2 = pNumQ->q2 * Normalize;
  pNumQ->q3 = pNumQ->q3 * Normalize;
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : Quaternion_RungeKutta
**功能 : 一阶龙格库塔法, 更新四元數
**输入 : *pNumQ, GyrX, GyrY, GyrZ, helfTimes
**输出 : None
**使用 : Quaternion_RungeKutta(&NumQ, GyrX, GyrY, GyrZ, helfT);
**====================================================================================================*/
/*====================================================================================================*/
void Quaternion_RungeKutta( Quaternion *pNumQ, float GyrX, float GyrY, float GyrZ, float helfTimes )
{
  float tmpq0 = pNumQ->q0;
  float tmpq1 = pNumQ->q1;
  float tmpq2 = pNumQ->q2;
  float tmpq3 = pNumQ->q3;

  pNumQ->q0 = pNumQ->q0 + (-tmpq1*GyrX - tmpq2*GyrY - tmpq3*GyrZ) * helfTimes;
  pNumQ->q1 = pNumQ->q1 + ( tmpq0*GyrX - tmpq3*GyrY + tmpq2*GyrZ) * helfTimes;
  pNumQ->q2 = pNumQ->q2 + ( tmpq3*GyrX + tmpq0*GyrY - tmpq1*GyrZ) * helfTimes;
  pNumQ->q3 = pNumQ->q3 + (-tmpq2*GyrX + tmpq1*GyrY + tmpq0*GyrZ) * helfTimes;
}
/*====================================================================================================*/

void Quaternion_ToMatrix(Quaternion *pNumQ, float *_matrix)
{
	_matrix[0] = pNumQ->q0*pNumQ->q0 + pNumQ->q1*pNumQ->q1 - pNumQ->q2*pNumQ->q2 - pNumQ->q3*pNumQ->q3; // 11
    _matrix[1] = 2.0f*(pNumQ->q0*pNumQ->q3 + pNumQ->q1*pNumQ->q2); // 12
    _matrix[2] = 2.0f*(pNumQ->q1*pNumQ->q3 - pNumQ->q0*pNumQ->q2);	// 13
    _matrix[3] = 2.0f*(pNumQ->q1*pNumQ->q2 - pNumQ->q0*pNumQ->q3);	// 21
    _matrix[4] = pNumQ->q0*pNumQ->q0 - pNumQ->q1*pNumQ->q1 + pNumQ->q2*pNumQ->q2 - pNumQ->q3*pNumQ->q3;;// 22
    _matrix[5] = 2.0f*(pNumQ->q0*pNumQ->q1 + pNumQ->q2*pNumQ->q3);	// 23
    _matrix[6] = 2.0f*(pNumQ->q1*pNumQ->q3 + pNumQ->q0*pNumQ->q2);	// 31
    _matrix[7] = 2.0f*(pNumQ->q2*pNumQ->q3 - pNumQ->q0*pNumQ->q1);	// 32
    _matrix[8] = pNumQ->q0*pNumQ->q0 - pNumQ->q1*pNumQ->q1 - pNumQ->q2*pNumQ->q2 + pNumQ->q3*pNumQ->q3; // 33
}