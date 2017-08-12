#include "PN020Series.h"
#include "ahrs.h"
#include "inertial_sensor.h"
#include "../Algorithm/Algorithm_filter/Algorithm_filter.h"
#include "../Algorithm/Algorithm_quaternion/Algorithm_quaternion.h"
#include "../Algorithm/Algorithm_math/Algorithm_math.h"
#include "../Algorithm/Algorithm_math/mymath.h"

#define KpDef 0.8f
#define KiDef 0.0005f
#define SampleRateHalf 0.00125f  //0.001

// /*	
// 	Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
// 	R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��	
// */
// #define KALMAN_Q        0.02
// #define KALMAN_R        8.0000

Quaternion NumQ = {1, 0, 0, 0};
EulerAngle AngE = {0},IMU = {0};

int16_t MAG[3];
Gravity V;//��������

/*====================================================================================================*/
/*====================================================================================================*
**���� : AHRS_GetQ
**���� : AHRS
**���� : None
**��� : None
**ʹ�� : AHRS_GetQ();
**====================================================================================================*/
/*====================================================================================================*/
void AHRS_GetQ( Quaternion *pNumQ )
{
  float ErrX, ErrY, ErrZ;
  float AccX, AccY, AccZ;
  float GyrX, GyrY, GyrZ;
    float Normalize;
  static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;

	
	// ���ٶȹ�һ��
	Normalize = Q_rsqrt(squa(sensor.acc.averag.x)+ squa(sensor.acc.averag.y) +squa(sensor.acc.averag.z));
	AccX = sensor.acc.averag.x*Normalize;
  AccY = sensor.acc.averag.y*Normalize;
  AccZ = sensor.acc.averag.z*Normalize;

	// ��ȡ��������
	V = Quaternion_vectorGravity(&NumQ);
	
	// �������
 	ErrX = (AccY*V.z - AccZ*V.y);
  ErrY = (AccZ*V.x - AccX*V.z);
  ErrZ = (AccX*V.y - AccY*V.x);
 	
 	exInt = exInt + ErrX * KiDef;
  eyInt = eyInt + ErrY * KiDef;
  ezInt = ezInt + ErrZ * KiDef;

  GyrX = Rad(sensor.gyro.averag.x) + KpDef * VariableParameter(ErrX) * ErrX  +  exInt;
  GyrY = Rad(sensor.gyro.averag.y) + KpDef * VariableParameter(ErrY) * ErrY  +  eyInt;
	GyrZ = Rad(sensor.gyro.averag.z) + KpDef * VariableParameter(ErrZ) * ErrZ  +  ezInt;
	
	
	// һ�����������, ������Ԫ��
	Quaternion_RungeKutta(&NumQ, GyrX, GyrY, GyrZ, SampleRateHalf);
	
	// ��Ԫ����һ��
	Quaternion_Normalize(&NumQ);
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : AHRS_Update
**���� : AHRS��ȡŷ����
**���� : None
**��� : None
**ʹ�� : AHRS_Update();
**====================================================================================================*/
/*====================================================================================================*/
void AHRS_Update(void)
{
    float sin_pitch,sin_roll,cos_roll,cos_pitch;
	
    inertial_sensor_read();
	
	// ��ȡ��Ԫ��
  AHRS_GetQ(&NumQ);
	
  // ��Ԫ��תŷ����
	Quaternion_ToAngE(&NumQ, &AngE);
	
  // ����ŷ���ǵ����Ǻ���ֵ
  sin_roll  = sin(AngE.Roll);
  sin_pitch = sin(AngE.Pitch);
  cos_roll  = cos(AngE.Roll);
  cos_pitch = cos(AngE.Pitch);
	
	//  �شŲ����ڻ�ش����ݲ�������ͣ�õش�����
  //flag.MagIssue=0;//�شŴ������⣬�����Խ��
  //flag.MagExist=0;

#if 0
	if(!flag.MagIssue && flag.MagExist){//40US
		// �ش���ǲ���
        float hx = MAG[0]*cos_pitch + MAG[1]*sin_pitch*sin_roll - MAG[2]*cos_roll*sin_pitch;
        float hy = MAG[1]*cos_roll + MAG[2]*sin_roll;
		
		// ���õشŽ��㺽���
        float mag_yaw = -Degree(atan2((fp64)hy,(fp64)hx));
		// �����ǻ��ֽ��㺽���
//		AngE.Yaw += Degree(sensor.gyro.averag.z * Gyro_Gr * 2 * SampleRateHalf);//�ش�bug���Ѿ��ǽǶȣ����ﲻ���ٳ���Gyro_Gr��
		AngE.Yaw += sensor.gyro.averag.z * 2  * SampleRateHalf;
		// �شŽ���ĺ�����������ǻ��ֽ���ĺ���ǽ��л����ں� 
		if((mag_yaw>90 && IMU.Yaw<-90) || (mag_yaw<-90 && IMU.Yaw>90)) 
			IMU.Yaw = -IMU.Yaw * 0.988f + mag_yaw * 0.012f;
		else 
			IMU.Yaw = IMU.Yaw * 0.988f + mag_yaw * 0.012f;
	}
	else 
#endif
		IMU.Yaw = Degree(AngE.Yaw);


  IMU.Roll = Degree(AngE.Roll);  // roll
	IMU.Pitch = Degree(AngE.Pitch); // pitch
}
