#include "PN020Series.h"
#include "ahrs.h"
#include "inertial_sensor.h"
#include "../Algorithm/Algorithm_filter/Algorithm_filter.h"
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
EulerAngle AngE = {0}, ahrs = {0};

int16_t MAG[3];
Gravity V;//��������

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

    // ��ȡ��Ԫ��
    AHRS_GetQ();

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
//		AngE.Yaw += Degree(inertial_sensor.gyro.filter.z * Gyro_Gr * 2 * SampleRateHalf);//�ش�bug���Ѿ��ǽǶȣ����ﲻ���ٳ���Gyro_Gr��
        AngE.Yaw += inertial_sensor.gyro.filter.z * 2  * SampleRateHalf;
        // �شŽ���ĺ�����������ǻ��ֽ���ĺ���ǽ��л����ں�
        if((mag_yaw>90 && IMU.Yaw<-90) || (mag_yaw<-90 && IMU.Yaw>90))
            IMU.Yaw = -IMU.Yaw * 0.988f + mag_yaw * 0.012f;
        else
            IMU.Yaw = IMU.Yaw * 0.988f + mag_yaw * 0.012f;
    }
    else
#endif

    ahrs.Yaw = Degree(AngE.Yaw);
    ahrs.Roll = Degree(AngE.Roll);  // roll
    ahrs.Pitch = Degree(AngE.Pitch); // pitch
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : AHRS_GetQ
**���� : AHRS
**���� : None
**��� : None
**ʹ�� : AHRS_GetQ();
**====================================================================================================*/
/*====================================================================================================*/
void AHRS_GetQ(void)
{
    float ErrX, ErrY, ErrZ;
    float AccX, AccY, AccZ;
    float GyrX, GyrY, GyrZ;
    float Normalize;
    static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;

		printf(" filter : acc.x:%.3f, acc.y:%.3f, acc.z:%.3f \n", inertial_sensor.accel.filter.x, inertial_sensor.accel.filter.y, inertial_sensor.accel.filter.z);
		printf(" filter : gyro.x:%.3f, gyro.y:%.3f, gyro.z:%.3f \n", inertial_sensor.gyro.filter.x, inertial_sensor.gyro.filter.y, inertial_sensor.gyro.filter.z);

	
    // ���ٶȹ�һ��
    Normalize = Q_rsqrt(squa(inertial_sensor.accel.filter.x) + squa(inertial_sensor.accel.filter.y) + squa(inertial_sensor.accel.filter.z));
    AccX = inertial_sensor.accel.filter.x * Normalize;
    AccY = inertial_sensor.accel.filter.y * Normalize;
    AccZ = inertial_sensor.accel.filter.z * Normalize;

    // ��ȡ��������
    V = Quaternion_vectorGravity(&NumQ);

    // �������
    ErrX = (AccY*V.z - AccZ*V.y);
    ErrY = (AccZ*V.x - AccX*V.z);
    ErrZ = (AccX*V.y - AccY*V.x);

    exInt = exInt + ErrX * KiDef;
    eyInt = eyInt + ErrY * KiDef;
    ezInt = ezInt + ErrZ * KiDef;

    GyrX = Rad(inertial_sensor.gyro.filter.x) + KpDef * VariableParameter(ErrX) * ErrX  +  exInt;
    GyrY = Rad(inertial_sensor.gyro.filter.y) + KpDef * VariableParameter(ErrY) * ErrY  +  eyInt;
    GyrZ = Rad(inertial_sensor.gyro.filter.z) + KpDef * VariableParameter(ErrZ) * ErrZ  +  ezInt;


    // һ�����������, ������Ԫ��
    Quaternion_RungeKutta(&NumQ, GyrX, GyrY, GyrZ, SampleRateHalf);

    // ��Ԫ����һ��
    Quaternion_Normalize(&NumQ);
}

