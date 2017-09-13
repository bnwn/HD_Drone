#include "PN020Series.h"
#include "ahrs.h"
#include "inertial_sensor.h"
#include "../Algorithm/Algorithm_filter/Algorithm_filter.h"
#include "../Algorithm/Algorithm_math/Algorithm_math.h"
#include "scheduler.h"
#include "common.h"

#if SENSOR_TYPE == SENSOR_BMI160
#define KpDef 1.0f
#define KiDef 0.005f
#define SampleRateHalf 0.005 //0.00125f  //0.001
#else
#define KpDef 0.8f
#define KiDef 0.0005f
#define SampleRateHalf 0.00125f  //0.001
#endif

// /*	
// 	Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
// 	R:测量噪声，R增大，动态响应变慢，收敛稳定性变好	
// */
// #define KALMAN_Q        0.02
// #define KALMAN_R        8.0000

Quaternion NumQ = {1, 0, 0, 0};
EulerAngle AngE = {0};
struct AHRS ahrs = {0};
float Rot_matrix[9] = {1.f,  0.0f,  0.0f, 0.0f,  1.f,  0.0f, 0.0f,  0.0f,  1.f };		/**< init: identity matrix */

float cmp_kp = KpDef, cmp_ki = KiDef;
float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;

/*====================================================================================================*/
/*====================================================================================================*
**函数 : AHRS_Update
**功能 : AHRS获取欧拉角
**输入 : None
**输出 : None
**使用 : AHRS_Update();
**====================================================================================================*/
/*====================================================================================================*/
void AHRS_Update(void)
{
//    float sin_pitch,sin_roll,cos_roll,cos_pitch;
    int i = 0;
    // 获取四元数
    AHRS_GetQ();

    // 四元数转欧拉角
    Quaternion_ToAngE(&NumQ, &AngE, Rot_matrix);

    // 计算欧拉角的三角函数值
//    sin_roll  = sin(AngE.Roll);
//    sin_pitch = sin(AngE.Pitch);
//    cos_roll  = cos(AngE.Roll);
//    cos_pitch = cos(AngE.Pitch);

    //  地磁不存在或地磁数据不正常则停用地磁数据
    //flag.MagIssue=0;//地磁存在问题，待调试解决
    //flag.MagExist=0;

#if 0
    if(!flag.MagIssue && flag.MagExist){//40US
        // 地磁倾角补偿
        float hx = MAG[0]*cos_pitch + MAG[1]*sin_pitch*sin_roll - MAG[2]*cos_roll*sin_pitch;
        float hy = MAG[1]*cos_roll + MAG[2]*sin_roll;

        // 利用地磁解算航向角
        float mag_yaw = -Degree(atan2((fp64)hy,(fp64)hx));
        // 陀螺仪积分解算航向角
//		AngE.Yaw += Degree(inertial_sensor.gyro.filter.z * Gyro_Gr * 2 * SampleRateHalf);//重大bug，已经是角度，这里不能再乘以Gyro_Gr了
        AngE.Yaw += inertial_sensor.gyro.filter.z * 2  * SampleRateHalf;
        // 地磁解算的航向角与陀螺仪积分解算的航向角进行互补融合
        if((mag_yaw>90 && IMU.Yaw<-90) || (mag_yaw<-90 && IMU.Yaw>90))
            IMU.Yaw = -IMU.Yaw * 0.988f + mag_yaw * 0.012f;
        else
            IMU.Yaw = IMU.Yaw * 0.988f + mag_yaw * 0.012f;
    }
    else
#endif

    //DCM . ground to body
    for(i=0; i<9; i++)
    {
        *(&(ahrs.dcm[0][0]) + i)=Rot_matrix[i];
    }

	/* rotation */
    ahrs.Yaw = (float)Degree((double)AngE.Yaw) * IMU_SENSOR_Z_FACTOR;
    ahrs.Roll = (float)Degree((double)AngE.Roll) * IMU_SENSOR_X_FACTOR;  // roll
    ahrs.Pitch = (float)Degree((double)AngE.Pitch) * IMU_SENSOR_Y_FACTOR; // pitch
#ifdef __DEVELOP__
//		printf("attitude:%.3f, %.3f, %.3f\n", ahrs.Roll, ahrs.Pitch, ahrs.Yaw);
#endif
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : AHRS_GetQ
**功能 : AHRS
**输入 : None
**输出 : None
**使用 : AHRS_GetQ();
**====================================================================================================*/
/*====================================================================================================*/
void AHRS_GetQ(void)
{
    float ErrX, ErrY, ErrZ;
    float AccX, AccY, AccZ;
    float GyrX, GyrY, GyrZ;
    float Normalize;
	_Vector_Float V;//重力分量
	
	static uint32_t timestamp = 0;
	uint32_t now = sys_micro();
	float dt = (timestamp>0) ? ((float)(now-timestamp)/1000000.0f) : 0;
	timestamp = now;
	
	
#ifdef __DEVELOP__
		//printf(" filter : acc.x:%.3f, acc.y:%.3f, acc.z:%.3f \n", inertial_sensor.accel.filter.x, inertial_sensor.accel.filter.y, inertial_sensor.accel.filter.z);
		//printf(" filter : gyro.x:%.3f, gyro.y:%.3f, gyro.z:%.3f \n", inertial_sensor.gyro.filter.x, inertial_sensor.gyro.filter.y, inertial_sensor.gyro.filter.z);
#endif

	
    // 加速度归一化
    Normalize = Q_rsqrt(squa(inertial_sensor.accel.filter.x) + squa(inertial_sensor.accel.filter.y) + squa(inertial_sensor.accel.filter.z));
    AccX = inertial_sensor.accel.filter.x * Normalize;
    AccY = inertial_sensor.accel.filter.y * Normalize;
    AccZ = inertial_sensor.accel.filter.z * Normalize;

    // 提取重力分量
    V = Quaternion_vectorGravity(&NumQ);

    // 向量差乘
    ErrX = (AccY*V.z - AccZ*V.y);
    ErrY = (AccZ*V.x - AccX*V.z);
    ErrZ = (AccX*V.y - AccY*V.x);

    exInt = exInt + ErrX * cmp_ki;
    eyInt = eyInt + ErrY * cmp_ki;
    ezInt = ezInt + ErrZ * cmp_ki;

    GyrX = Rad(inertial_sensor.gyro.filter.x) + cmp_kp * VariableParameter(ErrX) * ErrX  +  exInt;
    GyrY = Rad(inertial_sensor.gyro.filter.y) + cmp_kp * VariableParameter(ErrY) * ErrY  +  eyInt;
    GyrZ = Rad(inertial_sensor.gyro.filter.z) + cmp_kp * VariableParameter(ErrZ) * ErrZ  +  ezInt;


    // 一阶龙格库塔法, 更新四元数
    Quaternion_RungeKutta(&NumQ, GyrX, GyrY, GyrZ, (float)(dt/2.0f));

    // 四元数归一化
    Quaternion_Normalize(&NumQ);
}

void AHRS_Read_Attitude(EulerAngle *_attitude)
{
		_attitude->Roll = ahrs.Roll;
		_attitude->Pitch = ahrs.Pitch;
		_attitude->Yaw = ahrs.Yaw;
}

//void AHRS_set_complementary_filter_kp(float _kp)
//{
//		SCHEDULER_STOP;
//		cmp_kp = _kp;
//#ifdef __DEVELOP__
//		printf("kp: %.6f\n", cmp_kp);
//#endif
//		memset(&NumQ, 0, sizeof(Quaternion));
//		NumQ.q0 = 1;
//		memset(&AngE, 0, sizeof(EulerAngle));
//		memset(&ahrs, 0, sizeof(EulerAngle));
//		exInt = 0;
//		exInt = 0;
//		exInt = 0;
//		accel_offset();
//		SCHEDULER_RUN;
//}

//void AHRS_set_complementary_filter_ki(float _ki)
//{
//		SCHEDULER_STOP;
//		cmp_ki = _ki;
//#ifdef __DEVELOP__
//		printf("ki: %.6f\n", cmp_ki);
//#endif
//		memset(&NumQ, 0, sizeof(Quaternion));
//		NumQ.q0 = 1;
//		memset(&AngE, 0, sizeof(EulerAngle));
//		memset(&ahrs, 0, sizeof(EulerAngle));
//		exInt = 0;
//		exInt = 0;
//		exInt = 0;
//		accel_offset();
//		SCHEDULER_RUN;
//}
