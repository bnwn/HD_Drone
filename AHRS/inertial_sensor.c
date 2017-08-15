#include "PN020Series.h"
#include "inertial_sensor.h"
#include "../Driver/bmi160.h"
#include "../Algorithm/Algorithm_filter/Algorithm_filter.h"
#include "../Algorithm/Algorithm_math/Algorithm_math.h"

#define  IIR_ORDER     4      //使用IIR滤波器的阶数
double b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //系数b
double a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//系数a
double InPut_IIR[3][IIR_ORDER+1] = {0};
double OutPut_IIR[3][IIR_ORDER+1] = {0};

Inertial_Sensor inertial_sensor;

void inertial_sensor_read(void)
{
    static _Vector_Int16 last_gyro;

    switch (SENSOR_TYPE) {
        case SENSOR_BMI160:
            bmi160_read_raw(&inertial_sensor);
        break;
    default:
        break;
    }

    inertial_sensor.accel.relative.x = inertial_sensor.accel.average.x - inertial_sensor.accel.quiet.x;
    inertial_sensor.accel.relative.y = inertial_sensor.accel.average.y - inertial_sensor.accel.quiet.y;
    inertial_sensor.accel.relative.z = inertial_sensor.accel.average.z;

    inertial_sensor.gyro.relative.x = inertial_sensor.gyro.average.x - inertial_sensor.gyro.quiet.x;
    inertial_sensor.gyro.relative.y = inertial_sensor.gyro.average.y - inertial_sensor.gyro.quiet.y;
    inertial_sensor.gyro.relative.z = inertial_sensor.gyro.average.z - inertial_sensor.gyro.quiet.z;

    // 加速度计IIR滤波
    inertial_sensor.accel.filter.x = IIR_I_Filter(inertial_sensor.accel.relative.x, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
    inertial_sensor.accel.filter.y = IIR_I_Filter(inertial_sensor.accel.relative.y, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
    inertial_sensor.accel.filter.z = IIR_I_Filter(inertial_sensor.accel.relative.z, InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);

    // 陀螺仪一阶低通滤波
    inertial_sensor.gyro.filter.x = LPF_1st(last_gyro.x, inertial_sensor.gyro.relative.x * _gyro_scale, 0.386f);
    last_gyro.x = inertial_sensor.gyro.filter.x;
    inertial_sensor.gyro.filter.y = LPF_1st(last_gyro.y, inertial_sensor.gyro.relative.y * _gyro_scale, 0.386f);
    last_gyro.y = inertial_sensor.gyro.filter.y;
    inertial_sensor.gyro.filter.z = LPF_1st(last_gyro.z, inertial_sensor.gyro.relative.z * _gyro_scale, 0.386f);
    last_gyro.z = inertial_sensor.gyro.filter.z;//
		
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Gyro_Calculateoffest
**功能 : 计算陀螺仪零偏
**输入 :
**输出 : None
**使用 : Hto_Gyro_Calculateoffest();
**====================================================================================================*/
/*====================================================================================================*/
void gyro_caloffest(float x,float y,float z,uint16_t amount)
{
   inertial_sensor.gyro.quiet.x = x / amount;
     inertial_sensor.gyro.quiet.y = y / amount;
     inertial_sensor.gyro.quiet.z = z / amount;
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Gyro_OFFSET
**功能 : 陀螺仪静态采集
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void gyro_offset(void)
{
    static uint8_t over_flag=0;
    uint8_t  i,cnt_g = 0;

  int16_t gx_last=0,gy_last=0,gz_last=0;
    int16_t Integral[3] = {0,0,0};
    int32_t tempg[3]={0,0,0};
  over_flag=0;//因为定义的是static，如果不自己赋值，下次进来时over_flag就不会被赋值0了，保持为上一次校准完时赋值的1

    delay_ms(1500);
    delay_ms(1500);

    while(!over_flag)	//此循环是确保四轴处于完全静止状态
    {
        if(cnt_g < 30)
        {
            switch (SENSOR_TYPE) {
                case SENSOR_BMI160:
                    bmi160_read_raw(&inertial_sensor);
                break;
            default:
                break;
            }

            tempg[0] += inertial_sensor.gyro.average.x;
            tempg[1] += inertial_sensor.gyro.average.y;
            tempg[2] += inertial_sensor.gyro.average.z;

            Integral[0] += absu16(gx_last - inertial_sensor.gyro.average.x);
            Integral[1] += absu16(gy_last - inertial_sensor.gyro.average.y);
            Integral[2] += absu16(gz_last - inertial_sensor.gyro.average.z);

					//printf(" average : gyro.x:%d, gyro.y:%d, gyro.z:%d \n", inertial_sensor.gyro.average.x, inertial_sensor.gyro.average.y, inertial_sensor.gyro.average.z);

            gx_last = inertial_sensor.gyro.average.x;
            gy_last = inertial_sensor.gyro.average.y;
            gz_last = inertial_sensor.gyro.average.z;
        }
        else{
            // 未校准成功
            if(Integral[0]>=GYRO_GATHER || Integral[1]>=GYRO_GATHER || Integral[2]>= GYRO_GATHER){
                cnt_g = 0;
                for(i=0;i<3;i++){
                    tempg[i]=Integral[i]=0;
                }
            }
            // 校准成功
            else{
                   gyro_caloffest(tempg[0],tempg[1],tempg[2],200);
                   over_flag = 1;
                   // flag.calibratingG = 0;//成功后清楚校准标记
            }
        }
        cnt_g++;
    }
//    if(flag.calibratingA)
//      accel_offset();
}

/*====================================================================================================*
**函数 : Accl_OFFSET
**功能 : 加速度计补偿
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void accel_offset(void)
{
    int32_t	tempax=0,tempay=0,tempaz=0;
    uint8_t cnt_a=0;
    inertial_sensor.accel.quiet.x = 0;
    inertial_sensor.accel.quiet.y = 0;
    inertial_sensor.accel.quiet.z = 0;

    for(cnt_a=0;cnt_a<200;cnt_a++)
    {
				switch (SENSOR_TYPE) {
						case SENSOR_BMI160:
								bmi160_read_raw(&inertial_sensor);
						break;
				default:
						break;
				}
        tempax+= inertial_sensor.accel.average.x;
        tempay+= inertial_sensor.accel.average.y;
        tempaz+= inertial_sensor.accel.average.z;
        cnt_a++;
    }
    inertial_sensor.accel.quiet.x = tempax/cnt_a;
    inertial_sensor.accel.quiet.y = tempay/cnt_a;
    inertial_sensor.accel.quiet.z = tempaz/cnt_a;
    cnt_a = 0;
    //flag.calibratingA = 0;
    //EE_SAVE_ACC_OFFSET();//保存数据
}

