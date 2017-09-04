#include "PN020Series.h"
#include "inertial_nav.h"
#include "Algorithm_math.h"
#include "Algorithm_filter.h"
#include "fbm320.h"
#include "bmi160.h"
#include "ahrs.h"

_Nav_t nav = {0}; // NED frame in earth
_Vector_Float home_absolute_pos;

void inertial_nav_init(void)
{
	home_absolute_pos.x = 0.0f;
	home_absolute_pos.y = 0.0f;
	home_absolute_pos.z = 0.0f;
}
	
void inertial_nav_update(void)
{
    position_z_update(0.005f);
}

void position_z_update(float dt)
{
	static float z_est[3] = {0.0f, 0.0f, 0.0f};	// estimate z Vz  Az
	static float w_z_baro = 0.5f;
	static float w_z_acc = 20.0f;
	static float w_acc_bias = 0.05f;
	/* acceleration in NED frame */
	static float accel_NED[3] = {0.0f, 0.0f, -GRAVITY_MSS};
	/* store error when sensor updates, but correct on each time step to avoid jumps in estimated value */
	static float corr_acc[3] = {0.0f, 0.0f, 0.0f};	// N E D ,  m/s2
	static float acc_bias[3] = {0.0f, 0.0f, 0.0f};	// body frame ,
	
    uint8_t i, j;
	float corr_baro;//m
    /* accelerometer bias correction */
    float accel_bias_corr[3] = { 0.0f, 0.0f, 0.0f };
	float co_factor[3] = {IMU_SENSOR_X_FACTOR, IMU_SENSOR_Y_FACTOR, IMU_SENSOR_Z_FACTOR};
	
	if (!fc_status.position_z_ok)
	{
		return;
	}
		
    if (fc_status.altitude_updated) { // update altitude in 16.67Hz
        float new_alt = fbm320_packet.altitude - home_absolute_pos.z;
        corr_baro = 0 - new_alt - z_est[0];
//        new_alt = Moving_Median(0, 5, new_alt);

//        if (ABS(new_alt - nav_pos.z) < 10) {
//            nav_pos.z += (new_alt - nav_pos.z) * 0.2736f; // 2Hz LPF
//        } else if (ABS(new_alt - nav_pos.z) < 50) {
//            nav_pos.z += (new_alt - nav_pos.z) * 0.1585f; // 1Hz LPF
//        } else {
//            nav_pos.z += (new_alt - nav_pos.z) * 0.086f; // 0.5Hz LPF
//        }
        fc_status.altitude_updated = false;
    }

    if (fc_status.accel_updated) {
		float acc[3];
        acc[0] = inertial_sensor.accel.filter.x - acc_bias[0];
        acc[1] = inertial_sensor.accel.filter.y - acc_bias[1];
        acc[2] = inertial_sensor.accel.filter.z - acc_bias[2];

        for(i=0; i<3; i++)
        {
            accel_NED[i] = 0.0f;
            for(j=0; j<3; j++)
            {
                accel_NED[i] += ahrs.dcm[j][i] * acc[j] * co_factor[i];
            }
        }
		
		accel_NED[2] += GRAVITY_MSS;
        corr_acc[2] = accel_NED[2] - z_est[2];

        fc_status.accel_updated = false;
    }

    //correct accelerometer bias every time step
    accel_bias_corr[2] -= corr_baro * w_z_baro * w_z_baro;

    //transform error vector from NED frame to body frame
    for (i = 0; i < 3; i++)
    {
        float c = 0.0f;

        for (j = 0; j < 3; j++) {
            c += ahrs.dcm[i][j] * accel_bias_corr[j];
        }

        acc_bias[i] += c * w_acc_bias * dt * co_factor[i];		//accumulate bias
    }

    /* inertial filter prediction for altitude */
    inertial_filter_predict(dt, z_est, z_est[2]);//accel_NED[2]);
    /* inertial filter correction for altitude */
    inertial_filter_correct(corr_baro, dt, z_est, 0, w_z_baro);	//0.5f
    inertial_filter_correct(corr_acc[2], dt, z_est, 2, w_z_acc);		//20.0f

    nav.z = z_est[0];
    nav.vz = z_est[1];
    nav.az = z_est[2];

///* Complementary filter */
//    height_thr = LIMIT( thr , 0, 700 );
//    thr_lpf += ( 1 / ( 1 + 1 / ( 2.0f *3.14f *T ) ) ) *( height_thr - thr_lpf );//对油门值低通滤波修正
//    userdata1[0]=	thr_lpf;//调试用
//    /*下面的低通滤波用于测试对比效果，最终没有选用*/
////wz_acc += ( 1 / ( 1 + 1 / ( 8 *3.14f *T ) ) ) *my_deathzoom( (V.z *(sensor.acc.averag.z- sensor.acc.quiet.z) + V.x *sensor.acc.averag.x + V.y *sensor.acc.averag.y - wz_acc),100 );//
////wz_acc_mms2 = (float)(wz_acc/8192.0f) *9800 ;
//    //加速度的静态零点是读取的EEPROM数据，但是每次飞行都可能不一样，能每次起飞前校准一次最好，可以单独校准Z轴的，自行设计程序
//    wz_acc_temp = V.z *(sensor.acc.averag.z- sensor.acc.quiet.z) + V.x *sensor.acc.averag.x + V.y *sensor.acc.averag.y;//
//    Moving_Average( wz_acc_temp,acc_speed_arr,ACC_SPEED_NUM,acc_cnt ,&wz_acc1 );
//    tempacc_lpf= (float)(wz_acc1/8192.0f) *9800;//9800 *T;由于是+-4G共8G，65535/8g=8192 g，加速度，mms2毫米每平方秒
//    if(abs(tempacc_lpf)<50)tempacc_lpf=0;//简单消除下噪声
//    userdata1[2]=tempacc_lpf;
//    wz_speed_0 += tempacc_lpf *T;//加速度计积分成速度

//    if( ultra_start_f == 1 )////不管是啥模式，只要更新了超声波数据就进行运算
//    {
//         Ultra_dataporcess(15.0f*TT);
//         ultra_start_f=2;
//    }

//    if(baro_start_f==1)//不管是啥模式，只要更新了气压数据就进行运算
//    {
//         Baro_dataporcess(8.0f*TT);			 //8.0f*TT这么长时间更新一次气压计数据
//         userdata2[10]=baro_height;//Baro_Height_Source
//         baro_dis_delta=baro_height-baro_dis_old;
//         baro_dis_old=baro_height;//气压计速度可以继续优化，这里比较粗糙
//   //Moving_Average( (float)( baro_dis_delta/(8.0f*TT)),baro_speed_arr,BARO_SPEED_NUM,baro_cnt ,&baro_speed ); //单位mm/s
//         userdata1[11]=baro_speed_lpf=0.4* baro_dis_delta/(8.0f*TT);//baro_speed这里乘以系数以削减该值
//         baro_start_f=2;
//    }

//    if(flag.FlightMode==ULTRASONIC_High)
//    {
//         h_speed=ultra_speed;
//         wz_speed_0 = 0.99	*wz_speed_0 + 0.01*h_speed;//超声波垂直速度互补滤波
//    }
//    else if(flag.FlightMode==ATMOSPHERE_High)
//    {
//         h_speed=baro_speed_lpf;
//         wz_speed_0 = 0.99	*wz_speed_0 + 0.01*h_speed;//气压计垂直速度互补滤波，系数可调
//    }

//    userdata1[3] =h_speed;//h_speed是高度环传到速度环的实测高度方向速度【但可能是错误的，气压计速度不可靠】
//    userdata1[4]=wz_speed_0;//调试用
//    hc_speed_i += 0.4f *T *( h_speed - wz_speed );//速度偏差积分，乘以了0.4系数
//    hc_speed_i = LIMIT( hc_speed_i, -500, 500 );//积分限幅
//    userdata1[5] =hc_speed_i;//这个没显示
//    wz_speed_0 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( h_speed - wz_speed_0  ) ;//0.1实测速度修正加速度算的速度
//    userdata1[6] = wz_speed=wz_speed_0 + hc_speed_i;//经过修正的速度+经过限幅的增量式速度积分

}

void update_home_pos(void)
{
	static uint8_t count = 0;
	if ((!fc_status.position_z_ok) && fc_status.home_abs_alt_updated) {
		count++;
		if (count > 4)
			home_absolute_pos.z += fbm320_packet.altitude;
		if (count == 14) {
			home_absolute_pos.z = home_absolute_pos.z / (count - 4);
			count = 0;
			fc_status.position_z_ok = true;
		}
		fc_status.home_abs_alt_updated = false;;
	} else if (fc_status.armed == ARMED && fc_status.home_abs_alt_updated) {
		home_absolute_pos.z += (fbm320_packet.altitude - home_absolute_pos.z) * 0.2736f; // 2Hz LPF
		fc_status.home_abs_alt_updated = false;
	}
}

//Combine Filter to correct err
static void inertial_filter_predict(float dt, float *x, float acc)
{
    x[0] += x[1] * dt + acc * dt * dt / 2.0f;
    x[1] += acc * dt;
}

static void inertial_filter_correct(float e, float dt, float *x, int i, float w)
{
    float ewdt = e * w * dt;
    x[i] += ewdt;

    if (i == 0) {
        x[1] += w * ewdt;
        x[2] += w * w * ewdt / 3.0;

    } else if (i == 1) {
        x[2] += w * ewdt;
    }
}

