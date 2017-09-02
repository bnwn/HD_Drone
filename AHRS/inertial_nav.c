#include "PN020Series.h"
#include "inertial_nav.h"
#include "Algorithm_math.h"
#include "mymath.h"
#include "Algorithm_filter.h"
#include "fbm320.h"

Position nav_pos = {0};
Position nav_pos_vel = {0};
Position home_absolute_pos = {0};

void inertial_nav_update(void)
{
    position_z_update();
}

void position_z_update()
{
    if (fc_status.altitude_updated) { // update altitude in 16.67Hz
        float new_alt = fbm320_packet.Altitude - home_absolute_pos.z;
        new_alt = Moving_Median(0, 5, new_alt);

        if (ABS(new_alt - nav_pos.z) < 10) {
            nav_pos.z = (new_alt - nav_pos.z) * 0.2736f; // 2Hz LPF
        } else if (ABS(new_alt - nav_pos.z) < 50) {
            nav_pos.z = (new_alt - nav_pos.z) * 0.1585f; // 1Hz LPF
        } else {
            nav_pos.z = (new_alt - nav_pos.z) * 0.086f; // 0.5Hz LPF
        }
        fc_status.altitude_updated = false;
    }
    height_thr = LIMIT( thr , 0, 700 );
    thr_lpf += ( 1 / ( 1 + 1 / ( 2.0f *3.14f *T ) ) ) *( height_thr - thr_lpf );//������ֵ��ͨ�˲�����
    userdata1[0]=	thr_lpf;//������
    /*����ĵ�ͨ�˲����ڲ��ԶԱ�Ч��������û��ѡ��*/
//wz_acc += ( 1 / ( 1 + 1 / ( 8 *3.14f *T ) ) ) *my_deathzoom( (V.z *(sensor.acc.averag.z- sensor.acc.quiet.z) + V.x *sensor.acc.averag.x + V.y *sensor.acc.averag.y - wz_acc),100 );//
//wz_acc_mms2 = (float)(wz_acc/8192.0f) *9800 ;
    //���ٶȵľ�̬����Ƕ�ȡ��EEPROM���ݣ�����ÿ�η��ж����ܲ�һ������ÿ�����ǰУ׼һ����ã����Ե���У׼Z��ģ�������Ƴ���
    wz_acc_temp = V.z *(sensor.acc.averag.z- sensor.acc.quiet.z) + V.x *sensor.acc.averag.x + V.y *sensor.acc.averag.y;//
    Moving_Average( wz_acc_temp,acc_speed_arr,ACC_SPEED_NUM,acc_cnt ,&wz_acc1 );
    tempacc_lpf= (float)(wz_acc1/8192.0f) *9800;//9800 *T;������+-4G��8G��65535/8g=8192 g�����ٶȣ�mms2����ÿƽ����
    if(abs(tempacc_lpf)<50)tempacc_lpf=0;//������������
    userdata1[2]=tempacc_lpf;
    wz_speed_0 += tempacc_lpf *T;//���ٶȼƻ��ֳ��ٶ�

    if( ultra_start_f == 1 )////������ɶģʽ��ֻҪ�����˳��������ݾͽ�������
    {
         Ultra_dataporcess(15.0f*TT);
         ultra_start_f=2;
    }

    if(baro_start_f==1)//������ɶģʽ��ֻҪ��������ѹ���ݾͽ�������
    {
         Baro_dataporcess(8.0f*TT);			 //8.0f*TT��ô��ʱ�����һ����ѹ������
         userdata2[10]=baro_height;//Baro_Height_Source
         baro_dis_delta=baro_height-baro_dis_old;
         baro_dis_old=baro_height;//��ѹ���ٶȿ��Լ����Ż�������Ƚϴֲ�
   //Moving_Average( (float)( baro_dis_delta/(8.0f*TT)),baro_speed_arr,BARO_SPEED_NUM,baro_cnt ,&baro_speed ); //��λmm/s
         userdata1[11]=baro_speed_lpf=0.4* baro_dis_delta/(8.0f*TT);//baro_speed�������ϵ����������ֵ
         baro_start_f=2;
    }

    if(flag.FlightMode==ULTRASONIC_High)
    {
         h_speed=ultra_speed;
         wz_speed_0 = 0.99	*wz_speed_0 + 0.01*h_speed;//��������ֱ�ٶȻ����˲�
    }
    else if(flag.FlightMode==ATMOSPHERE_High)
    {
         h_speed=baro_speed_lpf;
         wz_speed_0 = 0.99	*wz_speed_0 + 0.01*h_speed;//��ѹ�ƴ�ֱ�ٶȻ����˲���ϵ���ɵ�
    }

    userdata1[3] =h_speed;//h_speed�Ǹ߶Ȼ������ٶȻ���ʵ��߶ȷ����ٶȡ��������Ǵ���ģ���ѹ���ٶȲ��ɿ���
    userdata1[4]=wz_speed_0;//������
    hc_speed_i += 0.4f *T *( h_speed - wz_speed );//�ٶ�ƫ����֣�������0.4ϵ��
    hc_speed_i = LIMIT( hc_speed_i, -500, 500 );//�����޷�
    userdata1[5] =hc_speed_i;//���û��ʾ
    wz_speed_0 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( h_speed - wz_speed_0  ) ;//0.1ʵ���ٶ��������ٶ�����ٶ�
    userdata1[6] = wz_speed=wz_speed_0 + hc_speed_i;//�����������ٶ�+�����޷�������ʽ�ٶȻ���

}
