#include "motor_control.h"

Thrust thrust;
float target_throttle;

/*====================================================================================================*/
/*====================================================================================================*
**���� : motors_output(void)
**���� : �������
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void motors_output(void)
{
    throttle_control();

#if VEHICLE_FRAME == QUAD
    motor_duty[0] = thrust.throttle - thrust.pitch - thrust.roll + thrust.yaw;
    motor_duty[1] = thrust.throttle - thrust.pitch + thrust.roll - thrust.yaw;
    motor_duty[2] = thrust.throttle + thrust.pitch + thrust.roll + thrust.yaw;
    motor_duty[3] = thrust.throttle + thrust.pitch - thrust.roll - thrust.yaw;
#elif VEHICLE_FRAME == HEXA
#endif

    motor_update(motor_duty);

#if 0
    if(flag.FlightMode==ULTRASONIC_High || flag.FlightMode==AUTO_High || flag.FlightMode==ACC_High  || flag.FlightMode==ATMOSPHERE_High){
            Moto[0] = thr_value - pitch - roll + yaw;
            Moto[1] = thr_value - pitch + roll - yaw;
            Moto[2] = thr_value + pitch + roll + yaw;
            Moto[3] = thr_value + pitch - roll - yaw;
    }
    else	if(RC_Data.THROTTLE > RC_MINCHECK) {
          date_throttle	= (RC_Data.THROTTLE-MINRCVALUE)/cos(IMU.Roll/RtA)/cos(IMU.Pitch/RtA);

        #ifdef QUADROTOR
            Moto[0] = date_throttle - pitch - roll + yaw + IDLING;
            Moto[1] = date_throttle - pitch + roll - yaw + IDLING;
            Moto[2] = date_throttle + pitch + roll + yaw + IDLING;
            Moto[3] = date_throttle + pitch - roll - yaw + IDLING;
        #elif defined HEXACOPTER
            Moto[0] = date_throttle - pitch + 0.5*roll - yaw + IDLING;
            Moto[1] = date_throttle         +     roll + yaw + IDLING;
            Moto[2] = date_throttle + pitch + 0.5*roll - yaw + IDLING;
            Moto[3] = date_throttle + pitch - 0.5*roll + yaw + IDLING;
            Moto[4] = date_throttle         -     roll - yaw + IDLING;
            Moto[5] = date_throttle - pitch - 0.5*roll + yaw + IDLING;
        #endif
        }
        else
        {
            array_assign(&Moto[0],IDLING,MOTOR_NUM);//������200
            Reset_Integral();//�ڻ�pidȫ�������0
        }

        if(flag.ARMED)
        {
            #ifdef QUADROTOR
                    Moto_duty[0]=Moto[0];
                    Moto_duty[1]=Moto[1];
                    Moto_duty[2]=Moto[2];
                    Moto_duty[3]=Moto[3];

            #elif defined HEXACOPTER
                    Moto_duty[0]=Moto[0];
                    Moto_duty[1]=Moto[1];
                    Moto_duty[2]=Moto[2];
                    Moto_duty[3]=Moto[3];
                    Moto_duty[4]=Moto[4];
                    Moto_duty[5]=Moto[5];

            #endif
#endif
}

void throttle_control(float dt)
{
    thrust.throttle	= (target_throttle - 1000)/cos(ahrs.Roll/RtA)/cos(ahrs.Pitch/RtA);
#if 0
///////////////////////////////////////////////////////////////////////////
    static float thr;
    static float Thr_tmp;
    thr = RC_Data.THROTTLE-1110; //����ֵthr 0 ~ 1000
    Thr_tmp += 10 *3.14f *dt *(thr/250.0f - Thr_tmp); //��ͨ�˲�
    Thr_Weight = LIMIT(Thr_tmp,0,1);    	//��߶ദ�������ݻ��õ����ֵ

///////////////////////////////////////////////////////////////////////////////

    if( thr < 100 )
    {
        Thr_Low = 1;
    }
    else
    {
        Thr_Low = 0;
    }

    #if(CTRL_HEIGHT)

    Height_Ctrl(T,thr);

    thr_value = Thr_Weight *height_ctrl_out;   //ʵ��ʹ��ֵ

    #else
    thr_value = thr;   //ʵ��ʹ��ֵ

    #endif

    thr_value = LIMIT(thr_value,0,10 *MAX_THR *MAX_PWM/100);//�����������Ϊ800����200��ظ���̬����
}

void set_motor_roll(float _thrust)
{
    thrust.roll = _thrust;
}

void set_motor_pitch(float _thrust)
{
    thrust.pitch = _thrust;
}

void set_motor_yaw(float _thrust)
{
    thrust.yaw = _thrust;
}

void set_motor_throttle(float _thrust)
{
    thrust.throttle = _thrust;
}
