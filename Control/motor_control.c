#include "PN020Series.h"
#include "motor_control.h"
#include "Algorithm_filter.h"
#include "common.h"

Thrust thrust = {0};
float target_throttle = 0, trace_throttle = 0;

/*====================================================================================================*/
/*====================================================================================================*
**���� : motors_output(void)
**���� : �������
**���� : None
**?�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void motors_output(void)
{
    static float old_desired_throttle = 0;
    if (thrust.thr_cutoff > 0.01f) {
        thrust.throttle = LPF_1st(old_desired_throttle, thrust.throttle, thrust.thr_cutoff);
        old_desired_throttle = thrust.throttle;
    } else if (old_desired_throttle != 0.0f){
        old_desired_throttle = 0.0f;
    }
#if VEHICLE_FRAME == QUAD
    motor_duty[MOTOR1_INDEX] = thrust.throttle + thrust.pitch - thrust.roll + thrust.yaw;
    motor_duty[MOTOR2_INDEX] = thrust.throttle - thrust.pitch + thrust.roll + thrust.yaw;
    motor_duty[MOTOR3_INDEX] = thrust.throttle + thrust.pitch + thrust.roll - thrust.yaw;
    motor_duty[MOTOR4_INDEX] = thrust.throttle - thrust.pitch - thrust.roll - thrust.yaw;

#ifdef __DEVELOP__
//		printf ("%d, %d, %d, %d\n", (int16_t)(motor_duty[MOTOR1_INDEX]*1000), (int16_t)(motor_duty[MOTOR2_INDEX]*1000), (int16_t)(motor_duty[MOTOR3_INDEX]*1000), \
																														(int16_t)(motor_duty[MOTOR4_INDEX]*1000));
#endif
#elif VEHICLE_FRAME == HEXA
#endif

    motor_update(motor_duty);
}

void throttle_control(float dt)
{
    //thrust.throttle	= (target_throttle - 1000)/cos(ahrs.Roll/RtA)/cos(ahrs.Pitch/RtA);
		//thrust.throttle = target_throttle; 
		thrust.throttle = trace_throttle;
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
#endif
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

void set_motor_throttle(float _thrust, float _cutoff)
{
    thrust.throttle = _thrust;
    thrust.thr_cutoff = _cutoff;
}

void set_trace_throttle(float _thr)
{
		trace_throttle = _thr;
#ifdef __DEVELOP__
	printf("trace throttle: %.3f\n", trace_throttle);
#endif
}

