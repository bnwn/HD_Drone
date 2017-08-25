#include "PN020Series.h"
#include "motor_control.h"

Thrust thrust = {0};
float target_throttle = 0, trace_throttle = 0;

/*====================================================================================================*/
/*====================================================================================================*
**函数 : motors_output(void)
**功能 : 电机控制
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void motors_output(void)
{
    throttle_control(0.025f);

#if VEHICLE_FRAME == QUAD
    motor_duty[MOTOR1_INDEX] = thrust.throttle + thrust.pitch - thrust.roll + thrust.yaw;
    motor_duty[MOTOR2_INDEX] = thrust.throttle - thrust.pitch + thrust.roll + thrust.yaw;
    motor_duty[MOTOR3_INDEX] = thrust.throttle + thrust.pitch + thrust.roll - thrust.yaw;
    motor_duty[MOTOR4_INDEX] = thrust.throttle - thrust.pitch - thrust.roll - thrust.yaw;
	
//		printf ("%d, %d, %d, %d\n", (int16_t)(motor_duty[MOTOR1_INDEX]*1000), (int16_t)(motor_duty[MOTOR2_INDEX]*1000), (int16_t)(motor_duty[MOTOR3_INDEX]*1000), \
																														(int16_t)(motor_duty[MOTOR4_INDEX]*1000));
#elif VEHICLE_FRAME == HEXA
#endif

    motor_update(motor_duty);
//		PWM_ConfigOutputChannel(PWM, 0, MOTOR_PWM_FREQ, 10);
//    PWM_ConfigOutputChannel(PWM, 1, MOTOR_PWM_FREQ, 10);
//    PWM_ConfigOutputChannel(PWM, 2, MOTOR_PWM_FREQ, 10);
//    PWM_ConfigOutputChannel(PWM, 3, MOTOR_PWM_FREQ, 10);
#if 0
    if(flag.FlightMode==ULTRASONIC_High || flag.FlightMode==AUTO_High || flag.FlightMode==ACC_High  || flag.FlightMode==ATMOSPHERE_High){
            Moto[0] = thr_value - pitch - roll + yaw;
            Moto[1] = thr_value - pitch + roll - yaw;
            Moto[2] = thr_value + pitch + roll + yaw;
            Moto[3] = thr_value + pitch - roll - yaw;
    }
    else	if(RC_Data.THROTTLE > RC_MINCHECK) {
          datej_throttle	= (RC_Data.THROTTLE-MINRCVALUE)/cos(IMU.Roll/RtA)/cos(IMU.Pitch/RtA);

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
            array_assign(&Moto[0],IDLING,MOTOR_NUM);//马达输出200
            Reset_Integral();//内环pid全部输出置0
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
    //thrust.throttle	= (target_throttle - 1000)/cos(ahrs.Roll/RtA)/cos(ahrs.Pitch/RtA);
		//thrust.throttle = target_throttle; 
		thrust.throttle = trace_throttle;
#if 0
///////////////////////////////////////////////////////////////////////////
    static float thr;
    static float Thr_tmp;
    thr = RC_Data.THROTTLE-1110; //油门值thr 0 ~ 1000
    Thr_tmp += 10 *3.14f *dt *(thr/250.0f - Thr_tmp); //低通滤波
    Thr_Weight = LIMIT(Thr_tmp,0,1);    	//后边多处分离数据会用到这个值

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

    thr_value = Thr_Weight *height_ctrl_out;   //实际使用值

    #else
    thr_value = thr;   //实际使用值

    #endif

    thr_value = LIMIT(thr_value,0,10 *MAX_THR *MAX_PWM/100);//限制油门最大为800，留200余地给姿态控制
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

void set_motor_throttle(float _thrust)
{
    thrust.throttle = _thrust;
		//thrust.throttle = trace_throttle;
}

void set_trace_throttle(float _thr)
{
		trace_throttle = _thr;
		
	printf("trace throttle: %.3f\n", trace_throttle);
}

