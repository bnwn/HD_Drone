#include "PN020Series.h"
#include "position_control.h"

// general purpose flags
struct poscontrol_flags {
    uint16_t recalc_leash_z     : 1;    // 1 if we should recalculate the z axis leash length
    uint16_t recalc_leash_xy    : 1;    // 1 if we should recalculate the xy axis leash length
    uint16_t reset_desired_vel_to_pos   : 1;    // 1 if we should reset the rate_to_accel_xy step
    uint16_t reset_rate_to_accel_xy     : 1;    // 1 if we should reset the rate_to_accel_xy step
    uint16_t reset_accel_to_lean_xy     : 1;    // 1 if we should reset the accel to lean angle step
    uint16_t reset_rate_to_accel_z      : 1;    // 1 if we should reset the rate_to_accel_z step
    uint16_t reset_accel_to_throttle    : 1;    // 1 if we should reset the accel_to_throttle step of the z-axis controller
    uint16_t freeze_ff_xy       : 1;    // 1 use to freeze feed forward during step updates
    uint16_t freeze_ff_z        : 1;    // 1 used to freeze velocity to accel feed forward for one iteration
    uint16_t use_desvel_ff_z    : 1;    // 1 to use z-axis desired velocity as feed forward into velocity step
    uint16_t vehicle_horiz_vel_override : 1; // 1 if we should use _vehicle_horiz_vel as our velocity process variable for one timestep
} pos_flags;

// limit flags structure
struct poscontrol_limit_flags {
    uint8_t pos_up      : 1;    // 1 if we have hit the vertical position leash limit while going up
    uint8_t pos_down    : 1;    // 1 if we have hit the vertical position leash limit while going down
    uint8_t vel_up      : 1;    // 1 if we have hit the vertical velocity limit going up
    uint8_t vel_down    : 1;    // 1 if we have hit the vertical velocity limit going down
    uint8_t accel_xy    : 1;    // 1 if we have hit the horizontal accel limit
} pos_limit;


#if 0
//cut deadband, move linear
float dbScaleLinear(float x, float x_end, float deadband)
{
    if (x > deadband) {
        return (x - deadband) / (x_end - deadband);

    } else if (x < -deadband) {
        return (x + deadband) / (x_end - deadband);

    } else {
        return 0.0f;
    }
}


float thrInit;

#define ALT_FEED_FORWARD  		0.5f
#define THR_MAX								1.0f		//max thrust
#define TILT_MAX 					(Angle_Max * M_PI_F / 180.0 )
const float ALT_CTRL_Z_DB = 0.1f;	//
float spZMoveRate;

uint8_t altCtrlMode;					//normal=0  CLIMB rate, normal .  tobe tested
float hoverThrust=0;
uint8_t zIntReset=1;	//integral reset at first . when change manual mode to climb rate mode
float thrustZInt=0, thrustZSp=0;
float thrustXYSp[2]={0,0};	//roll pitch
uint8_t recAltFlag=0;
float holdAlt=0;
uint8_t satZ=0,satXY=0;	//�Ƿ������


#define ALT_LIMIT							2.0f		//�޸� 3.5
uint8_t isAltLimit=0;
float altLand;



//��������estimateHoverThru()
//���룺��
//���: Ԥ���õ�����ͣ���Ż�׼ֵ
//������Ԥ����ͣ���Ż�׼ֵ��ֱ��Ӱ�쵽�÷�������z����ͣ
//��ͣ����ֵ��������У���ص�ѹ
//Get a estimated value for hold throttle.It will have a direct affection on hover
//Battery voltage
float estimateHoverThru(void){
    float hoverHru = -0.55f;

    //��ص�ѹ���
    Battery.BatteryAD  = GetBatteryAD();
    Battery.BatteryVal = Battery.Bat_K * (Battery.BatteryAD/4096.0) * Battery.ADRef;//ʵ�ʵ�ѹ ֵ����

    if(Battery.BatteryVal > 4.05){
        hoverHru = -0.25f;
    }else if(Battery.BatteryVal > 3.90){
        hoverHru = -0.40f;
    }else if(Battery.BatteryVal > 3.80){
        hoverHru = -0.45f;
    }else if(Battery.BatteryVal > 3.70){
        hoverHru = -0.50f;
    }else{
        hoverHru = -0.55f;
    }


//	if(Battery.BatteryVal > 4.05){
//		hoverHru = -0.05f;
//	}else if(Battery.BatteryVal > 3.90){
//		hoverHru = -0.10f;
//	}else if(Battery.BatteryVal > 3.80){
//		hoverHru = -0.15f;
//	}else if(Battery.BatteryVal > 3.70){
//		hoverHru = -0.20f;
//	}else{
//		hoverHru = -0.25f;
//	}

    return hoverHru;
}


//��������estimateMinThru()
//���룺��
//���: Ԥ���õ�����С����ֵ
//������Ԥ����С����ֵ�����ݻ��ء���ص�������
//���Ź�С���½��ٶȹ���ʱ������ʧ�⣬��������½�ʱ����ζ�������������fuzzy control ��������Сʱ�ø������̬����
//��������У���ص�ѹ
float estimateMinThru(void){
    float minThru = -0.55f;

    //��ص�ѹ���
    Battery.BatteryAD  = GetBatteryAD();
    Battery.BatteryVal = Battery.Bat_K * (Battery.BatteryAD/4096.0) * Battery.ADRef;//ʵ�ʵ�ѹ ֵ����

    if(Battery.BatteryVal > 4.05){
        minThru = -0.30f;
    }else if(Battery.BatteryVal > 3.90){
        minThru = -0.40f;
    }else{
        minThru = -0.55f;
    }

    return minThru;
}


float manThr=0,alt=0,velZ=0;
float altSp=0;
float posZVelSp=0;
float altSpOffset,altSpOffsetMax=0;
float dt=0,t=0;
static float tPrev=0,velZPrev=0;
float posZErr=0,velZErr=0,valZErrD=0;
float thrustXYSpLen=0,thrustSpLen=0;
float thrustXYMax=0;
float minThrust;

//get dt
//��֤dt���㲻�ܱ���ϣ����ָ��£�����dt���󣬻��ֱ�����
if(tPrev==0){
        tPrev=micros();
        return;
}else{
        t=micros();
        dt=(t-tPrev) /1000000.0f;
        tPrev=t;
}

//only in climb rate mode and landind mode. now we don't work on manual mode
//�ֶ�ģʽ��ʹ�øø߶ȿ����㷨
if(MANUAL == altCtrlMode || !FLY_ENABLE){
    return;
}

//--------------pos z ctrol---------------//
//get current alt
alt=-nav.z;
//get desired move rate from stick
manThr=RC_DATA.THROTTLE / 1000.0f;
spZMoveRate= -dbScaleLinear(manThr-0.5f,0.5f,ALT_CTRL_Z_DB);	// scale to -1~1 . NED frame
spZMoveRate = spZMoveRate * ALT_VEL_MAX;	// scale to vel min max

//get alt setpoint in CLIMB rate mode
altSp 	=-nav.z;						//only alt is not in ned frame.
altSp  -= spZMoveRate * dt;
//limit alt setpoint
altSpOffsetMax=ALT_VEL_MAX / alt_PID.P * 2.0f;
altSpOffset = altSp-alt;
if( altSpOffset > altSpOffsetMax){
    altSp=alt +  altSpOffsetMax;
}else if( altSpOffset < -altSpOffsetMax){
    altSp=alt - altSpOffsetMax;
}

//�޸�
if(isAltLimit)
{
    if(altSp - altLand > ALT_LIMIT)
    {
            altSp=altLand+ALT_LIMIT;
            spZMoveRate=0;
    }
}

// pid and feedforward control . in ned frame
posZErr= -(altSp - alt);
posZVelSp = posZErr * alt_PID.P + spZMoveRate * ALT_FEED_FORWARD;
//consider landing mode
if(altCtrlMode==LANDING)
    posZVelSp = LAND_SPEED;

//��ȡһ��Ԥ����Z����ͣ��׼ֵ����������е�ص�ѹ
//get hold throttle. give it a estimated value
if(zIntReset){
    thrustZInt = estimateHoverThru();
    zIntReset = 0;
}

velZ=nav.vz;
velZErr = posZVelSp - velZ;
valZErrD = (spZMoveRate - velZ) * alt_PID.P - (velZ - velZPrev) / dt;	//spZMoveRate is from manual stick vel control
velZPrev=velZ;

thrustZSp= velZErr * alt_vel_PID.P + valZErrD * alt_vel_PID.D + thrustZInt;	//in ned frame. thrustZInt contains hover thrust

//������С�½�����
minThrust = estimateMinThru();
if(altCtrlMode!=LANDING){
    if (-thrustZSp < minThrust){
        thrustZSp = -minThrust;
    }
}

//�붯���������	testing
satXY=0;
satZ=0;
thrustXYSp[0]= sinf(RC_DATA.ROOL * M_PI_F /180.0f) ;//Ŀ��Ƕ�ת���ٶ�
thrustXYSp[1]= sinf(RC_DATA.PITCH * M_PI_F /180.0f) ; 	//��һ��
thrustXYSpLen= sqrtf(thrustXYSp[0] * thrustXYSp[0] + thrustXYSp[1] * thrustXYSp[1]);
//limit tilt max
if(thrustXYSpLen >0.01f )
{
    thrustXYMax=-thrustZSp * tanf(TILT_MAX);
    if(thrustXYSpLen > thrustXYMax)
    {
            float k=thrustXYMax / thrustXYSpLen;
            thrustXYSp[1] *= k;
            thrustXYSp[0] *= k;
            satXY=1;
            thrustXYSpLen= sqrtf(thrustXYSp[0] * thrustXYSp[0] + thrustXYSp[1] * thrustXYSp[1]);
    }

}
//limit max thrust!!
thrustSpLen=sqrtf(thrustXYSpLen * thrustXYSpLen + thrustZSp * thrustZSp);
if(thrustSpLen > THR_MAX)
{
        if(thrustZSp < 0.0f)	//going up
        {
                    if (-thrustZSp > THR_MAX)
                    {
                            /* thrust Z component is too large, limit it */
                            thrustXYSp[0] = 0.0f;
                            thrustXYSp[1] = 0.0f;
                            thrustZSp = -THR_MAX;
                            satXY = 1;
                            satZ = 1;

                        }
                        else {
                            float k = 0;
                            /* preserve thrust Z component and lower XY, keeping altitude is more important than position */
                            thrustXYMax = sqrtf(THR_MAX * THR_MAX- thrustZSp * thrustZSp);
                            k=thrustXYMax / thrustXYSpLen;
                            thrustXYSp[1] *=k;
                            thrustXYSp[0] *= k;
                            satXY=1;
                        }
        }
        else {		//going down
                        /* Z component is negative, going down, simply limit thrust vector */
                        float k = THR_MAX / thrustSpLen;
                        thrustZSp *= k;
                        thrustXYSp[1] *=k;
                        thrustXYSp[0] *= k;
                        satXY = 1;
                        satZ = 1;
                    }

}
rollSp= asinf(thrustXYSp[0]) * 180.0f /M_PI_F;
pitchSp = asinf(thrustXYSp[1]) * 180.0f /M_PI_F;


// if saturation ,don't integral
if(!satZ )//&& fabs(thrustZSp)<THR_MAX
{
        thrustZInt += velZErr * alt_vel_PID.I * dt;
        if (thrustZInt > 0.0f)
                        thrustZInt = 0.0f;
}
/* altitude control */
void position_z_control(void)
{
    // check if leash lengths need to be recalculated
    calc_leash_length_z();

    // call position controller
    pos_to_rate_z();
}

// pos_to_rate_z - position to rate controller for Z axis
// calculates desired rate in earth-frame z axis and passes to rate controller
// vel_up_max, vel_down_max should have already been set before calling this method
void pos_to_rate_z(void)
{
    float curr_alt = _inav.get_altitude();

    // clear position limit flags
    _limit.pos_up = false;
    _limit.pos_down = false;

    // calculate altitude error
    _pos_error.z = _pos_target.z - curr_alt;

    // do not let target altitude get too far from current altitude
    if (_pos_error.z > _leash_up_z) {
        _pos_target.z = curr_alt + _leash_up_z;
        _pos_error.z = _leash_up_z;
        _limit.pos_up = true;
    }
    if (_pos_error.z < -_leash_down_z) {
        _pos_target.z = curr_alt - _leash_down_z;
        _pos_error.z = -_leash_down_z;
        _limit.pos_down = true;
    }

    // calculate _vel_target.z using from _pos_error.z using sqrt controller
    _vel_target.z = AC_AttitudeControl::sqrt_controller(_pos_error.z, _p_pos_z.kP(), _accel_z_cms);

    // check speed limits
    // To-Do: check these speed limits here or in the pos->rate controller
    _limit.vel_up = false;
    _limit.vel_down = false;
    if (_vel_target.z < _speed_down_cms) {
        _vel_target.z = _speed_down_cms;
        _limit.vel_down = true;
    }
    if (_vel_target.z > _speed_up_cms) {
        _vel_target.z = _speed_up_cms;
        _limit.vel_up = true;
    }

    // add feed forward component
    if (_flags.use_desvel_ff_z) {
        _vel_target.z += _vel_desired.z;
    }

    // call rate based throttle controller which will update accel based throttle controller targets
    rate_to_accel_z();
}

// rate_to_accel_z - calculates desired accel required to achieve the velocity target
// calculates desired acceleration and calls accel throttle controller
void AC_PosControl::rate_to_accel_z(void)
{
    const Vector3f& curr_vel = _inav.get_velocity();
    float p;                                // used to capture pid values for logging

    // reset last velocity target to current target
    if (_flags.reset_rate_to_accel_z) {
        _vel_last.z = _vel_target.z;
    }

    // feed forward desired acceleration calculation
    if (_dt > 0.0f) {
        if (!_flags.freeze_ff_z) {
            _accel_feedforward.z = (_vel_target.z - _vel_last.z)/_dt;
        } else {
            // stop the feed forward being calculated during a known discontinuity
            _flags.freeze_ff_z = false;
        }
    } else {
        _accel_feedforward.z = 0.0f;
    }

    // store this iteration's velocities for the next iteration
    _vel_last.z = _vel_target.z;

    // reset velocity error and filter if this controller has just been engaged
    if (_flags.reset_rate_to_accel_z) {
        // Reset Filter
        _vel_error.z = 0;
        _vel_error_filter.reset(0);
        _flags.reset_rate_to_accel_z = false;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz
        _vel_error.z = _vel_error_filter.apply(_vel_target.z - curr_vel.z, _dt);
    }

    // calculate p
    p = _p_vel_z.kP() * _vel_error.z;

    // consolidate and constrain target acceleration
    _accel_target.z = _accel_feedforward.z + p;

    // set target for accel based throttle controller
    accel_to_throttle(_accel_target.z);
}

// accel_to_throttle - alt hold's acceleration controller
// calculates a desired throttle which is sent directly to the motors
void AC_PosControl::accel_to_throttle(float accel_target_z)
{
    float z_accel_meas;         // actual acceleration
    float p,i,d;              // used to capture pid values for logging

    // Calculate Earth Frame Z acceleration
    z_accel_meas = -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;

    // reset target altitude if this controller has just been engaged
    if (_flags.reset_accel_to_throttle) {
        // Reset Filter
        _accel_error.z = 0;
        _flags.reset_accel_to_throttle = false;
    } else {
        // calculate accel error
        _accel_error.z = accel_target_z - z_accel_meas;
    }

    // set input to PID
    _pid_accel_z.set_input_filter_all(_accel_error.z);
    _pid_accel_z.set_desired_rate(accel_target_z);

    // separately calculate p, i, d values for logging
    p = _pid_accel_z.get_p();

    // get i term
    i = _pid_accel_z.get_integrator();

    // ensure imax is always large enough to overpower hover throttle
    if (_motors.get_throttle_hover() * 1000.0f > _pid_accel_z.imax()) {
        _pid_accel_z.imax(_motors.get_throttle_hover() * 1000.0f);
    }

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    // To-Do: should this be replaced with limits check from attitude_controller?
    if ((!_motors.limit.throttle_lower && !_motors.limit.throttle_upper) || (i>0&&_accel_error.z<0) || (i<0&&_accel_error.z>0)) {
        i = _pid_accel_z.get_i();
    }

    // get d term
    d = _pid_accel_z.get_d();

    float thr_out = (p+i+d)/1000.0f +_motors.get_throttle_hover();

    // send throttle to attitude controller with angle boost
    _attitude_control.set_throttle_out(thr_out, true, POSCONTROL_THROTTLE_CUTOFF_FREQ);
}

/// calc_leash_length - calculates the vertical leash lengths from maximum speed, acceleration
///     called by pos_to_rate_z if z-axis speed or accelerations are changed
void calc_leash_length_z(void)
{
    if (pos_flags.recalc_leash_z) {
        _leash_up_z = calc_leash_length(_speed_up_cms, _accel_z_cms, _p_pos_z.kP());
        _leash_down_z = calc_leash_length(-_speed_down_cms, _accel_z_cms, _p_pos_z.kP());
        pos_flags.recalc_leash_z = false;
    }
}

/// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration and position kP gain
float calc_leash_length(float speed_cms, float accel_cms, float kP)
{
    float leash_length;

    // sanity check acceleration and avoid divide by zero
    if (accel_cms <= 0.0f) {
        accel_cms = POSCONTROL_ACCELERATION_MIN;
    }

    // avoid divide by zero
    if (kP <= 0.0f) {
        return POSCONTROL_LEASH_LENGTH_MIN;
    }

    // calculate leash length
    if(speed_cms <= accel_cms / kP) {
        // linear leash length based on speed close in
        leash_length = speed_cms / kP;
    }else{
        // leash length grows at sqrt of speed further out
        leash_length = (accel_cms / (2.0f*kP*kP)) + (speed_cms*speed_cms / (2.0f*accel_cms));
    }

    // ensure leash is at least 1m long
    if( leash_length < POSCONTROL_LEASH_LENGTH_MIN ) {
        leash_length = POSCONTROL_LEASH_LENGTH_MIN;
    }

    return leash_length;
}
#endif
