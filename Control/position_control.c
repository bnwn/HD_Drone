#include "PN020Series.h"
#include "position_control.h"
#include "attitude_control.h"
#include "inertial_nav.h"
#include "rc_channel.h"

_Nav_t pos_target, pos_error;

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
} pos_flags = {0};

// limit flags structure
struct poscontrol_limit_flags {
    uint8_t pos_up      : 1;    // 1 if we have hit the vertical position leash limit while going up
    uint8_t pos_down    : 1;    // 1 if we have hit the vertical position leash limit while going down
    uint8_t vel_up      : 1;    // 1 if we have hit the vertical velocity limit going up
    uint8_t vel_down    : 1;    // 1 if we have hit the vertical velocity limit going down
    uint8_t accel_xy    : 1;    // 1 if we have hit the horizontal accel limit
} pos_limit = {0};

float leash_up_z = POSCONTROL_LEASH_LENGTH_MIN, leash_down_z = POSCONTROL_LEASH_LENGTH_MIN;
float speed_up_cms = POSCONTROL_SPEED_UP, speed_down_cms = POSCONTROL_SPEED_DOWN;
float accel_z_cms = POSCONTROL_ACCEL_Z;
float accel_last_z_cms = 0.0f;
_Vector_Float vel_desired, vel_last, accel_feedforward;
uint32_t last_update_z_ms = 0;

void poscontrol_init_takeoff(void)
{
    float curr_alt = get_inav_alt();

    pos_target.z = curr_alt;

    // freeze feedforward to avoid jump
    pos_flags.freeze_ff_z = true;

    // shift difference between last motor out and hover throttle into accelerometer I
    ctrl_loop.pos_accel.z.integrator = ((1.0f+norm_input_dz(&rc_channels[RC_THROTTLE_CH]))/2.0f - get_throttle_hover())*1000.0f;

    // initialise ekf reset handler
    // init_ekf_z_reset();
}

/* altitude control */
void position_z_controller(void)
{
	uint32_t now = sys_milli();
    uint32_t dt = (last_update_z_ms>0) ? (now-last_update_z_ms) : 0;
    last_update_z_ms = now;
	
	if (dt > POSCONTROL_ACTIVE_TIMEOUT_MS) {
		pos_flags.reset_accel_to_throttle = true;
		pos_flags.reset_rate_to_accel_z = true;
	}
	
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
    float curr_alt = get_inav_alt();

    // clear position limit flags
    pos_limit.pos_up = false;
    pos_limit.pos_down = false;

    // calculate altitude error
    pos_error.z = pos_target.z - curr_alt;

    // do not let target altitude get too far from current altitude
    if (pos_error.z > leash_up_z) {
        pos_target.z = curr_alt + leash_up_z;
        pos_error.z = leash_up_z;
        pos_limit.pos_up = true;
    }
    if (pos_error.z < -leash_down_z) {
        pos_target.z = curr_alt - leash_down_z;
        pos_error.z = -leash_down_z;
        pos_limit.pos_down = true;
    }

    // calculate _vel_target.z using from _pos_error.z using sqrt controller
    pos_target.vz = sqrt_controller(pos_error.z, ctrl_loop.pos.z.kp, accel_z_cms);

    // check speed limits
    // To-Do: check these speed limits here or in the pos->rate controller
    pos_limit.vel_up = false;
    pos_limit.vel_down = false;
    if (pos_target.vz < speed_down_cms) {
        pos_target.vz = speed_down_cms;
        pos_limit.vel_down = true;
    }
    if (pos_target.vz > speed_up_cms) {
        pos_target.vz = speed_up_cms;
        pos_limit.vel_up = true;
    }

    // add feed forward component
    if (pos_flags.use_desvel_ff_z) {
        pos_target.vz += vel_desired.z;
    }

    // call rate based throttle controller which will update accel based throttle controller targets
    rate_to_accel_z();
}

// rate_to_accel_z - calculates desired accel required to achieve the velocity target
// calculates desired acceleration and calls accel throttle controller
void rate_to_accel_z(void)
{
    const _Vector_Float curr_vel = get_inav_velocity();
    float p;                                // used to capture pid values for logging
	static float last_vel_error = 0.0f;
	
	static uint32_t timestamp = 0;
	uint32_t now = sys_micro();
	float dt = (timestamp>0) ? ((float)(now-timestamp)/1000000.0f) : 0;
	timestamp = now;
	
    // reset last velocity target to current target
    if (pos_flags.reset_rate_to_accel_z) {
        vel_last.z = pos_target.vz;
    }

    // feed forward desired acceleration calculation
    if (dt > 0.0f) {
        if (!pos_flags.freeze_ff_z) {
            accel_feedforward.z = (pos_target.vz - vel_last.z) / dt;
        } else {
            // stop the feed forward being calculated during a known discontinuity
            pos_flags.freeze_ff_z = false;
        }
    } else {
        accel_feedforward.z = 0.0f;
    }

    // store this iteration's velocities for the next iteration
    vel_last.z = pos_target.vz;

    // reset velocity error and filter if this controller has just been engaged
    if (pos_flags.reset_rate_to_accel_z) {
        // Reset Filter
        pos_error.vz = 0;
		last_vel_error = 0.0f;
        pos_flags.reset_rate_to_accel_z = false;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz, 100Hz loop
        pos_error.vz = LPF_1st(last_vel_error, pos_target.vz - curr_vel.z, 0.059f);
		last_vel_error = pos_error.vz;
    }

    // calculate p
    p = ctrl_loop.pos_vel.z.kp * pos_error.vz;

    // consolidate and constrain_float target acceleration
    pos_target.az = accel_feedforward.z + p;

    // set target for accel based throttle controller
    accel_to_throttle(pos_target.az);
}

// accel_to_throttle - alt hold's acceleration controller
// calculates a desired throttle which is sent directly to the motors
void accel_to_throttle(float accel_target_z)
{
    float z_accel_meas;         // actual acceleration
    float p,i,d;              // used to capture pid values for logging
	float thr_out;

    // Calculate Earth Frame Z acceleration
    z_accel_meas = get_inav_accel().z;

    // reset target altitude if this controller has just been engaged
    if (pos_flags.reset_accel_to_throttle) {
        // Reset Filter
        pos_error.az = 0;
        pos_flags.reset_accel_to_throttle = false;
    } else {
        // calculate accel error
        pos_error.az = accel_target_z - z_accel_meas;
    }

    // set input to PID
	set_pid_input(&ctrl_loop.pos_accel.z, pos_error.az);

    // separately calculate p, i, d values for logging
    p = ctrl_loop.pos_accel.z.P_Item_Output;

    // get i term
    i = get_integrator(&ctrl_loop.pos_accel.z);

    // ensure imax is always large enough to overpower hover throttle
    if (get_throttle_hover() * 1000.0f > ctrl_loop.pos_accel.z.imax) {
        ctrl_loop.pos_accel.z.imax = get_throttle_hover() * 1000.0f;
    }
	
	if ((i > 0 && pos_error.az < 0) || (i < 0 && pos_error.az > 0)) {
		i = get_i_output(&ctrl_loop.pos_accel.z);
	}
    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    // To-Do: should this be replaced with limits check from attitude_controller?
//    if ((!_motors.limit.throttle_lower && !_motors.limit.throttle_upper) || (i>0&&_accel_error.z<0) || (i<0&&_accel_error.z>0)) {
//        i = _pid_accel_z.get_i();
//    }

    // get d term
    d = ctrl_loop.pos_accel.z.D_Item_Output;

    thr_out = (p+i+d)/1000.0f + get_throttle_hover();

    // send throttle to attitude controller with angle boost
    attitude_throttle_controller(thr_out, true, POSCONTROL_THROTTLE_CUTOFF_FREQ);
}

/// add_takeoff_climb_rate - adjusts alt target up or down using a climb rate in cm/s
///     should be called continuously (with dt set to be the expected time between calls)
///     almost no checks are performed on the input
void add_takeoff_climb_rate(float _climb_rate_cms, float _dt)
{
    pos_target.z += _climb_rate_cms * _dt;
}

/// set_alt_target_from_climb_rate_ff - adjusts target up or down using a climb rate in cm/s using feed-forward
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded
///     set force_descend to true during landing to allow target to move low enough to slow the motors
void set_alt_target_from_climb_rate_ff(float climb_rate_cms, float dt, bool force_descend)
{
    // calculated increased maximum acceleration if over speed
    float _accel_z_cms = accel_z_cms;
	float vel_change_limit;
	float accel_z_max;
	float jerk_z;
	
    if (vel_desired.z < speed_down_cms && (speed_down_cms != 0.0f)) {
        _accel_z_cms *= POSCONTROL_OVERSPEED_GAIN_Z * vel_desired.z / speed_down_cms;
    }
    if (vel_desired.z > speed_up_cms && (speed_down_cms != 0.0f)) {
        _accel_z_cms *= POSCONTROL_OVERSPEED_GAIN_Z * vel_desired.z / speed_up_cms;
    }
    _accel_z_cms = (float)constrain_float(_accel_z_cms, 0.0f, 750.0f);

    // jerk_z is calculated to reach full acceleration in 1000ms.
    jerk_z = _accel_z_cms * POSCONTROL_JERK_RATIO;

    accel_z_max = MIN(_accel_z_cms, sqrt(2.0f*fabsf(vel_desired.z - climb_rate_cms)*jerk_z));

    accel_last_z_cms += jerk_z * dt;
    accel_last_z_cms = MIN(accel_z_max, accel_last_z_cms);

    vel_change_limit = accel_last_z_cms * dt;
    vel_desired.z = constrain_float(climb_rate_cms, vel_desired.z-vel_change_limit, vel_desired.z+vel_change_limit);
    pos_flags.use_desvel_ff_z = true;

    // adjust desired alt if motors have not hit their limits
    // To-Do: add check of _limit.pos_down?
    if ((vel_desired.z<0 && (!motor.limit_throttle_lower || force_descend)) || (vel_desired.z>0 && !motor.limit_throttle_upper && pos_limit.pos_up)) {
        pos_target.z += vel_desired.z * dt;
    }
}

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
uint8_t satZ=0,satXY=0;	//是否过饱和


#define ALT_LIMIT							2.0f		//限高 3.5
uint8_t isAltLimit=0;
float altLand;



//函数名：estimateHoverThru()
//输入：无
//输出: 预估得到的悬停油门基准值
//描述：预估悬停油门基准值，直接影响到该飞行器的z轴悬停
//悬停油门值相关因素有：电池电压
//Get a estimated value for hold throttle.It will have a direct affection on hover
//Battery voltage
float estimateHoverThru(void){
    float hoverHru = -0.55f;

    //电池电压检测
    Battery.BatteryAD  = GetBatteryAD();
    Battery.BatteryVal = Battery.Bat_K * (Battery.BatteryAD/4096.0) * Battery.ADRef;//实际电压 值计算

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


//函数名：estimateMinThru()
//输入：无
//输出: 预估得到的最小油门值
//描述：预估最小油门值，根据机重、电池电量而定
//油门过小，下降速度过大时，导致失衡，例如快速下降时机身晃动厉害。再增加fuzzy control ，在油门小时用更大的姿态参数
//相关因素有：电池电压
float estimateMinThru(void){
    float minThru = -0.55f;

    //电池电压检测
    Battery.BatteryAD  = GetBatteryAD();
    Battery.BatteryVal = Battery.Bat_K * (Battery.BatteryAD/4096.0) * Battery.ADRef;//实际电压 值计算

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
//保证dt运算不能被打断，保持更新，否则dt过大，积分爆满。
if(tPrev==0){
        tPrev=micros();
        return;
}else{
        t=micros();
        dt=(t-tPrev) /1000000.0f;
        tPrev=t;
}

//only in climb rate mode and landind mode. now we don't work on manual mode
//手动模式不使用该高度控制算法
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

//限高
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

//获取一个预估的Z轴悬停基准值，相关因素有电池电压
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

//限制最小下降油门
minThrust = estimateMinThru();
if(altCtrlMode!=LANDING){
    if (-thrustZSp < minThrust){
        thrustZSp = -minThrust;
    }
}

//与动力分配相关	testing
satXY=0;
satZ=0;
thrustXYSp[0]= sinf(RC_DATA.ROOL * M_PI_F /180.0f) ;//目标角度转加速度
thrustXYSp[1]= sinf(RC_DATA.PITCH * M_PI_F /180.0f) ; 	//归一化
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
if(!satZ )//&& fabsf(thrustZSp)<THR_MAX
{
        thrustZInt += velZErr * alt_vel_PID.I * dt;
        if (thrustZInt > 0.0f)
                        thrustZInt = 0.0f;
}


#endif
void relax_alt_controller(float _throttle_setting)
{
    pos_target.z = get_inav_alt();
    vel_desired.z = 0.0f;
    pos_flags.use_desvel_ff_z = false;
    pos_target.vz = get_inav_velocity().z;
    vel_last.z = get_inav_velocity().z;
    accel_feedforward.z = 0.0f;
    accel_last_z_cms = 0.0f;
    pos_target.az = get_inav_accel().z;
    pos_flags.reset_accel_to_throttle = true;

    ctrl_loop.pos_accel.z.integrator = (_throttle_setting-get_throttle_hover())*1000.0f;
}

/// calc_leash_length - calculates the vertical leash lengths from maximum speed, acceleration
///     called by pos_to_rate_z if z-axis speed or accelerations are changed
void calc_leash_length_z(void)
{
    if (pos_flags.recalc_leash_z) {
        leash_up_z = calc_leash_length(speed_up_cms, accel_z_cms, ctrl_loop.pos.z.kp);
        leash_down_z = calc_leash_length(-speed_down_cms, accel_z_cms, ctrl_loop.pos.z.kp);
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

// Proportional controller with piecewise sqrt sections to constrain second derivative
float sqrt_controller(float error, float p, float second_ord_lim)
{
	float linear_dist;
    if (second_ord_lim < 0.0f || (second_ord_lim == 0.0f) || (p == 0.0f)) {
        return error*p;
    }

    linear_dist = second_ord_lim / (p * p);

    if (error > linear_dist) {
        return sqrtf(2.0f*second_ord_lim*(error-(linear_dist/2.0f)));
    } else if (error < -linear_dist) {
        return sqrtf(2.0f*second_ord_lim*(-error-(linear_dist/2.0f)));
    } else {
        return error*p;
    }
}

/// set_speed_z - sets maximum climb and descent rates
/// To-Do: call this in the main code as part of flight mode initialisation
///     calc_leash_length_z should be called afterwards
///     speed_down should be a negative number
void set_speed_z(float _speed_down, float _speed_up)
{
    _speed_down = -fabsf(_speed_down);

    if ((fabsf(speed_down_cms-_speed_down) > 1.0f) || (fabsf(speed_up_cms-_speed_up) > 1.0f)) {
        speed_down_cms = _speed_down;
        speed_up_cms = _speed_up;
        pos_flags.recalc_leash_z = true;
        calc_leash_length_z();
    }
}

/// set_accel_z - set vertical acceleration in cm/s/s
void set_accel_z(float _accel_cmss)
{
    if (fabsf(accel_z_cms-_accel_cmss) > 1.0f) {
        accel_z_cms = _accel_cmss;
        pos_flags.recalc_leash_z = true;
        calc_leash_length_z();
    }
}

bool is_active_z(void)
{
    return ((sys_milli() - last_update_z_ms) <= POSCONTROL_ACTIVE_TIMEOUT_MS);
}

void set_alt_target_to_current_alt(void)
{
    pos_target.z = get_inav_alt();
}

void set_desired_vel_z(float _vel_z_cms)
{
    pos_target.vz = _vel_z_cms;
}
