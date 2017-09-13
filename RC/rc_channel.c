#include "PN020Series.h"
#include "rf.h"
#include "rc_channel.h"
#include "Algorithm_math.h"
#include "timer_delay.h"
#include "common.h"
#include "flight_mode_control.h"
#include "position_control.h"

Rc_Channel_t rc_channels[RC_CHANNEL_NUM_MAX] = {0};
Rc_Switch_t rc_switchs[RC_SWITCH_MAX] = {0};
uint8_t rc_buf[PAYLOAD_WIDTH+1] = {0}; 
uint8_t roll_code[ROLL_CODE_NUM] = {0};
extern float cmp_kp, cmp_ki;

void rc_channel_init(void)
{
	uint8_t try_times = 5;
#if RADIO == RF_XNS104
    /* only use spi1 */
    SPI1_Init();
    RF_Init();
    RF_RxMode();
#ifdef __DEVELOP__
    printf("rf xns104 init success\n");
#endif
    for (; try_times>0; try_times--) {
		if (auto_code_matching()) {
			break;
		}
	}
#else
#endif
	setting_rc_channel_all();
}

bool rc_channel_read(void)
{
    static uint8_t buf_index = 0;
    uint8_t rev_len = PAYLOAD_WIDTH - buf_index;
    static uint32_t timestamp = 0;
    uint32_t now = sys_milli();
    uint8_t j = 0;
	
	if (!fc_status.code_matched) {
		auto_code_matching();
		return false;
	}

    if ((now - timestamp) > RADIO_SIGNAL_LOST_MS) {
        set_radio_lost(true);
    }

	memset(rc_buf, 0, PAYLOAD_WIDTH);
    ucRF_DumpRxData(rc_buf + buf_index, rev_len);

#ifdef __DEVELOP__
//		for(; j<PAYLOAD_WIDTH; j++)
//		{
//				printf("0x%X ", rc_buf[j]);
//		}
//		printf("\n");
#endif
	
#if 1
    // if (roll_code eq)
    if ((rc_buf[START_CODE_INDEX] == 0xAA) && (rc_buf[START_CODE_INDEX+1] == 0xAA) && (rc_buf[END_CODE_INDEX] == 0xCE) && (rc_buf[END_CODE_INDEX+1] == 0xED)) { // start and end code
			uint8_t roll_code_index = 0;
            for(; roll_code_index<ROLL_CODE_NUM; roll_code_index++) {
                if (rc_buf[ROLL_CODE_INDEX+roll_code_index] != roll_code[roll_code_index]) {
                    return false;
                }
            }
			
            timestamp = sys_milli();
            set_radio_lost(false);

            rc_channels[0].rc_in = (rc_buf[DATA_INDEX] << 8) | rc_buf[DATA_INDEX+1];
            rc_channels[1].rc_in = (rc_buf[DATA_INDEX+2] << 8) | rc_buf[DATA_INDEX+3];
            rc_channels[2].rc_in = (rc_buf[DATA_INDEX+4] << 8) | rc_buf[DATA_INDEX+5];
            rc_channels[3].rc_in = (rc_buf[DATA_INDEX+6] << 8) | rc_buf[DATA_INDEX+7];

            rc_switchs[0].rc_in = (rc_buf[DATA_INDEX+8] & 0x01) ? 1 : 0;
            rc_switchs[1].rc_in = (rc_buf[DATA_INDEX+8] & 0x02) ? 1 : 0;
            rc_switchs[2].rc_in = (rc_buf[DATA_INDEX+8] & 0x04) ? 1 : 0;
            rc_switchs[3].rc_in = (rc_buf[DATA_INDEX+8] & 0x08) ? 1 : 0;
            rc_switchs[4].rc_in = (rc_buf[DATA_INDEX+8] & 0x10) ? 1 : 0;
            rc_switchs[5].rc_in = (rc_buf[DATA_INDEX+8] & 0x20) ? 1 : 0;
            rc_switchs[6].rc_in = (rc_buf[DATA_INDEX+8] & 0x40) ? 1 : 0;
            rc_switchs[7].rc_in = (rc_buf[DATA_INDEX+8] & 0x80) ? 1 : 0;
    } /* else if (rc_buf[START_CODE_INDEX] == 0xAC && rc_buf[START_CODE_INDEX+1] == 0xCE \
               && rc_buf[END_CODE_INDEX] == 0xCE && rc_buf[END_CODE_INDEX+1] == 0xED) { // match and end code
            set_roll_code(rc_buf+ROLL_CODE_INDEX);
    }*/ else {
        return false;
    }
#else
        static uint8_t buf_index = 0, data_buf_index = 0, rc_data_len = 0;
        static uint8_t step = RC_START;
        static bool rc_data_update = false;
        uint8_t rev_len = PAYLOAD_WIDTH - buf_index;
        uint8_t current_index = 0;
        uint8_t j = 0;
		while(current_index < PAYLOAD_WIDTH) {
				switch (step) {
						case RC_START:
								if ((rc_buf[current_index] == 0xAA) && (rc_buf[current_index+1] == 0xAA)) {
										step = RC_ROLL_CODE;
										current_index += 2;
								} else {
										current_index++;	
								}
								break;
						case RC_ROLL_CODE:
								j = 0;
								for (; j<ROLL_CODE_NUM; j++) {
										roll_code[j] = rc_buf[current_index+j];
								}
								current_index += j;
								j = 0;
								step = RC_DATA_LEN;
								break;
						case RC_DATA_LEN:
								rc_data_len = rc_buf[current_index];
								if (rc_data_len <= DATA_BUF_MAX)
								{
										step = RC_DATA;
										current_index++;
								} else {
										step = RC_START;
								}
								break;
						case RC_DATA:
								data_buf[data_buf_index++] = rc_buf[current_index];
								if (data_buf_index == rc_data_len)
								{
										data_buf_index = 0;
										step = RC_END;
								}
								current_index++;
								break;
						case RC_END:
								if ((rc_buf[current_index] == 0xCE) && (rc_buf[current_index+1] == 0xED)) {
										rc_data_update = true;
										current_index += 2;
								} else {
										current_index++;
								}
								step = RC_START;
								break;
						default:
								break;
				}
							
				if (rc_data_update) {
						int i = 0;
						rc_data_update = false;
					
						/* rc channel handle */
						packet_parse();
						switch_handle();
					
						buf_index = PAYLOAD_WIDTH - current_index;
						for(; i<buf_index; i++) {
								rc_buf[i] = rc_buf[current_index+i];
						}
						current_index = 0;
						break;
				}
		}
#endif
    return true;
}

#if 0
static void packet_parse(void)
{
    rc_channels[0].rc_in = (data_buf[0] << 8) | data_buf[1];
    rc_channels[1].rc_in = (data_buf[2] << 8) | data_buf[3];
    rc_channels[2].rc_in = (data_buf[4] << 8) | data_buf[5];
    rc_channels[3].rc_in = (data_buf[6] << 8) | data_buf[7];

    rc_switchs[0].rc_in = (data_buf[8] & 0x01) ? 1 : 0;
    rc_switchs[1].rc_in = (data_buf[8] & 0x02) ? 1 : 0;
    rc_switchs[2].rc_in = (data_buf[8] & 0x04) ? 1 : 0;
    rc_switchs[3].rc_in = (data_buf[8] & 0x08) ? 1 : 0;
    rc_switchs[4].rc_in = (data_buf[8] & 0x10) ? 1 : 0;
    rc_switchs[5].rc_in = (data_buf[8] & 0x20) ? 1 : 0;
    rc_switchs[6].rc_in = (data_buf[8] & 0x40) ? 1 : 0;
    rc_switchs[7].rc_in = (data_buf[8] & 0x80) ? 1 : 0;
}
#endif

void switch_handle(void)
{
    static uint8_t switch_on_tick[RC_SWITCH_MAX] = {0};
    uint8_t ch_index = 0;
//				pre_switch = data_buf[8];
	
    for (; ch_index < RC_SWITCH_MAX; ch_index++) {
        if (rc_switchs[ch_index].rc_in) {
            switch_on_tick[ch_index]++;
        } else if (switch_on_tick[ch_index] > 0) {
            switch_event_trigger(ch_index, (switch_on_tick[ch_index] >= 100));
            switch_on_tick[ch_index] = 0;
        }
    }
}

static void switch_event_trigger(uint8_t _ch, bool _is_long_hold)
{
    switch(_ch) {
        case 0:
			set_pid_param_p(&ctrl_loop.rate.roll, (ctrl_loop.rate.roll.kp + 0.01f));
			set_pid_param_p(&ctrl_loop.rate.pitch, (ctrl_loop.rate.pitch.kp + 0.01f));
			printf("set_pid_param_p:%i", (int16_t)(ctrl_loop.rate.pitch.kp * 100));
            break;
        case 1:
			set_pid_param_p(&ctrl_loop.rate.roll, (ctrl_loop.rate.roll.kp - 0.01f));
			set_pid_param_p(&ctrl_loop.rate.pitch, (ctrl_loop.rate.pitch.kp - 0.01f));
			printf("set_pid_param_p:%i", (int16_t)(ctrl_loop.rate.pitch.kp*100));
            break;
//		 case 0:
//			set_pid_param_p(&ctrl_loop.angle.roll, (ctrl_loop.angle.roll.kp + 5.0f));
//			set_pid_param_p(&ctrl_loop.angle.pitch, (ctrl_loop.angle.pitch.kp + 5.0f));
//			printf("set_pid_param_p:%i", (int16_t)(ctrl_loop.angle.pitch.kp));
//            break;
//        case 1:
//			set_pid_param_p(&ctrl_loop.angle.roll, (ctrl_loop.angle.roll.kp - 5.0f));
//			set_pid_param_p(&ctrl_loop.angle.pitch, (ctrl_loop.angle.pitch.kp - 5.0f));
//			printf("set_pid_param_p:%i", (int16_t)(ctrl_loop.angle.pitch.kp));
//            break;
		case 7:
			set_pid_param_i(&ctrl_loop.rate.roll, (ctrl_loop.rate.roll.ki + 0.01f));
			set_pid_param_i(&ctrl_loop.rate.pitch, (ctrl_loop.rate.pitch.ki + 0.01f));
			printf("set_pid_param_i:%i", (int16_t)(ctrl_loop.rate.pitch.ki * 100));
            break;
        case 2:
			set_pid_param_i(&ctrl_loop.rate.roll, (ctrl_loop.rate.roll.ki - 0.01f));
			set_pid_param_i(&ctrl_loop.rate.pitch, (ctrl_loop.rate.pitch.ki - 0.01f));
			printf("set_pid_param_i:%i", (int16_t)(ctrl_loop.rate.pitch.ki*100));
            break;
//        case 2:
//			if (fc_status.printf_flag != 255) fc_status.printf_flag = 255;
//			else {
//				fc_status.printf_flag = 0x0;
//				
//                fc_status.armed = DISARMED;
//			}
		
//			fc_status.printf_flag--;
//			if (fc_status.printf_flag < 0) fc_status.printf_flag = 8;
//            break;
        case 3:
            fc_status.armed = DISARMED;
			set_land_complete(true);
            break;
        case 4:
//			set_flight_mode(Acro);
			if (set_flight_mode(AltHold))
				printf("set AltHold mode.\n");
            break;
        case 5:
			set_pid_param_d(&ctrl_loop.rate.roll, (ctrl_loop.rate.roll.kd + 0.0005f));
			set_pid_param_d(&ctrl_loop.rate.pitch, (ctrl_loop.rate.pitch.kd + 0.0005f));
			printf("set_pid_param_d:%d", (int16_t)(ctrl_loop.rate.pitch.kd*10000));
            break;
        case 6:
			set_pid_param_d(&ctrl_loop.rate.roll, (ctrl_loop.rate.roll.kd - 0.0005f));
			set_pid_param_d(&ctrl_loop.rate.pitch, (ctrl_loop.rate.pitch.kd - 0.0005f));
			printf("set_pid_param_d:%d", (int16_t)(ctrl_loop.rate.pitch.kd*10000));
		
//			if (set_flight_mode(Stabilize))
//				printf("set Stabilize mode.\n");
            break;
//        case 7:
//			fc_status.printf_flag++;
//			if (fc_status.printf_flag >= 9) fc_status.printf_flag = 0;
//            break;
        default:
            break;
    }
}

float norm_input(Rc_Channel_t *_rc)
{
    float ret;
    int16_t reverse_mul = (_rc->reversed ? -1:1);

    if (_rc->rc_in < _rc->rc_neutral) {
        if (_rc->rc_min >= _rc->rc_neutral) {
            return 0.0f;
        }
        ret = reverse_mul * (float)(_rc->rc_in - _rc->rc_neutral) / (float)(_rc->rc_neutral - _rc->rc_min);
    } else {
        if (_rc->rc_max <= _rc->rc_neutral) {
            return 0.0f;
        }
        ret = reverse_mul * (float)(_rc->rc_in - _rc->rc_neutral) / (float)(_rc->rc_max - _rc->rc_neutral);
    }

    return (float)constrain_float(ret, -1.0f, 1.0f);
}

float norm_input_dz(Rc_Channel_t *_rc)
{
    float ret;
    int16_t reverse_mul = (_rc->reversed ? -1:1);
    int16_t dz_min = _rc->rc_neutral - _rc->dead_zone;
    int16_t dz_max = _rc->rc_neutral + _rc->dead_zone;

    if (_rc->rc_in < dz_min && dz_min > _rc->rc_min) {
        ret = reverse_mul * (float)(_rc->rc_in - dz_min) / (float)(dz_min - _rc->rc_min);
    } else if (_rc->rc_in > dz_max && _rc->rc_max > dz_max) {
        ret = reverse_mul * (float)(_rc->rc_in - dz_max) / (float)(_rc->rc_max - dz_max);
    } else {
        ret = 0.0f;
    }

    return (float)constrain_float(ret, -1.0f, 1.0f);
}

float channel_input_to_target(Rc_Channel_t *_rc, float _max_range)
{
    return (float)(norm_input_dz(_rc) * _max_range);
}

void set_roll_code(uint8_t *_code)
{
    uint8_t i = 0;
    for (; i<ROLL_CODE_NUM; i++) {
        roll_code[i] = *(_code + i);
    }
#ifdef __DEVELOP__
    printf("roll code: 0x%X 0x%X 0x%X\n", roll_code[0], roll_code[1], roll_code[2]);
#endif
}

void get_desired_leans_angles(_Target_Attitude *_target_att, float _leans_limit)
{
    float _roll, _pitch, _yaw_stick_angle, _squa_angle;

    if (_leans_limit > ATT_ROLL_PITCH_YAW_MAX) {
        _leans_limit = ATT_ROLL_PITCH_YAW_MAX;
    }

    _roll = channel_input_to_target(&rc_channels[RC_EULER_ROLL_CH], _leans_limit);
    _pitch = channel_input_to_target(&rc_channels[RC_EULER_PITCH_CH], _leans_limit);
    _squa_angle = squa(_roll) + squa(_pitch);

    if (_squa_angle > squa(_leans_limit)) {
        float _norm = Q_rsqrt(_squa_angle);
        float _ratio = _leans_limit * _norm;
        _roll *= _ratio;
        _pitch *= _ratio;
    }

    _target_att->roll = (180/M_PI) * atanf(cosf(_pitch*(M_PI/180))*tanf(_roll*(M_PI/180)));
//	_target_att->roll = _roll;
    _target_att->pitch = _pitch;

    _yaw_stick_angle = channel_input_to_target(&rc_channels[RC_EULER_YAW_CH], _leans_limit);
    // calculate yaw rate
    _target_att->yaw = _yaw_stick_angle * ctrl_loop.acro_sensibility.yaw.kp;
}

float get_desired_throttle_expo(void)
{
	float throttle_out;
    if (fc_status.radio_lost) {
        return 0.0f;
    }

    throttle_out = (float)((norm_input_dz(&rc_channels[RC_THROTTLE_CH]) + 1.0f) / 2.0f);

    // calculate the output throttle using the given expo function
    throttle_out = throttle_out * 0.6 + 0.4 * throttle_out * throttle_out * throttle_out;
	
    throttle_out = (float)constrain_float(throttle_out, 0.0f, RC_THROTTLE_OUT_LIMIT);
    return throttle_out;
}

// get_desired_climb_rate - transform pilot's throttle input to climb rate in cm/s
// without any deadzone at the bottom
float get_desired_climb_rate(void)
{
    float desired_rate = 0.0f;
    float thr_in = (float)((norm_input_dz(&rc_channels[RC_THROTTLE_CH]) + 1.0f) / 2.0f) * 1000.0f;
    float deadband_top = 500.0f + THR_DZ_DEFAULT;
    float deadband_bottom = 500.0f - THR_DZ_DEFAULT;
	
    // throttle failsafe check
    if (fc_status.radio_lost) {
        return 0.0f;
    }

    // ensure a reasonable throttle value
    thr_in = constrain_float(thr_in, 0.0f, 1000.0f);

    // check throttle is above, below or in the deadband
    if (thr_in < deadband_bottom) {
        // below the deadband
        desired_rate = POSCONTROL_VELZ_MAX * (thr_in-deadband_bottom) / deadband_bottom;
    }else if (thr_in > deadband_top) {
        // above the deadband
        desired_rate = POSCONTROL_VELZ_MAX * (thr_in-deadband_top) / (1000.0f-deadband_top);
    }else{
        // must be in the deadband
        desired_rate = 0.0f;
    }

    return desired_rate;
}

bool auto_code_matching(void)
{
	ucRF_DumpRxData(rc_buf, PAYLOAD_WIDTH);

	if (rc_buf[START_CODE_INDEX] == 0xAA && rc_buf[START_CODE_INDEX+1] == 0xAA) {
		if (rc_buf[END_CODE_INDEX] == 0xCE && rc_buf[END_CODE_INDEX+1] == 0xED) {
			//if () // setting roll code action
			set_roll_code(rc_buf+ROLL_CODE_INDEX);
			fc_status.code_matched = true;
			return true;
		}
	}
	
	return false;
}

void set_rc_channel_max(Rc_Channel_t *_rc, uint16_t _value)
{
	_rc->rc_max = _value;
}

void set_rc_channel_min(Rc_Channel_t *_rc, uint16_t _value)
{
	_rc->rc_min = _value;
}

void set_rc_channel_neutral(Rc_Channel_t *_rc, uint16_t _value)
{
	_rc->rc_neutral = _value;
}

void set_rc_channel_reversed(Rc_Channel_t *_rc, uint8_t _value)
{
	_rc->reversed = _value;
}

void set_rc_channel_deadzone(Rc_Channel_t *_rc, uint16_t _value)
{
	_rc->dead_zone = _value;
}

void setting_rc_channel_all(void)
{
	uint8_t i = 0;
	
	for (; i<RC_CHANNEL_NUM_MAX; i++) {
		set_rc_channel_max(&rc_channels[i], RC_CHANNEL_MAX);
		set_rc_channel_min(&rc_channels[i], RC_CHANNEL_MIN);
		set_rc_channel_neutral(&rc_channels[i], RC_CHANNEL_NEUTRAL);
		set_rc_channel_deadzone(&rc_channels[i], RC_CHANNEL_DEADZONE);
	}
	set_rc_channel_reversed(&rc_channels[RC_EULER_ROLL_CH], RC_ROLL_REVERSED);
	set_rc_channel_reversed(&rc_channels[RC_EULER_YAW_CH], RC_YAW_REVERSED);
	set_rc_channel_reversed(&rc_channels[RC_EULER_PITCH_CH], RC_PITCH_REVERSED);
	set_rc_channel_reversed(&rc_channels[RC_THROTTLE_CH], RC_THROTTLE_REVERSED);
}

void check_motor_armed(void)
{
	static uint8_t check_times = 0;
	static uint8_t idle_tick = 0;
	
	switch (fc_status.armed) {
        case DISARMED:
			if ((rc_channels[0].rc_in > RC_CHANNEL_DISARMED) && (rc_channels[1].rc_in < (RC_CHANNEL_MAX - RC_CHANNEL_DISARMED)) \
									&& (rc_channels[2].rc_in > RC_CHANNEL_DISARMED) && (rc_channels[3].rc_in > RC_CHANNEL_DISARMED)) {
				check_times++;
				if (check_times > 10) {
					fc_status.armed = IDLED;
					check_times = 0;
					idle_tick = 0;
				}
			} else {
				check_times = 0;
			}
			break;
		case IDLED:
			idle_tick++;
			
			if (idle_tick > 100) {
                fc_status.armed = DISARMED;
				idle_tick = 0;
			}
			
			if ((rc_channels[RC_THROTTLE_CH].reversed && (rc_channels[RC_THROTTLE_CH].rc_in < (rc_channels[RC_THROTTLE_CH].rc_neutral - rc_channels[RC_THROTTLE_CH].dead_zone * 2))) || \
					(!rc_channels[RC_THROTTLE_CH].reversed && (rc_channels[RC_THROTTLE_CH].rc_in > (rc_channels[RC_THROTTLE_CH].rc_neutral + rc_channels[RC_THROTTLE_CH].dead_zone * 2))))
			{
                fc_status.armed = ARMED;
				check_times = 0;
			} else if ((rc_channels[0].rc_in > RC_CHANNEL_DISARMED) && (rc_channels[1].rc_in > RC_CHANNEL_DISARMED) \
									&& (rc_channels[2].rc_in > RC_CHANNEL_DISARMED) && (rc_channels[3].rc_in < (RC_CHANNEL_MAX - RC_CHANNEL_DISARMED))) {
				check_times++;
				if (check_times > 10) {						
                    fc_status.armed = DISARMED;
					check_times = 0;
				}
			} else {
				check_times = 0;
			}
			
			break;
        case ARMED:
			if (((rc_channels[RC_THROTTLE_CH].reversed && (rc_channels[RC_THROTTLE_CH].rc_in > (rc_channels[RC_THROTTLE_CH].rc_max - rc_channels[RC_THROTTLE_CH].dead_zone * 10))) || \
					(!rc_channels[RC_THROTTLE_CH].reversed && (rc_channels[RC_THROTTLE_CH].rc_in < (rc_channels[RC_THROTTLE_CH].rc_min + rc_channels[RC_THROTTLE_CH].dead_zone * 10)))))
			{
				check_times++;
				
				// need to rewrite
				if (check_times > 15) {
                    set_land_complete(true);
					check_times = 0;
				}
				if (fc_status.land_complete) {
					fc_status.armed = IDLED;
					idle_tick = 0;
				}
			} else {
				check_times = 0;
			}
			break;
		case MOTOR_TEST:
			break;
		default:
			break;
	}
}

bool check_throttle_is_safe(void)
{
    if (rc_channels[RC_THROTTLE_CH].reversed) {
        return (rc_channels[RC_THROTTLE_CH].rc_in < (RC_CHANNEL_NEUTRAL + RC_CHANNEL_DEADZONE * 10));
    } else {
        return (rc_channels[RC_THROTTLE_CH].rc_in > (RC_CHANNEL_NEUTRAL - RC_CHANNEL_DEADZONE * 10));
    }
}

