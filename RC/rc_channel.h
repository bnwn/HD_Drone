#ifndef __RC_CHANNEL_H
#define __RC_CHANNEL_H

#include "attitude_control.h"

#define RF_XNS104 104
#define RADIO RF_XNS104
#define RC_CHANNEL_NUM_MAX 4 
#define RC_SWITCH_MAX 8
#define DATA_BUF_MAX 9
#define ROLL_CODE_NUM 3

#define START_CODE_INDEX 0
#define ROLL_CODE_INDEX 2
#define DATA_LEN_INDEX 5
#define DATA_INDEX 6
#define END_CODE_INDEX 15

#define RC_START 0
#define RC_ROLL_CODE 1
#define RC_DATA_LEN 2
#define RC_DATA 3
#define RC_END 4

#define ATT_ROLL_PITCH_YAW_MAX 45 // unit(degree)
#define RC_EULER_PITCH_CH 0
#define RC_EULER_ROLL_CH 1
#define RC_THROTTLE_CH 2
#define RC_EULER_YAW_CH 3

#define RC_CHANNEL_DISARMED 2867 // percent 70
#define RC_CHANNEL_MAX 4096
#define RC_CHANNEL_MIN 0
#define RC_CHANNEL_NEUTRAL 2048
#define RC_CHANNEL_DEADZONE 50
#define RC_THROTTLE_REVERSED 1
#define RC_ROLL_REVERSED 0
#define RC_PITCH_REVERSED 0
#define RC_YAW_REVERSED  0

#define RC_THROTTLE_OUT_LIMIT 0.95f

typedef struct {
		uint16_t rc_in;
		uint16_t rc_max;
		uint16_t rc_min;
		uint16_t rc_neutral;
		uint8_t reversed;
		uint16_t dead_zone;
}Rc_Channel_t;

typedef struct {
	uint8_t rc_in : 2;
}Rc_Switch_t;

void rc_channel_init(void);
bool rc_channel_read(void);
static void packet_parse(void);
static void switch_handle(void);
static void switch_event_trigger(uint8_t _ch, bool _is_long_hold);
float norm_input(Rc_Channel_t *_rc);
float norm_input_dz(Rc_Channel_t *_rc);
float channel_input_to_target(Rc_Channel_t *_rc, float _max_range);
void get_desired_leans_angles(_Target_Attitude *_target_att, float _leans_limit);
float get_desired_throttle_expo(void);
void set_roll_code(uint8_t *_code);
bool auto_code_matching(void);
void set_rc_channel_max(Rc_Channel_t *_rc, uint16_t _value);
void set_rc_channel_min(Rc_Channel_t *_rc, uint16_t _value);
void set_rc_channel_neutral(Rc_Channel_t *_rc, uint16_t _value);
void set_rc_channel_reversed(Rc_Channel_t *_rc, uint8_t _value);
void set_rc_channel_deadzone(Rc_Channel_t *_rc, uint16_t _value);
void setting_rc_channel_all(void);
void check_motor_armed(void);

extern Rc_Channel_t rc_channels[RC_CHANNEL_NUM_MAX];
extern Rc_Switch_t rc_switchs[RC_SWITCH_MAX];
#endif

