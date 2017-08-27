#ifndef __RC_CHANNEL_H
#define __RC_CHANNEL_H

#define RF_XNS104 104
#define RADIO RF_XNS104
#define RC_CHANNEL_MAX 12
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

typedef struct {
		int16_t rc_in;
		int16_t rc_max;
		int16_t rc_min;
		int16_t rc_neutral;
		int8_t reversed;
		int16_t dead_zone;
}Rc_Channel_t;

void rc_channel_init(void);
bool rc_channel_read(void);
static void packet_parse(void);
static void switch_handle(void);
static void switch_event_trigger(uint8_t _ch, bool _long_hold);
float norm_input(Rc_Channel_t *_rc);
float norm_input_dz(Rc_Channel_t *_rc);
void set_roll_code(uint8_t *_code);
void auto_code_matching(void);


extern Rc_Channel_t rc_channels[RC_CHANNEL_MAX];

#endif

