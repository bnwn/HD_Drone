#include "PN020Series.h"
#include "rf.h"
#include "rc_channel.h"
#include "Algorithm_math.h"

Rc_Channel_t rc_channels[RC_CHANNEL_MAX] = {0};
uint8_t rc_buf[PAYLOAD_WIDTH] = {0}, data_buf[DATA_BUF_MAX] = {0}, roll_code[ROLL_CODE_NUM] = {0};
static uint8_t pre_switch = 0;

void rc_channel_init(void)
{
#if RADIO == RF_XNS104
    /* only use spi1 */
    SPI1_Init();
    RF_Init();
    RF_RxMode();
    printf("rf xns104 init success\n");
    code_matching();
#else
#endif
}

bool rc_channel_read(void)
{
    static uint8_t buf_index = 0;
    uint8_t rev_len = PAYLOAD_WIDTH - buf_index;
    uint8_t j = 0;

    ucRF_DumpRxData(rc_buf + buf_index, rev_len);
//		for(; j<PAYLOAD_WIDTH; j++)
//		{
//				printf("0x%X ", rc_buf[j]);
//		}
//		printf("\n");
	
#if 1
    // if (roll_code eq)
    if (rc_buf[START_CODE_INDEX] == 0xAA && rc_buf[START_CODE_INDEX+1] == 0xAA \ // start code
            && rc_buf[END_CODE_INDEX] == 0xCE && rc_buf[END_CODE_INDEX+1] == 0xED) { // end code

            for(; j<ROLL_CODE_NUM; j++) {
                if (rc_buf[ROLL_CODE_INDEX+j] != roll_code[j]) {
                    return false;
                }
            }
            rc_channels[0].rc_in = (rc_buf[DATA_INDEX] << 8) | rc_buf[DATA_INDEX+1];
            rc_channels[1].rc_in = (rc_buf[DATA_INDEX+2] << 8) | rc_buf[DATA_INDEX+3];
            rc_channels[2].rc_in = (rc_buf[DATA_INDEX+4] << 8) | rc_buf[DATA_INDEX+5];
            rc_channels[3].rc_in = (rc_buf[DATA_INDEX+6] << 8) | rc_buf[DATA_INDEX+7];

            rc_channels[4].rc_in = (rc_buf[DATA_INDEX+8] & 0x01) ? 1 : 0;
            rc_channels[5].rc_in = (rc_buf[DATA_INDEX+8] & 0x02) ? 1 : 0;
            rc_channels[6].rc_in = (rc_buf[DATA_INDEX+8] & 0x04) ? 1 : 0;
            rc_channels[7].rc_in = (rc_buf[DATA_INDEX+8] & 0x08) ? 1 : 0;
            rc_channels[8].rc_in = (rc_buf[DATA_INDEX+8] & 0x10) ? 1 : 0;
            rc_channels[9].rc_in = (rc_buf[DATA_INDEX+8] & 0x20) ? 1 : 0;
            rc_channels[10].rc_in = (rc_buf[DATA_INDEX+8] & 0x40) ? 1 : 0;
            rc_channels[11].rc_in = (rc_buf[DATA_INDEX+8] & 0x80) ? 1 : 0;

            switch_handle();

    } else {
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

static void packet_parse(void)
{
    rc_channels[0].rc_in = (data_buf[0] << 8) | data_buf[1];
    rc_channels[1].rc_in = (data_buf[2] << 8) | data_buf[3];
    rc_channels[2].rc_in = (data_buf[4] << 8) | data_buf[5];
    rc_channels[3].rc_in = (data_buf[6] << 8) | data_buf[7];

    rc_channels[4].rc_in = (data_buf[8] & 0x01) ? 1 : 0;
    rc_channels[5].rc_in = (data_buf[8] & 0x02) ? 1 : 0;
    rc_channels[6].rc_in = (data_buf[8] & 0x04) ? 1 : 0;
    rc_channels[7].rc_in = (data_buf[8] & 0x08) ? 1 : 0;
    rc_channels[8].rc_in = (data_buf[8] & 0x10) ? 1 : 0;
    rc_channels[9].rc_in = (data_buf[8] & 0x20) ? 1 : 0;
    rc_channels[10].rc_in = (data_buf[8] & 0x40) ? 1 : 0;
    rc_channels[11].rc_in = (data_buf[8] & 0x80) ? 1 : 0;
}

static void switch_handle(void)
{
    static uint8_t switch_on_tick[RC_SWITCH_MAX] = {0};
    uint8_t ch_index = 0;
//				pre_switch = data_buf[8];
	
    for (; ch_index < RC_SWITCH_MAX; ch_index++) {
        if (rc_channels[ch_index+4].rc_in) {
            switch_on_tick[ch_index]++;
        } else if (switch_on_tick[ch_index] > 0) {
            switch_event_trigger(ch_index, (switch_on_tick[ch_index] >= 250));
            switch_on_tick[ch_index] = 0;
        }
    }
}

static void switch_event_trigger(uint8_t _ch, bool _long_hold)
{
    switch(_ch) {
        case 0:
            break;
        case 1:
            break;
        case 2:
            break;
        case 3:
            break;
        case 4:
            break;
        case 5:
            break;
        case 6:
            break;
        case 7:
            break;
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

    return data_limit(ret, 1.0f, -1.0f);
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

    return data_limit(ret, 1.0f, -1.0f);
}

void set_roll_code(uint8_t *_code)
{
    uint8_t i = 0;
    for (; i<ROLL_CODE_NUM; i++) {
        roll_code[i] = *(_code + i);
    }
    printf("roll code: 0x%X 0x%X 0x%X\n", roll_code[0], roll_code[1], roll_code[2]);
}

void code_matching(void)
{
    ucRF_DumpRxData(rc_buf, PAYLOAD_WIDTH);

    if (rc_buf[START_CODE_INDEX] == 0xAA && rc_buf[START_CODE_INDEX+1] == 0xAA) {
        if (rc_buf[END_CODE_INDEX] == 0xCE && rc_buf[END_CODE_INDEX+1] == 0xED) {
            set_roll_code(rc_buf+ROLL_CODE_INDEX);
        }
    }
}
