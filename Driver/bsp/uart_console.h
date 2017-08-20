#ifndef __UART_CONSOLE_H
#define __UART_CONSOLE_H

#include "PN020Series.h"
#include "uart.h"

#define CONSOLE_BAUDRATE 115200
#define CONSOLE_BUF_SIZE 1024

#define ASCII_A 0x61
//#define ASCII_B 0x62
//#define ASCII_C 0x63
#define ASCII_D 0x64
//#define ASCII_E 0x65
//#define ASCII_F 0x66
#define ASCII_G 0x67
#define ASCII_H 0x68
#define ASCII_I 0x69
//#define ASCII_J 0x6A
//#define ASCII_K 0x6B
#define ASCII_L 0x6C
//#define ASCII_M 0x6D
#define ASCII_N 0x6E
//#define ASCII_O 0x6F
#define ASCII_P 0x70
//#define ASCII_Q 0x71
#define ASCII_R 0x72
#define ASCII_S 0x73
#define ASCII_T 0x74
//#define ASCII_U 0x75
//#define ASCII_V 0x76
#define ASCII_W 0x77
//#define ASCII_X 0x78
#define ASCII_Y 0x79
//#define ASCII_Z 0x7A

#define ASCII_RLL (0x00ffffff & ((ASCII_R << 16) | (ASCII_L << 8) | ASCII_L))
#define ASCII_PTH (0x00ffffff & ((ASCII_P << 16) | (ASCII_T << 8) | ASCII_H))
#define ASCII_YAW (0x00ffffff & ((ASCII_Y << 16) | (ASCII_A << 8) | ASCII_W))
#define ASCII_ANG (0x00ffffff & ((ASCII_A << 16) | (ASCII_N << 8) | ASCII_G))
#define ASCII_RAT (0x00ffffff & ((ASCII_R << 16) | (ASCII_A << 8) | ASCII_T))

#define ASCII_SET 0x736574
#define ASCII_AHRS 0x61687273
#define ASCII_RADIX 0x2E
#define ASCII_SPACE 0x20
#define ASCII_LF 0x0A
#define ASCII_REC_START 0x24
#define ASCII_REC_END 0x23

void uart_console_init(UART_T* _uart, uint32_t _baudrate);
float get_float(uint16_t _buf_index);
bool console_input_handle(void);

#endif
