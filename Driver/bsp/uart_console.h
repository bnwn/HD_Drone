#ifndef __UART_CONSOLE_H
#define __UART_CONSOLE_H

#include "uart.h"

#define CONSOLE_BAUDRATE 115200

void uart_console_init(UART_T* _uart, uint32_t _baudrate);

#endif
