#include "PN020Series.h"
#include "uart_console.h"

void uart_console_init(UART_T* _uart, uint32_t _baudrate)
{
    UART_Open(_uart, _baudrate);
}
