#ifndef _SVL_UART_H_
#define _SVL_UART_H_

#include <stdint.h>
#include <stddef.h>

size_t svl_uart_read(void *pHandle, uint8_t *buf, size_t len);
size_t svl_uart_write(void *pHandle, const uint8_t *c, size_t len);
size_t svl_uart_print(void *pHandle, const char* str);

#endif // _SVL_UART_H_
