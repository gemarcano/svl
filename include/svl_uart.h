#ifndef _SVL_UART_H_
#define _SVL_UART_H_

#include <stdint.h>
#include <stddef.h>


size_t svl_uart_read          (void *pHandle, char* buf, size_t len);
size_t svl_uart_write         (void *pHandle, char* buf, size_t len);
size_t svl_uart_write_byte    (void *pHandle, uint8_t c);
size_t svl_uart_print         (void *pHandle, char* str);


#endif // _SVL_UART_H_
