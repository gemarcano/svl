#ifndef _SVL_UART_H_
#define _SVL_UART_H_

#include <stdint.h>
#include <stddef.h>

size_t svl_uart_read(void *pHandle, uint8_t *buf, size_t len);
size_t svl_uart_write(void *pHandle, const uint8_t *c, size_t len);

/** Discard all contents of the RX buffer.
 */
static inline void svl_uart_discard(void *pHandle)
{
	uint8_t tmp;
	while(svl_uart_read(pHandle, &tmp, sizeof(tmp)) == sizeof(tmp))
	{
		// Just wait until we're no longer receiving any data.
	}
}

#endif // _SVL_UART_H_
