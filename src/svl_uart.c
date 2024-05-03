#include "svl_uart.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <am_mcu_apollo.h>

//*****************************************************************************
//
// UART read buffer
//
//*****************************************************************************
size_t svl_uart_read(void *pHandle, uint8_t* buf, size_t len){
	uint32_t ui32BytesRead = 0x00;
	am_hal_uart_transfer_t sRead = {
		.ui32Direction = AM_HAL_UART_READ,
		.pui8Data = buf,
		.ui32NumBytes = len,
		.ui32TimeoutMs = 0,
		.pui32BytesTransferred = &ui32BytesRead,
	};
	am_hal_uart_transfer(pHandle, &sRead);
	return ui32BytesRead;
}

//*****************************************************************************
//
// UART write buffer
//
//*****************************************************************************
size_t svl_uart_write(void *pHandle, const uint8_t* buf, size_t len){
	uint32_t ui32BytesWritten = 0;
	const am_hal_uart_transfer_t sUartWrite =
	{
		.ui32Direction = AM_HAL_UART_WRITE,
		.pui8Data = (uint8_t*)buf,
		.ui32NumBytes = len,
		.ui32TimeoutMs = AM_HAL_UART_WAIT_FOREVER,
		.pui32BytesTransferred = &ui32BytesWritten,
	};

	am_hal_uart_transfer(pHandle, &sUartWrite);

	return ui32BytesWritten;
}

//*****************************************************************************
//
// UART send string
//
//*****************************************************************************
size_t svl_uart_print(void *pHandle, const char* str){
	uint32_t ui32StrLen = strlen(str);
	return svl_uart_write(pHandle, (const uint8_t*)str, ui32StrLen);
}
