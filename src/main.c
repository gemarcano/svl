//*****************************************************************************
//
//! @file main.c
//!
//! @brief A variable-baud rate bootloader for Apollo3 / Artemis module
//!
//! Purpose:
//
//*****************************************************************************

/*
Copyright (c) 2020 SparkFun Electronics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*

Authors:
Owen Lyke, Nathan Seidle

Modified: Juy 22 2019

Futher modifications and refactoring:
Gabriel Marcano

Modified: April 24 2024

*/

//*****************************************************************************
//
// Includes
//
//*****************************************************************************
#include "am_mcu_apollo.h"
#include "am_util.h"

#include "stdint.h"
#include <assert.h>

#include "svl_packet.h"
#include "svl_uart.h"
#include "svl_utils.h"

//*****************************************************************************
//
// Defines
//
//*****************************************************************************
#define SVL_VERSION_NUMBER 0x05

//*****************************************************************************
//
// Bootloader Options
//
//*****************************************************************************
// must be larger than maximum frame transmission length for guaranteed
// performance
#define BL_UART_BUF_LEN   (2048 + 512)
// which UART peripheral to use for BL data
#define BL_UART_INST      0
// Pad to determine whether to skip BL or not
#define BL_BOOT_PAD       47
// RX pad for BL_UART_INST
#define BL_RX_PAD         49
// TX pad for BL_UART_INST
#define BL_TX_PAD         48
// location in flash to store user's code
// (Linker script vectors must match this address)
#define USERCODE_OFFSET   (0xC000 + 0x4000)
// maximum number of 4-byte words that can be transmitted in one frame packet
#define FRAME_BUFFER_SIZE 512

//*****************************************************************************
//
// Bootloader Commands
//
//*****************************************************************************
enum command
{
	/** Command containing the SVL version.
	 *
	 * This is sent from the device to the svl application.
	 *
	 * Payload contains a single byte, representing the SVL version.
	 */
	CMD_VERSION = 1,
	/** Command requesting bootloader to enter into flashing state.
	 *
	 * This is sent by the svl application to the device.
	 *
	 * There is no payload.
	 */
	CMD_BLMODE,
	/** Command asking for more data to flash to the device.
	 *
	 * This is sent by the device to the svl application.
	 *
	 * There is no payload.
	 */
	CMD_NEXT,
	/** Command with data to flash.
	 *
	 * This is sent by the svl application to the device.
	 *
	 * The payload is at most FRAME_BUFFER_SIZE bytes of data to be flashed
	 * onto the device.
	 */
	CMD_FRAME,
	/** Command requesting previous data to be retransmitted.
	 *
	 * This is sent by the device to the svl application.
	 *
	 * There is no payload.
	 */
	CMD_RETRY,
	/** Command indicating that there is no more data to flash.
	 *
	 * This is sent by the svl application to the device.
	 *
	 * There is no payload.
	 */
	CMD_DONE,
	/** Command requesting data from memory.
	 *
	 * This is sent by the svl application to the device.
	 *
	 * The payload contains the address in 4 bytes to read from memory, and 4
	 * bytes indicating how many bytes to read. The two numbers must be in
	 * network byte order.
	 */
	CMD_READ = 100,
	/** Command with data from memory.
	 *
	 * This is sent by the device to the svl application.
	 *
	 * The payload contains the contents of memory requested by CMD_READ, with
	 * lowest memory addresses first.
	 */
	CMD_READ_RESP,
};

//*****************************************************************************
//
// Macros
//
//*****************************************************************************
#define UART_GPIO_PINCONFIG_INNER(INST, TXRX, PAD) \
	{ \
		.uFuncSel = AM_HAL_PIN_##PAD##_UART##INST##TXRX, \
		.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA \
	}

#define UART_GPIO_PINCONFIG(INST, TXRX, PAD) \
	UART_GPIO_PINCONFIG_INNER(INST, TXRX, PAD)

//*****************************************************************************
//
// Globals
//
//*****************************************************************************

// pointer to handle for bootloader UART
static void *hUART_bl = NULL;

// Storage for UART ring buffers
static uint8_t rx_buffer[BL_UART_BUF_LEN] = {0};
static uint8_t tx_buffer[BL_UART_BUF_LEN] = {0};

// The following are used by the UART baud autodetection
#define BL_BAUD_SAMPLES (5)
static volatile uint8_t bl_baud_ticks_index = 0x00;
static volatile uint32_t bl_baud_ticks[BL_BAUD_SAMPLES] = {0};

//*****************************************************************************
//
// Additional functions
//
//*****************************************************************************

/** Initializes the CPU and timer.
 */
static void setup(void)
{
	// Set the clock frequency.
	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

	// Set the default cache configuration
	am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
	am_hal_cachectrl_enable();

	// Configure the stimer
	am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREA);
	NVIC_EnableIRQ(STIMER_CMPR0_IRQn);
	am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
	am_hal_stimer_compare_delta_set(0, 3000);
	am_hal_stimer_config(AM_HAL_STIMER_HFRC_3MHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE);

	// Enable interrupts.
	am_hal_interrupt_master_enable();
}

/** Undoes setup, in preparation for jumping to user code
 */
static void unsetup(void)
{
	disable_burst_mode();

	// Deconfigure the stimer
	am_hal_stimer_int_disable(AM_HAL_STIMER_INT_COMPAREA);
	NVIC_DisableIRQ(STIMER_CMPR0_IRQn);
	am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
	am_hal_stimer_config(AM_HAL_STIMER_NO_CLK);

	// Disable interrupts.
	am_hal_interrupt_master_disable();
}

/** Peform baud-rate auto-detection.
 *
 * This works by listening for a 'U' character (0b01010101), and timestamping
 * low->high transitions. This is done by tracking time in ms, and by
 * leveraging GPIO interrupts to detect low->high transitions. The UART signal
 * looks like:
 *
 * ______      ____      ____      ____      ____      ______
 *       |____|    |____|    |____|    |____|    |____|
 *       start  1    0    1    0    1    0    1    0     end
 *
 * This leads to 5 low->high transitions. By taking the average duration of 2
 * bits (every low->high transition), we can estimate the baud rate.
 *
 * @param [out] baud Pointer to storage for result of baud autodetection.
 *
 * @returns True if baud autodetection detected a value, false otherwise.
 */
static bool detect_baud_rate(uint32_t *baud)
{
	enable_burst_mode();
	const am_hal_gpio_pincfg_t input_pullup_lo2hi =
	{
		.uFuncSel       = 3,
		.eGPOutcfg      = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
		.eGPInput       = AM_HAL_GPIO_PIN_INPUT_ENABLE,
		.eGPRdZero      = AM_HAL_GPIO_PIN_RDZERO_READPIN,
		.ePullup        = AM_HAL_GPIO_PIN_PULLUP_WEAK,
		.eIntDir        = AM_HAL_GPIO_PIN_INTDIR_LO2HI
	};
	am_hal_gpio_pinconfig(BL_RX_PAD, input_pullup_lo2hi);
	am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(BL_RX_PAD));
	am_hal_gpio_interrupt_enable(AM_HAL_GPIO_BIT(BL_RX_PAD));

	const uint32_t bl_entry_timeout_ms = 200;
	uint32_t bl_entry_timeout_start = millis();
	NVIC_EnableIRQ(GPIO_IRQn);
	while ((bl_baud_ticks_index != BL_BAUD_SAMPLES) &&
			((millis() - bl_entry_timeout_start) < bl_entry_timeout_ms))
	{
		// Just wait until we time out or we get enough samples
	}

	NVIC_DisableIRQ(GPIO_IRQn);
	am_hal_gpio_interrupt_disable(AM_HAL_GPIO_BIT(BL_RX_PAD));
	am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(BL_RX_PAD));

	disable_burst_mode();

	bool baud_is_valid = false;
	if (bl_baud_ticks_index == BL_BAUD_SAMPLES)
	{
		// compute differences between samples
		float mean = 0.0;
		for (uint8_t indi = 0; indi < (BL_BAUD_SAMPLES - 1); indi++)
		{
			uint32_t value = bl_baud_ticks[indi + 1] - bl_baud_ticks[indi];
			bl_baud_ticks[indi] = value;
			mean += value;
		}
		mean /= (BL_BAUD_SAMPLES - 1);

		// The timer is ticking at 3MHz. Each duration contains 2 bits. Thus,
		// Estimated baud rate = 1 / ((mean / 2) / 3 MHz)

		// Between 1500000 and 750000 baud...
		if ((mean >= 4) && (mean <= 8))
		{
			*baud = 921600;
			baud_is_valid = true;
		}
		// Between 600000 and 428571 baud...
		else if ((mean >= 10) && (mean <= 14))
		{
			*baud = 460800;
			baud_is_valid = true;
		}
		// Between 240000 and 200000 baud...
		else if ((mean >= 25) && (mean <= 30))
		{
			*baud = 230400;
			baud_is_valid = true;
		}
		// Between 133333 and 109091 baud...
		else if ((mean >= 45) && (mean <= 55))
		{
			*baud = 115200;
			baud_is_valid = true;
		}
		// Between 65934 and 54054 baud...
		else if ((mean >= 91) && (mean <= 111))
		{
			*baud = 57600;
			baud_is_valid = true;
		}
		// else invalid
	}

	return baud_is_valid;
}

// Start BL UART at desired baud
static void start_uart_bl(uint32_t baud)
{
	// Initialize the printf interface for UART output.
	am_hal_uart_initialize(BL_UART_INST, &hUART_bl);
	am_hal_uart_power_control(hUART_bl, AM_HAL_SYSCTRL_WAKE, false);

	const am_hal_uart_config_t bl_uart_config =
	{
		// Standard UART settings: 115200-8-N-1
		.ui32BaudRate = baud,
		.ui32DataBits = AM_HAL_UART_DATA_BITS_8,
		.ui32Parity = AM_HAL_UART_PARITY_NONE,
		.ui32StopBits = AM_HAL_UART_ONE_STOP_BIT,
		.ui32FlowControl = AM_HAL_UART_FLOW_CTRL_NONE,

		// Set TX and RX FIFOs to interrupt at half-full.
		.ui32FifoLevels = (AM_HAL_UART_TX_FIFO_1_2 |
			 AM_HAL_UART_RX_FIFO_1_2),

		// Buffers
		.pui8TxBuffer = tx_buffer,
		.ui32TxBufferSize = sizeof(tx_buffer),
		.pui8RxBuffer = rx_buffer,
		.ui32RxBufferSize = sizeof(rx_buffer),
	};
	am_hal_uart_configure(hUART_bl, &bl_uart_config);

	// Enable the UART pins.
	const am_hal_gpio_pincfg_t bl_uart_tx_pinconfig =
		UART_GPIO_PINCONFIG(BL_UART_INST, TX, BL_TX_PAD);
	am_hal_gpio_pinconfig(BL_TX_PAD, bl_uart_tx_pinconfig);

	const am_hal_gpio_pincfg_t bl_uart_rx_pinconfig =
		UART_GPIO_PINCONFIG(BL_UART_INST, RX, BL_RX_PAD);
	am_hal_gpio_pinconfig(BL_RX_PAD, bl_uart_rx_pinconfig);

	// Enable interrupts.
	NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + BL_UART_INST));
	am_hal_uart_interrupt_enable(hUART_bl, (AM_HAL_UART_INT_RX));

	// Provide SVL Packet interfaces
	const svl_packet_driver_t driver = {
		.read = svl_uart_read,
		.write = svl_uart_write,
		.millis = millis,
		.read_param = hUART_bl,
		.write_param = hUART_bl
	};
	svl_packet_driver_register(&driver);
}

// Disable UART interface
static void stop_uart_bl()
{
	// Disable interrupts before powering off
	NVIC_DisableIRQ((IRQn_Type)(UART0_IRQn + BL_UART_INST));
	am_hal_uart_interrupt_disable(hUART_bl, (AM_HAL_UART_INT_RX));

	// This also clears interrupts internally...
	am_hal_uart_power_control(hUART_bl, AM_HAL_SYSCTRL_DEEPSLEEP, false);
}

// Handle a frame packet
static uint8_t handle_frame_packet(
	svl_packet_t *packet,
	uint32_t *p_frame_address,
	uint16_t *p_last_page_erased)
{
	const uint32_t num_words = (packet->pl_len / 4);

	// Check payload length is multiple of words
	if ((packet->pl_len % 4))
	{
		return 1;
	}

	int32_t i32ReturnCode = 0;
	uint32_t offset_address = (*(p_frame_address) + USERCODE_OFFSET);
	if ((*p_last_page_erased) < AM_HAL_FLASH_ADDR2PAGE(offset_address))
	{
		// Erase the 8k page for this address
		i32ReturnCode = am_hal_flash_page_erase(
			AM_HAL_FLASH_PROGRAM_KEY,
			AM_HAL_FLASH_ADDR2INST(offset_address),
			AM_HAL_FLASH_ADDR2PAGE(offset_address));

		*(p_last_page_erased) = AM_HAL_FLASH_ADDR2PAGE(offset_address);

		if (i32ReturnCode)
		{
			//FIXME what should we do if we get a bad return code?
		}
	}

	// Record the array
	i32ReturnCode = am_hal_flash_program_main(
		AM_HAL_FLASH_PROGRAM_KEY,
		(uint32_t *)packet->pl,
		(uint32_t *)(*(p_frame_address) + USERCODE_OFFSET),
		num_words);

	if (i32ReturnCode)
	{
		// flash write error
		return 1;
	}

	*(p_frame_address) += num_words * 4;

	return 0;
}

// Bootload phase
static void enter_bootload(void)
{
	enable_burst_mode();

	// Storage for the incoming payload data
	static uint32_t frame_buffer[FRAME_BUFFER_SIZE];

	svl_packet_t svl_packet_incoming_frame = {
		.cmd = CMD_FRAME,
		.pl = (uint8_t *)frame_buffer,
		.pl_len = sizeof(frame_buffer) / sizeof(uint8_t),
		.max_pl_len = sizeof(frame_buffer) / sizeof(uint8_t)
	};

	const svl_packet_t svl_packet_retry = {
		.cmd = CMD_RETRY,
		.pl = NULL,
		.pl_len = 0,
		.max_pl_len = 0
	};

	const svl_packet_t svl_packet_next = {
		.cmd = CMD_NEXT,
		.pl = NULL,
		.pl_len = 0,
		.max_pl_len = 0
	};

	bool done = false;
	// Current offset/address of payload being processed
	uint32_t frame_address = 0;
	// Page number in flash that was last erased during programming
	uint16_t last_page_erased = 0;
	// The current retransmit count, 0 means this is the first transmit attempt
	bool retransmit = false;
	while (!done)
	{
		// Always send a NEXT or RETRY command
		// Effectively, NEXT acts like an ACK or that we're waiting for the
		// next command
		if (retransmit)
		{
			svl_packet_send(&svl_packet_retry); // Ask to retransmit
			am_hal_uart_tx_flush(hUART_bl);
			retransmit = false;
		}
		else
		{
			svl_packet_send(&svl_packet_next); // Ask for the next frame packet
			am_hal_uart_tx_flush(hUART_bl);
		}

		uint8_t stat = svl_packet_wait(&svl_packet_incoming_frame);
		if (stat != 0)
		{
			// Some error occurred receiving a packet,
			// wait for either a frame or the done command
			retransmit = true;
			am_util_delay_us(177000); //Worst case: wait 177ms for 2048 byte transfer at 115200bps to complete

			//Flush the buffers to remove any inbound or outbound garbage
			am_hal_uart_tx_flush(hUART_bl);
			uint8_t tmp[BL_UART_BUF_LEN];
			while(svl_uart_read(hUART_bl, tmp, sizeof(tmp)) == sizeof(tmp))
			{
				// Just wait until we're no longer receiving any data.
			}
			continue;
		}

		switch (svl_packet_incoming_frame.cmd)
		{
		case CMD_FRAME:
			if (handle_frame_packet(&svl_packet_incoming_frame, &frame_address, &last_page_erased) != 0)
			{
				retransmit = true;
			}
			break;
		case CMD_DONE:
			done = true;
			break;
		case CMD_READ:
			if (svl_packet_incoming_frame.pl_len == 8)
			{
				//FIXME what if address is invalid?
				uint8_t *address = (uint8_t*)(
					((uintptr_t)svl_packet_incoming_frame.pl[0]) << 24 |
					svl_packet_incoming_frame.pl[1] << 16 |
					svl_packet_incoming_frame.pl[2] << 8 |
					svl_packet_incoming_frame.pl[3] << 0);
				uint32_t size = ((svl_packet_incoming_frame.pl[4]) << 24 |
					svl_packet_incoming_frame.pl[5] << 16 |
					svl_packet_incoming_frame.pl[6] << 8 |
					svl_packet_incoming_frame.pl[7] << 0);
				const svl_packet_t svl_packet_response = {
					.cmd = CMD_READ_RESP,
					.pl = address,
					.pl_len = size,
					.max_pl_len = size
				};
				svl_packet_send(&svl_packet_response);
				am_hal_uart_tx_flush(hUART_bl);
			}
			else
			{
				retransmit = true;
			}
			break;
		default:
			retransmit = true;
			break;
		}
	}

	// finish bootloading
}

// External function that does the actual jump, written in assembly since we're
// screwing around with the LR register, and that may/may not confuse GCC.
// Plus, it's easier to mark that as noreturn and have GCC believe us.
[[noreturn]]
void app_jump(void*);

// Jump to the application
[[noreturn]]
static void app_start(void)
{
	// Undoes configuration to provide users with a clean slate
	unsetup();

	// Jump to start of user code
	// Using assembly since we're overriding the LR register to point to the
	// application's reset vector
	app_jump((void*)USERCODE_OFFSET);
}

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int main(void)
{
	setup();

	// Detects the baud rate. Returns true if a valid baud rate was found
	uint32_t bl_baud = 0x00;
	if (detect_baud_rate(&bl_baud) == false)
	{
		app_start(); // w/o valid baud rate jump t the app
	}

	// FIXME verify that the blip is real! I don't see any reason in software for it to exist
	start_uart_bl(bl_baud); // This will create a 23 us wide low 'blip' on the TX line (until possibly fixed)
	am_util_delay_us(200);  // At the minimum baud rate of 115200 one byte (10 bits with start/stop) takes 10/115200 or 87 us. 87+23 = 100, double to be safe

	const uint8_t packet_ver_buf[1] = {SVL_VERSION_NUMBER};
	const svl_packet_t svl_packet_version = {
		.cmd = CMD_VERSION,
		.pl = (uint8_t*)packet_ver_buf,
		.pl_len = sizeof(packet_ver_buf),
		.max_pl_len = sizeof(packet_ver_buf)
	};

	svl_packet_send(&svl_packet_version); // when baud rate is determined send the version packet
	am_hal_uart_tx_flush(hUART_bl);

	svl_packet_t svl_packet_blmode = {
		.cmd = CMD_BLMODE,
		.pl = NULL,
		.pl_len = 0,
		.max_pl_len = 0
	};

	if (svl_packet_wait(&svl_packet_blmode) != 0)
	{ // wait for the bootloader to confirm bootloader mode entry
		stop_uart_bl();
		app_start(); // break to app
	}

	enter_bootload(); // Now we are locked in

	am_hal_reset_control(AM_HAL_RESET_CONTROL_SWPOI, 0); //Cause a system Power On Init to release as much of the stack as possible
	__builtin_unreachable();
}

//*****************************************************************************
//
// UART interrupt handlers
//
//*****************************************************************************

__attribute__ ((used))
#if BL_UART_INST == 0
void am_uart_isr(void)
#else
void am_uart1_isr(void)
#endif // BL_UART_INST == 0
{
	// Service the FIFOs as necessary, and clear the interrupts.
	uint32_t ui32Status, ui32Idle;
	am_hal_uart_interrupt_status_get(hUART_bl, &ui32Status, true);
	am_hal_uart_interrupt_clear(hUART_bl, ui32Status);
	am_hal_uart_interrupt_service(hUART_bl, ui32Status, &ui32Idle);
}

//*****************************************************************************
//
// GPIO interrupt handler
//
//*****************************************************************************
__attribute__ ((used))
void am_gpio_isr(void)
{
	am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(BL_RX_PAD));
	if (bl_baud_ticks_index < BL_BAUD_SAMPLES)
	{
		bl_baud_ticks[bl_baud_ticks_index++] = CTIMER->STTMR;
	}
}

//*****************************************************************************
//
// STimer interrupt handler
//
//*****************************************************************************
volatile uint32_t jiffies;
__attribute__ ((used))
void am_stimer_cmpr0_isr(void)
{
	am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREA);
	am_hal_stimer_compare_delta_set(0, 3000);
	jiffies += 1;
}
