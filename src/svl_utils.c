#include "svl_utils.h"
#include <am_mcu_apollo.h>

#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

#define AP3_STIMER_FREQ_HZ (3000000)
#define AP3_STIMER_FREQ_KHZ (AP3_STIMER_FREQ_HZ / 1000)
#define AP3_STIMER_FREQ_MHZ (AP3_STIMER_FREQ_HZ / 1000000)

volatile atomic_uint_least32_t ap3_stimer_overflows = 0x00;

size_t millis(void)
{
	uint64_t ticks = ap3_stimer_overflows;
	ticks <<= 32;
	ticks |= (am_hal_stimer_counter_get() & 0xFFFFFFFF);
	return (size_t)(ticks / AP3_STIMER_FREQ_KHZ);
}

//*****************************************************************************
//
// Burst mode
//
//*****************************************************************************
bool enable_burst_mode(void)
{
	// Check that the Burst Feature is available.
	am_hal_burst_avail_e eBurstModeAvailable;
	if (AM_HAL_STATUS_SUCCESS != am_hal_burst_mode_initialize(&eBurstModeAvailable))
	{
		return (false);
	}

	// Put the MCU into "Burst" mode.
	am_hal_burst_mode_e eBurstMode;
	if (AM_HAL_STATUS_SUCCESS != am_hal_burst_mode_enable(&eBurstMode))
	{
		return (false);
	}
	return (true);
}

//Turns main processor from 96MHz to 48MHz
//Returns false if disable fails
bool disable_burst_mode(void)
{
	am_hal_burst_mode_e eBurstMode;
	if (AM_HAL_STATUS_SUCCESS == am_hal_burst_mode_disable(&eBurstMode))
	{
		if (AM_HAL_NORMAL_MODE != eBurstMode)
		{
			return (false);
		}
	}
	else
	{
		return (false);
	}
	return (true);
}
