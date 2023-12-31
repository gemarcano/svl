#ifndef _SVL_UTILS_H_
#define _SVL_UTILS_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

void        _fill_ticks(void);
size_t      millis(void);
bool        enable_burst_mode(void);
bool        disable_burst_mode(void);
uint32_t    ap3_gpio_enable_interrupts(uint32_t ui32Pin, uint32_t eIntDir);

extern volatile uint32_t ap3_stimer_overflows;

#endif // _SVL_UTILS_H_
