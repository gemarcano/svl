#ifndef _SVL_UTILS_H_
#define _SVL_UTILS_H_

#include <stdint.h>
#include <stdbool.h>

/** Enable Apollo3 burst mode, 96MHz system clock.
 */
bool enable_burst_mode(void);

/** Disable Apollo3 burst mode, 96MHz system clock.
 */
bool disable_burst_mode(void);

// The number of milliseconds since the timer was initialized
extern volatile uint32_t jiffies;

#endif // _SVL_UTILS_H_
