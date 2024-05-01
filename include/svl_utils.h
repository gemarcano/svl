#ifndef _SVL_UTILS_H_
#define _SVL_UTILS_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>

/** Returns the number of milliseconds since STIMER was initialized.
 *
 * This assumes STIMER is initialized and configured to run at 3MHz.
 *
 * @returns The number of milliseconds since STIMER was started.
 */
size_t millis(void);

/** Enable Apollo3 burst mode, 96MHz system clock.
 */
bool enable_burst_mode(void);

/** Disable Apollo3 burst mode, 96MHz system clock.
 */
bool disable_burst_mode(void);

// The number of times the STIMER has overflowed
extern volatile atomic_uint_least32_t ap3_stimer_overflows;

#endif // _SVL_UTILS_H_
