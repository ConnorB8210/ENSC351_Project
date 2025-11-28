// fast_loop.h
#pragma once

#include <stdbool.h>

/**
 * @brief Initialize fast loop timing.
 *
 * @param period_s  desired fast loop period in seconds (e.g. 0.00005f for 20 kHz)
 */
void FastLoop_init(float period_s);

/**
 * @brief Run fast loop tasks if it's time.
 *
 * Call this frequently from main() with a monotonic timestamp in seconds.
 */
void FastLoop_run(float now_s);
