// slow_loop.h
#pragma once

/**
 * @brief Initialize slow loop timing.
 *
 * @param period_s  desired slow loop period in seconds (e.g. 0.001f for 1 kHz)
 */
void SlowLoop_init(float period_s);

/**
 * @brief Run slow loop tasks if it's time.
 *
 * Call this frequently from main() with a monotonic timestamp in seconds.
 */
void SlowLoop_run(float now_s);
