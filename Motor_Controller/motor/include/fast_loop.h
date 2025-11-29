/// fast_loop.h
#pragma once

void FastLoop_init(float period_s);

/**
 * @brief One iteration of the fast loop.
 *
 * Must be called periodically at the configured rate
 * (e.g. 20 kHz) from a real-time thread.
 */
void FastLoop_step(float now_s);