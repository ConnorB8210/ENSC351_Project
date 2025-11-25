#ifndef TIMER_H
#define TIMER_H

#include <time.h>

// Return current monotonic timestamp
struct timespec timer_now(void);

// Return time difference in ms
long timer_diff_ms(struct timespec start, struct timespec end);

// Sleep for specified milliseconds
void timer_delay_ms(int ms);

// Check if timeout has expired (returns 1 = expired, 0 = not yet)
int timer_expired(struct timespec start, long timeout_ms);

// Return how many ms remaining before timeout expires
long timer_remaining(struct timespec start, long timeout_ms);

#endif