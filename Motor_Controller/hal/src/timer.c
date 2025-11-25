#include "timer.h"
#include <time.h>
#include <unistd.h>

struct timespec timer_now(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts;
}

long timer_diff_ms(struct timespec start, struct timespec end) {
    long sec_diff = end.tv_sec - start.tv_sec;
    long nsec_diff = end.tv_nsec - start.tv_nsec;
    return sec_diff * 1000 + nsec_diff / 1000000;
}

void timer_delay_ms(int ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

int timer_expired(struct timespec start, long timeout_ms) {
    struct timespec now = timer_now();
    return timer_diff_ms(start, now) >= timeout_ms;
}

long timer_remaining(struct timespec start, long timeout_ms) {
    struct timespec now = timer_now();
    long elapsed = timer_diff_ms(start, now);
    long remaining = timeout_ms - elapsed;
    return remaining > 0 ? remaining : 0;
}