
#include "filters.h"

/* ============================
 *  First-order low-pass filter
 * ============================
 *
 * y[k] = y[k-1] + alpha * (x[k] - y[k-1])
 *  - alpha in [0,1]
 *  - alpha = 1  → no filtering
 *  - alpha ≈ 0 → very heavy filtering
 */

void LPF1_init(LPF1_t *f, float alpha)
{
    if (!f) return;

    f->alpha       = alpha;
    f->state       = 0.0f;
    f->initialized = 0;
}

void LPF1_reset(LPF1_t *f, float value)
{
    if (!f) return;

    f->state       = value;
    f->initialized = 1;
}

float LPF1_apply(LPF1_t *f, float x)
{
    if (!f) return x;

    // First sample: just take raw value
    if (!f->initialized) {
        f->state       = x;
        f->initialized = 1;
        return x;
    }

    f->state = f->state + f->alpha * (x - f->state);
    return f->state;
}


/* ============================
 *  Moving average filter
 * ============================
 *
 * windowed average over N samples:
 *   y[k] = (x[k] + ... + x[k-N+1]) / N
 */

void MA_init(MAFilter_t *f, float *buf, size_t length)
{
    if (!f || !buf || length == 0) return;

    f->buffer = buf;
    f->length = length;
    f->index  = 0;
    f->count  = 0;
    f->sum    = 0.0f;

    for (size_t i = 0; i < length; i++) {
        f->buffer[i] = 0.0f;
    }
}

void MA_reset(MAFilter_t *f, float value)
{
    if (!f || !f->buffer || f->length == 0) return;

    f->sum   = value * (float)f->length;
    f->count = f->length;
    f->index = 0;

    for (size_t i = 0; i < f->length; i++) {
        f->buffer[i] = value;
    }
}

float MA_apply(MAFilter_t *f, float x)
{
    if (!f || !f->buffer || f->length == 0) {
        return x;
    }

    if (f->count < f->length) {
        // Still filling the window
        f->count++;
    } else {
        // Subtract the oldest sample from the sum
        f->sum -= f->buffer[f->index];
    }

    // Insert new sample
    f->buffer[f->index] = x;
    f->sum += x;

    // Advance index (circular buffer)
    f->index++;
    if (f->index >= f->length) {
        f->index = 0;
    }

    // Return current average
    return f->sum / (float)f->count;
}


/* ============================
 *  Helpers: clamp & slew limit
 * ============================
 */

float clampf(float x, float min, float max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

/**
 * @brief Slew-rate limiter
 *
 * Limits |new - prev| <= max_step
 */
float slew_limit(float prev, float target, float max_step)
{
    float delta = target - prev;

    if (delta >  max_step) delta =  max_step;
    if (delta < -max_step) delta = -max_step;

    return prev + delta;
}