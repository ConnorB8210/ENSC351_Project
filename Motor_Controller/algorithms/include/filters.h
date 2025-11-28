// filters.h
#pragma once

#include <stddef.h>   // for size_t

/* ============================
 *  First‑order low‑pass filter
 * ============================ */

typedef struct
{
    float alpha;       // 0..1, 1 = no filtering
    float state;       // last output
    int   initialized; // 0 = not yet initialized, 1 = initialized
} LPF1_t;

/**
 * @brief Initialize a 1st‑order low‑pass filter
 *
 * y[k] = y[k-1] + alpha * (x[k] - y[k-1])
 */
void LPF1_init(LPF1_t *f, float alpha);

/**
 * @brief Reset low‑pass filter to a known value
 */
void LPF1_reset(LPF1_t *f, float value);

/**
 * @brief Apply low‑pass filter to input sample
 */
float LPF1_apply(LPF1_t *f, float x);


/* ============================
 *  Moving average filter
 * ============================ */

typedef struct
{
    float  *buffer;    // caller‑provided buffer
    size_t  length;    // window size
    size_t  index;     // current position
    size_t  count;     // number of valid samples (<= length)
    float   sum;       // running sum
} MAFilter_t;

/**
 * @brief Initialize moving average filter
 *
 * @param f       Filter instance
 * @param buf     Caller‑allocated buffer of length 'length'
 * @param length  Window size
 */
void MA_init(MAFilter_t *f, float *buf, size_t length);

/**
 * @brief Reset moving average filter to a constant value
 */
void MA_reset(MAFilter_t *f, float value);

/**
 * @brief Feed one sample, returns current average
 */
float MA_apply(MAFilter_t *f, float x);


/* ============================
 *  Helpers
 * ============================ */

/**
 * @brief Clamp float to [min, max]
 */
float clampf(float x, float min, float max);

/**
 * @brief Slew‑rate limiter: limit how fast a value can change
 *
 * @param prev     previous output
 * @param target   desired new value
 * @param max_step maximum allowed change per call
 * @return         new limited value
 */
float slew_limit(float prev, float target, float max_step);
