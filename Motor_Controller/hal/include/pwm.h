#ifndef PWM_H
#define PWM_H

#include "gpio.h"
#include "timer.h"

struct pwm {
    void *handle;          // GPIO handle
    unsigned int line;     // GPIO line number
    long period_ms;        // Toggle period in milliseconds
    int state;             // Current LED state (0=OFF, 1=ON)
    struct timespec timer; // Last toggle timestamp
};

// Initialize PWM line
int pwm_init(struct pwm *p, const char *chip_path, unsigned int line, long period_ms);

// Update LED state; call continuously in main loop
void pwm_update(struct pwm *p);

// Set PWM period
void pwm_set_period(struct pwm *p, long period_ms);

// Close PWM line
void pwm_close(struct pwm *p);

#endif