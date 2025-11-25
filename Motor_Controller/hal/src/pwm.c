#include "pwm.h"
#include <stdio.h>

int pwm_init(struct pwm *p, const char *chip_path, unsigned int line, long period_ms)
{
    p->handle = gpio_init(chip_path, &line, 1, GPIOD_LINE_DIRECTION_OUTPUT, GPIOD_LINE_EDGE_NONE);
    if (!p->handle) {
        fprintf(stderr, "Failed to initialize PWM line %u on %s\n", line, chip_path);
        return -1;
    }
    p->line = 0;
    p->period_ms = period_ms;
    p->state = 0;
    p->timer = timer_now();
    gpio_write(p->handle, p->line, 0); // start LED OFF
    return 0;
}

void pwm_update(struct pwm *p)
{
    // If period_ms == 0, keep LED off and skip toggling
    if (p->period_ms == 0) {
        if (p->state != 0) {
            p->state = 0;
            gpio_write(p->handle, p->line, 0);
        }
        return;
    }

    if (timer_expired(p->timer, p->period_ms)) {
        p->state = !p->state;
        gpio_write(p->handle, p->line, p->state);
        p->timer = timer_now();
    }
}

void pwm_set_period(struct pwm *p, long period_ms)
{
    if (period_ms == p->period_ms)
        return;

    p->period_ms = period_ms;

    // If period set to 0 (0 Hz), turn LED off immediately
    if (p->period_ms == 0 && p->state != 0) {
        p->state = 0;
        gpio_write(p->handle, p->line, 0);
    }
}

void pwm_close(struct pwm *p)
{
    if (p->handle) {
        gpio_write(p->handle, p->line, 0); // ensure LED off on exit
        gpio_close(p->handle);
        p->handle = NULL;
    }
}