// pwm_motor.c

#include "pwm_motor.h"
#include <stdlib.h>
#include <stdio.h>
#include <gpiod.h>   // for GPIOD_LINE_DIRECTION_OUTPUT, GPIOD_LINE_EDGE_NONE

// Local helper to drive one phase's INH/INL according to sign.
static void drive_phase(GPIO_Handle *gpio,
                        unsigned int inh_idx,
                        unsigned int inl_idx,
                        int sign,
                        bool enable)
{
    int inh_val = 0;
    int inl_val = 0;

    if (!enable || sign == 0) {
        // Phase off / floating -> both low
        inh_val = 0;
        inl_val = 0;
    } else if (sign > 0) {
        // High-side active
        inh_val = 1;
        inl_val = 0;
    } else { // sign < 0
        // Low-side active
        inh_val = 0;
        inl_val = 1;
    }

    gpio_write(gpio, inh_idx, inh_val);
    gpio_write(gpio, inl_idx, inl_val);
}

bool PwmMotor_init(PwmMotor_t *m,
                   const char *chip_path,
                   unsigned int inh_a_offset,
                   unsigned int inl_a_offset,
                   unsigned int inh_b_offset,
                   unsigned int inl_b_offset,
                   unsigned int inh_c_offset,
                   unsigned int inl_c_offset)
{
    if (!m || !chip_path) {
        return false;
    }

    unsigned int offsets[6] = {
        inh_a_offset,
        inl_a_offset,
        inh_b_offset,
        inl_b_offset,
        inh_c_offset,
        inl_c_offset
    };

    // Request all 6 lines as outputs, initially low.
    m->gpio = gpio_init(chip_path,
                        offsets,
                        6,
                        GPIOD_LINE_DIRECTION_OUTPUT,
                        GPIOD_LINE_EDGE_NONE);
    if (!m->gpio) {
        fprintf(stderr,
                "PwmMotor_init: gpio_init failed for chip %s (offsets %u,%u,%u,%u,%u,%u)\n",
                chip_path,
                inh_a_offset, inl_a_offset,
                inh_b_offset, inl_b_offset,
                inh_c_offset, inl_c_offset);
        return false;
    }

    m->inh_a_idx = 0;
    m->inl_a_idx = 1;
    m->inh_b_idx = 2;
    m->inl_b_idx = 3;
    m->inh_c_idx = 4;
    m->inl_c_idx = 5;

    m->enabled = false;

    // Ensure everything is off initially
    drive_phase(m->gpio, m->inh_a_idx, m->inl_a_idx, 0, false);
    drive_phase(m->gpio, m->inh_b_idx, m->inl_b_idx, 0, false);
    drive_phase(m->gpio, m->inh_c_idx, m->inl_c_idx, 0, false);

    return true;
}

void PwmMotor_setEnable(PwmMotor_t *m, bool enable)
{
    if (!m || !m->gpio) return;

    m->enabled = enable;

    if (!enable) {
        // Immediately force all phases off
        drive_phase(m->gpio, m->inh_a_idx, m->inl_a_idx, 0, false);
        drive_phase(m->gpio, m->inh_b_idx, m->inl_b_idx, 0, false);
        drive_phase(m->gpio, m->inh_c_idx, m->inl_c_idx, 0, false);
    }
}

void PwmMotor_applyPhaseState(PwmMotor_t *m,
                              int u, int v, int w,
                              float duty)
{
    if (!m || !m->gpio) return;

    // Currently we treat duty simply as on/off:
    //  duty <= 0 → no drive, duty > 0 → full drive.
    // Later, you can integrate real PWM by modulating the
    // high/low active time per phase based on duty.
    bool active = (duty > 0.0f) && m->enabled;

    int su = active ? u : 0;
    int sv = active ? v : 0;
    int sw = active ? w : 0;

    drive_phase(m->gpio, m->inh_a_idx, m->inl_a_idx, su, m->enabled);
    drive_phase(m->gpio, m->inh_b_idx, m->inl_b_idx, sv, m->enabled);
    drive_phase(m->gpio, m->inh_c_idx, m->inl_c_idx, sw, m->enabled);
}

void PwmMotor_deinit(PwmMotor_t *m)
{
    if (!m) return;

    if (m->gpio) {
        // Ensure gates are all off
        drive_phase(m->gpio, m->inh_a_idx, m->inl_a_idx, 0, false);
        drive_phase(m->gpio, m->inh_b_idx, m->inl_b_idx, 0, false);
        drive_phase(m->gpio, m->inh_c_idx, m->inl_c_idx, 0, false);

        gpio_close(m->gpio);
        m->gpio = NULL;
    }

    m->enabled = false;
}