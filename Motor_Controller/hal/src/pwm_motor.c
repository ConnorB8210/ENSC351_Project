// hal/src/pwm_motor.c

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pwm_motor.h"   // brings in gpio.h and PwmMotor_t
#define PWM_DEBUG 1 
// ---------------- Internal helpers ----------------

// Helper to drive one phase using simple GPIO high/low logic.
static void set_phase_gpio(GPIO_Handle *gpio,
                           const char *name,
                           bool active,
                           int sign,
                           unsigned int inh_idx,
                           unsigned int inl_idx)
{
    if (!gpio) return;

    int inh_val = 0;
    int inl_val = 0;

    if (!active || sign == 0) {
        // Off / float
        inh_val = 0;
        inl_val = 0;
    } else if (sign > 0) {
        // High-side on, low-side off
        inh_val = 1;
        inl_val = 0;
    } else {
        // Low-side on, high-side off
        inh_val = 0;
        inl_val = 1;
    }

#if PWM_DEBUG
    // Track last values so we only print when something changes
    static int last_inh_val[6] = { -1, -1, -1, -1, -1, -1 };
    static int last_inl_val[6] = { -1, -1, -1, -1, -1, -1 };

    if (last_inh_val[inh_idx] != inh_val || last_inl_val[inl_idx] != inl_val) {
        last_inh_val[inh_idx] = inh_val;
        last_inl_val[inl_idx] = inl_val;

        printf("PWM DBG %s: INH(idx=%u)=%d  INL(idx=%u)=%d\n",
               name, inh_idx, inh_val, inl_idx, inl_val);
        fflush(stdout);
    }
#endif

    gpio_write(gpio, inh_idx, inh_val);
    gpio_write(gpio, inl_idx, inl_val);
}

// Map a 6-step sector + direction into (u,v,w) signs (+1, -1, 0)
static void sector_to_signs(uint8_t sector,
                            bool forward,
                            int *u, int *v, int *w)
{
    *u = 0;
    *v = 0;
    *w = 0;

    if (sector > 5) {
        // invalid sector -> all off
        return;
    }

    if (forward) {
        // Forward 6-step sequence:
        // sector:  U   V   W
        //   0     +1  -1   0
        //   1     +1   0  -1
        //   2      0  +1  -1
        //   3     -1  +1   0
        //   4     -1   0  +1
        //   5      0  -1  +1
        switch (sector) {
        case 0: *u = +1; *v = -1; *w =  0; break;
        case 1: *u = +1; *v =  0; *w = -1; break;
        case 2: *u =  0; *v = +1; *w = -1; break;
        case 3: *u = -1; *v = +1; *w =  0; break;
        case 4: *u = -1; *v =  0; *w = +1; break;
        case 5: *u =  0; *v = -1; *w = +1; break;
        default: break;
        }
    } else {
        // Reverse sequence: invert pattern
        switch (sector) {
        case 0: *u = -1; *v = +1; *w =  0; break;
        case 1: *u = -1; *v =  0; *w = +1; break;
        case 2: *u =  0; *v = -1; *w = +1; break;
        case 3: *u = +1; *v = -1; *w =  0; break;
        case 4: *u = +1; *v =  0; *w = -1; break;
        case 5: *u =  0; *v = +1; *w = -1; break;
        default: break;
        }
    }
}

// ---------------- Public API ----------------

// Initialize motor PWM/gate outputs (GPIO-backed for now).
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

    GPIO_Handle *h = gpio_init(chip_path,
                               offsets,
                               6,
                               GPIOD_LINE_DIRECTION_OUTPUT,
                               GPIOD_LINE_EDGE_NONE);
    if (!h) {
        return false;
    }

    m->gpio      = h;
    m->inh_a_idx = 0;
    m->inl_a_idx = 1;
    m->inh_b_idx = 2;
    m->inl_b_idx = 3;
    m->inh_c_idx = 4;
    m->inl_c_idx = 5;
    m->enabled   = false;

    // Drive everything low initially
    for (int i = 0; i < 6; ++i) {
        gpio_write(m->gpio, i, 0);
    }

    return true;
}

// Enable or disable all phases.
// When disabled, all gate lines are driven low.
void PwmMotor_setEnable(PwmMotor_t *m, bool enable)
{
    if (!m || !m->gpio) return;

    m->enabled = enable;
    if (!enable) {
        // Force everything low
        for (int i = 0; i < 6; ++i) {
            gpio_write(m->gpio, i, 0);
        }
    }
}

// Apply per-phase signs to gate lines.
// u, v, w: +1 = high-side, -1 = low-side, 0 = off.
// duty: [0..1]; this implementation only uses "duty > 0" as ON.
void PwmMotor_applyPhaseState(PwmMotor_t *m,
                              int u, int v, int w,
                              float duty)
{
    if (!m || !m->gpio) return;

    if (!m->enabled) {
        // If not enabled, ensure everything is low.
        for (int i = 0; i < 6; ++i) {
            gpio_write(m->gpio, i, 0);
        }
        return;
    }

    bool active = (duty > 0.0f);
    set_phase_gpio(m->gpio, "A", active, u, m->inh_a_idx, m->inl_a_idx);
    set_phase_gpio(m->gpio, "B", active, v, m->inh_b_idx, m->inl_b_idx);
    set_phase_gpio(m->gpio, "C", active, w, m->inh_c_idx, m->inl_c_idx);
}

// Drive all outputs low and release GPIO handle.
void PwmMotor_deinit(PwmMotor_t *m)
{
    if (!m) return;

    if (m->gpio) {
        for (int i = 0; i < 6; ++i) {
            gpio_write(m->gpio, i, 0);
        }
        gpio_close(m->gpio);
        m->gpio = NULL;
    }
    m->enabled = false;
}

// Convenience helper to drive all outputs low and disable.
void PwmMotor_stop(PwmMotor_t *m)
{
    if (!m || !m->gpio) return;
    PwmMotor_setEnable(m, false);
}

// 6-step commutation helper.
void PwmMotor_setSixStep(PwmMotor_t *m,
                         uint8_t sector,
                         float duty,
                         bool forward)
{
    if (!m || !m->gpio) return;

    // Clamp duty
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    int u = 0, v = 0, w = 0;
    sector_to_signs(sector, forward, &u, &v, &w);

    bool active = (duty > 0.0f);
    PwmMotor_setEnable(m, active);
    PwmMotor_applyPhaseState(m, u, v, w, duty);
}