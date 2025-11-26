#include "pi_controller.h"

static float clamp_float(float x, float min, float max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

void PI_init(PI_Controller_t *pi,
             float kp,
             float ki,
             float Ts,
             float out_min,
             float out_max)
{
    if (!pi) return;

    pi->kp = kp;
    pi->ki = ki;
    pi->Ts = Ts;

    pi->integrator = 0.0f;
    pi->out_min = out_min;
    pi->out_max = out_max;
    pi->last_output = 0.0f;
}

void PI_reset(PI_Controller_t *pi)
{
    if (!pi) return;

    pi->integrator = 0.0f;
    pi->last_output = 0.0f;
}

void PI_setGains(PI_Controller_t *pi,
                 float kp,
                 float ki)
{
    if (!pi) return;

    pi->kp = kp;
    pi->ki = ki;
}

float PI_step(PI_Controller_t *pi,
              float ref,
              float meas,
              bool use_antiwindup,
              PI_Status_t *status_out)
{
    if (!pi) return 0.0f;

    PI_Status_t status = PI_OK;

    // Error
    float e = ref - meas;

    // Proportional term
    float u_p = pi->kp * e;

    // Candidate new integrator value
    float i_candidate = pi->integrator + (pi->ki * pi->Ts * e);

    // Combine candidate PI output
    float u_unsat = u_p + i_candidate;

    // Apply output saturation
    float u_sat = clamp_float(u_unsat, pi->out_min, pi->out_max);

    // Anti-windup: only update integrator if we are NOT
    // pushing further into saturation.
    if (use_antiwindup) {
        if (u_sat == pi->out_max && e > 0.0f) {
            // Saturated high and error would push further up -> don't integrate
            // keep pi->integrator as is
        } else if (u_sat == pi->out_min && e < 0.0f) {
            // Saturated low and error would push further down -> don't integrate
        } else {
            // Safe to accept new integrator
            pi->integrator = i_candidate;
        }
    } else {
        // No anti-windup: always accept candidate integrator
        pi->integrator = i_candidate;
    }

    // Update status based on saturation
    if (u_sat >= pi->out_max - 1e-6f) {
        status = PI_SAT_HIGH;
    } else if (u_sat <= pi->out_min + 1e-6f) {
        status = PI_SAT_LOW;
    }

    pi->last_output = u_sat;

    if (status_out) {
        *status_out = status;
    }

    return u_sat;
}