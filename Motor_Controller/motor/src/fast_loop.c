// fast_loop.c

#include "fast_loop.h"

#include "bemf.h"
#include "speed_measurement.h"
#include "position_estimator.h"
#include "motor_control.h"
#include "sensorless_handover.h"
#include "motor_states.h"

#include <math.h>

// These are defined in main.c (or another hw-init file)
extern BemfHandle_t          g_bemf;
extern SensorlessHandover_t  g_handover;

// Fast loop period (seconds) and last execution time
static float s_fast_period_s = 0.00005f;  // default 20 kHz
static float s_last_run_s    = 0.0f;

void FastLoop_init(float period_s)
{
    if (period_s > 0.0f) {
        s_fast_period_s = period_s;
    }
    s_last_run_s = 0.0f;
}

void FastLoop_run(float now_s)
{
    // Simple periodic scheduling
    if (s_last_run_s == 0.0f) {
        s_last_run_s = now_s;
    }

    if ((now_s - s_last_run_s) < s_fast_period_s) {
        return; // not time yet
    }

    s_last_run_s += s_fast_period_s;  // keep consistent period

    // -------- Fast loop tasks --------

    // 1) Update BEMF ADC readings
    Bemf_update(&g_bemf);

    // 2) Update speed estimation (Hall or BEMF, depending on SpeedMeas mode)
    SpeedMeas_update(now_s);

    // 3) Update position estimator (angle + sector)
    PosEst_update();

    // 4) Optional: Hall->BEMF handover logic while in RUN
    MotorContext_t ctx = MotorControl_getContext();

    if (ctx.state == MOTOR_STATE_RUN) {
        PosEst_t pe = PosEst_get();

        uint8_t current_sector = pe.sector;      // 0..5
        bool direction_fwd     = (ctx.cmd.direction == 0);  // 0 = fwd in your struct

        (void)SensorlessHandover_step(&g_handover,
                                      now_s,
                                      current_sector,
                                      direction_fwd);
    }

    // 5) Finally run the high-rate motor control (commutation, PI, PWM)
    MotorControl_stepFast();
}