// slow_loop.c

#include "slow_loop.h"

#include "motor_control.h"
#include "motor_states.h"
#include "position_estimator.h"
#include "speed_measurement.h"

#include <stdio.h>

// Slow loop period (seconds)
static float s_slow_period_s = 0.001f;  // default 1 kHz
static float s_last_run_s    = 0.0f;

void SlowLoop_init(float period_s)
{
    if (period_s > 0.0f) {
        s_slow_period_s = period_s;
    }
    s_last_run_s = 0.0f;
}

void SlowLoop_run(float now_s)
{
    if (s_last_run_s == 0.0f) {
        s_last_run_s = now_s;
    }

    if ((now_s - s_last_run_s) < s_slow_period_s) {
        return; // not time yet
    }

    s_last_run_s += s_slow_period_s;

    // -------- Slow loop tasks --------

    // 1) Run outer control logic (state machine, speed PI)
    MotorControl_stepSlow();

    // 2) Example: debug/telemetry print (optional or move to UDP)
    MotorContext_t ctx = MotorControl_getContext();
    PosEst_t       pe  = PosEst_get();

    //Very lightweight printf, or replace with your UDP logging
    Comment out if you don’t want console spam.
    printf("State=%d RPM=%.1f sector=%u torque=%.2f\n",
           ctx.state,
            pe.mech_speed,
            pe.sector,
            ctx.cmd.torque_cmd);
}