// fast_loop.c

#include "fast_loop.h"

#include "bemf.h"
#include "speed_measurement.h"
#include "position_estimator.h"
#include "motor_control.h"
#include "sensorless_handover.h"
#include "motor_states.h"

extern BemfHandle_t         g_bemf;
extern SensorlessHandover_t g_handover;

static float s_fast_period_s = 0.00005f;  // default 20 kHz

void FastLoop_init(float period_s)
{
    if (period_s > 0.0f) {
        s_fast_period_s = period_s;
    }
}

void FastLoop_step(float now_s)
{
    (void)s_fast_period_s; // kept if you want it later, but not needed here

    // 1) Update BEMF ADC readings
    Bemf_update(&g_bemf);

    // 2) Update speed estimation (Hall or BEMF, depending on mode)
    SpeedMeas_update(now_s);

    // 3) Update position estimator
    PosEst_update();

    // 4) Optional: Hall->BEMF handover (only meaningful in RUN)
    MotorContext_t ctx = MotorControl_getContext();
    if (ctx.state == MOTOR_STATE_RUN) {
        PosEst_t pe = PosEst_get();

        uint8_t current_sector = pe.sector;           // 0..5
        bool    direction_fwd  = (ctx.cmd.direction == 0); // 0=fwd in your struct

        SensorlessHandover_step(&g_handover,
                                now_s,
                                current_sector,
                                direction_fwd);
    }

    // 5) High-rate motor control (commutation, PI, PWM)
    MotorControl_stepFast();
}