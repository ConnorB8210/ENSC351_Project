// motor_control.c

#include "motor_control.h"
#include "motor_states.h"
#include "position_estimator.h"
#include "speed_measurement.h"
#include "hall_commutator.h"
#include "pi_controller.h"
#include "filters.h"
#include "motor_config.h"

#include <math.h>   // fabsf
#include <string.h> // memset
#include <stdio.h>  // optional debug

// -----------------------------
// Config defaults (override in motor_config.h if you like)
// -----------------------------

#ifndef SPEED_LOOP_HZ
#define SPEED_LOOP_HZ        1000.0f   // 1 kHz speed loop
#endif

#ifndef SPEED_PI_KP
#define SPEED_PI_KP          0.05f
#endif

#ifndef SPEED_PI_KI
#define SPEED_PI_KI          5.0f
#endif

#ifndef MAX_TORQUE_CMD
#define MAX_TORQUE_CMD       1.0f      // normalized torque/duty command
#endif

// -----------------------------
// Internal state
// -----------------------------

static PI_Controller_t s_speed_pi;
static PosMode_t       s_pos_mode = POS_MODE_HALL;

// Forward declaration: you’ll hook this to your PWM HAL.
static void MotorControl_applyCommutation(uint8_t sector,
                                          float duty,
                                          bool direction);


// -----------------------------
// Public API
// -----------------------------

void MotorControl_init(void)
{
    // Init global motor context
    MotorStates_init();

    // Default: use Hall-based position/speed
    s_pos_mode = POS_MODE_HALL;
    PosEst_init(s_pos_mode);

    // Init speed estimator (hall-based)
    SpeedMeas_init();

    // Init speed PI
    float Ts_speed = 1.0f / SPEED_LOOP_HZ;
    PI_init(&s_speed_pi,
            SPEED_PI_KP,
            SPEED_PI_KI,
            Ts_speed,
            -MAX_TORQUE_CMD,
            +MAX_TORQUE_CMD);

    PI_reset(&s_speed_pi);

    // Start in IDLE
    MotorStates_setState(MOTOR_STATE_IDLE);
    MotorStates_setEnable(false);
    MotorStates_setTorqueCmd(0.0f);
}

void MotorControl_setSpeedCmd(float rpm, bool direction)
{
    MotorStates_setSpeedCmd(rpm);
    MotorStates_setDirection(direction);
}

void MotorControl_setEnable(bool enable)
{
    MotorStates_setEnable(enable);

    MotorContext_t ctx = MotorStates_get();
    if (!enable) {
        // Disable → go to IDLE and clear torque
        MotorStates_setState(MOTOR_STATE_IDLE);
        MotorStates_setTorqueCmd(0.0f);
    } else {
        // Enable → if not faulted, go to RUN
        if (ctx.state != MOTOR_STATE_FAULT) {
            MotorStates_setState(MOTOR_STATE_RUN);
        }
    }
}

MotorContext_t MotorControl_getContext(void)
{
    return MotorStates_get();
}

void MotorControl_onFault(void)
{
    // Latch fault state, disable torque
    MotorStates_setState(MOTOR_STATE_FAULT);
    MotorStates_setEnable(false);
    MotorStates_setTorqueCmd(0.0f);

    // TODO: also disable PWM outputs via your PWM/DRV HAL here
    // e.g. HAL_PWM_enableOutputs(false);
}

// -----------------------------
// Slow loop (e.g. 1 kHz)
// - speed PI
// - state machine decisions
// -----------------------------
void MotorControl_stepSlow(void)
{
    MotorContext_t ctx = MotorStates_get();
    PosEst_t       est = PosEst_get();

    if (ctx.state == MOTOR_STATE_FAULT) {
        // In fault: nothing to do here (wait for external reset)
        MotorStates_setTorqueCmd(0.0f);
        return;
    }

    if (!ctx.cmd.enable) {
        // Motor disabled: stay / go to IDLE
        MotorStates_setState(MOTOR_STATE_IDLE);
        MotorStates_setTorqueCmd(0.0f);
        PI_reset(&s_speed_pi);
        return;
    }

    // If we’re enabled but IDLE, bump into RUN
    if (ctx.state == MOTOR_STATE_IDLE) {
        MotorStates_setState(MOTOR_STATE_RUN);
    }

    // speed control only makes sense in RUN
    if (ctx.state == MOTOR_STATE_RUN) {
        float rpm_ref  = ctx.cmd.rpm_cmd;
        float rpm_meas = est.mech_speed;   // from speed estimator / halls

        PI_Status_t pi_status;
        float torque_cmd = PI_step(&s_speed_pi,
                                   rpm_ref,
                                   rpm_meas,
                                   true,         // use anti-windup
                                   &pi_status);

        // Clamp again (PI already clamped, but this is defensive)
        torque_cmd = clampf(torque_cmd, -MAX_TORQUE_CMD, +MAX_TORQUE_CMD);

        MotorStates_setTorqueCmd(torque_cmd);

        // Optional: log or react to PI saturation via pi_status
        (void)pi_status;
    }
}

// -----------------------------
// Fast loop (e.g. 10–20 kHz)
// - update position estimator
// - apply commutation based on sector & torque_cmd
// -----------------------------
void MotorControl_stepFast(void)
{
    MotorContext_t ctx = MotorStates_get();

    // Update position estimator each fast step
    PosEst_update();
    PosEst_t est = PosEst_get();

    if (ctx.state != MOTOR_STATE_RUN || !ctx.cmd.enable) {
        // Motor not running → no torque, outputs off / low duty
        MotorControl_applyCommutation(0, 0.0f, false);
        return;
    }

    if (!est.valid) {
        // No valid speed/position yet → you might want to do open-loop startup here.
        MotorControl_applyCommutation(0, 0.0f, ctx.cmd.direction);
        return;
    }

    uint8_t sector = est.sector;
    if (sector >= 6) {
        // Invalid sector
        MotorControl_applyCommutation(0, 0.0f, ctx.cmd.direction);
        return;
    }

    // Torque command is normalized [-1, 1]; convert to duty [0, 1]
    float tq = ctx.cmd.torque_cmd;
    float duty = fabsf(tq);

    // Clip duty; you may want to use a MAX_DUTY from motor_config.h
    duty = clampf(duty, 0.0f, 0.95f);

    bool direction = ctx.cmd.direction;

    // If you want torque sign to override direction:
    // if (tq < 0.0f) {
    //     direction = !direction;
    // }

    MotorControl_applyCommutation(sector, duty, direction);
}

// -----------------------------
// Local helper: map sector + direction to phase pattern
// and call your PWM HAL.
// -----------------------------
static void MotorControl_applyCommutation(uint8_t sector,
                                          float duty,
                                          bool direction)
{
    int u, v, w;

    // Convert hall sector -> phase state (+1/-1/0)
    uint8_t eff_sector = sector;

    // Simple direction handling: flip all signs when reversing
    HallComm_getPhaseState(eff_sector, &u, &v, &w);
    if (direction) {
        u = -u;
        v = -v;
        w = -w;
    }

    // At this point:
    //  u, v, w ∈ {-1, 0, +1}
    //  duty ∈ [0, 0.95]
    //
    // Map this to your PWM HAL. The exact API depends on your hal_pwm.h,
    // so the code below is example / pseudo-code.
    //
    // Replace with your actual functions.

#if 0
    // Example if you have something like:
    //   void HAL_PWM_setPhase(Phase_t phase, float duty, int sign);
    //
    HAL_PWM_setPhase(PHASE_U, duty, u);
    HAL_PWM_setPhase(PHASE_V, duty, v);
    HAL_PWM_setPhase(PHASE_W, duty, w);
#else
    // TODO: implement using your actual PWM HAL.
    (void)u;
    (void)v;
    (void)w;
    (void)duty;
    (void)direction;
#endif
}