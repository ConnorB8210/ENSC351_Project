// motor_control.c
//
// High-level motor control:
//  - Manages MotorState_t (IDLE/RUN/FAULT)
//  - Runs speed PI loop
//  - Uses position_estimator / speed_measure for feedback
//  - Calls pwm_motor to apply 6-step commutation based on sector + direction

#include "motor_control.h"

#include "motor_states.h"
#include "position_estimator.h"
#include "speed_measurement.h"
#include "hall_commutator.h"
#include "pi_controller.h"
#include "filters.h"
#include "motor_config.h"
#include "pwm_motor.h"

#include <math.h>
#include <string.h>

// -----------------------------
// Config defaults
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
#define MAX_TORQUE_CMD       1.0f      // normalized torque command [-1..1]
#endif

#ifndef MAX_DUTY
#define MAX_DUTY             0.95f     // guard-band for PWM
#endif

// -----------------------------
// Internal state
// -----------------------------

static PI_Controller_t s_speed_pi;
static PosMode_t       s_pos_mode = POS_MODE_HALL;
static PwmMotor_t     *s_pwm      = NULL;

// Forward declaration
static void MotorControl_applyCommutation(uint8_t sector,
                                          float duty,
                                          bool direction);

// -----------------------------
// Public API
// -----------------------------

void MotorControl_init(PwmMotor_t *pwm)
{
    s_pwm = pwm;

    // Init shared context
    MotorStates_init();

    // Position / speed path
    s_pos_mode = POS_MODE_HALL;
    PosEst_init(s_pos_mode);
    SpeedMeas_init();

    // Speed PI
    float Ts_speed = 1.0f / SPEED_LOOP_HZ;
    PI_init(&s_speed_pi,
            SPEED_PI_KP,
            SPEED_PI_KI,
            Ts_speed,
            -MAX_TORQUE_CMD,
            +MAX_TORQUE_CMD);
    PI_reset(&s_speed_pi);

    // Start safe
    MotorStates_setState(MOTOR_STATE_IDLE);
    MotorStates_setEnable(false);
    MotorStates_setTorqueCmd(0.0f);

    // Outputs off
    if (s_pwm) {
        PwmMotor_setEnable(s_pwm, false);
        PwmMotor_applyPhaseState(s_pwm, 0, 0, 0, 0.0f);
    }
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
        MotorStates_setState(MOTOR_STATE_IDLE);
        MotorStates_setTorqueCmd(0.0f);
        PI_reset(&s_speed_pi);

        if (s_pwm) {
            PwmMotor_setEnable(s_pwm, false);
            PwmMotor_applyPhaseState(s_pwm, 0, 0, 0, 0.0f);
        }
    } else {
        if (ctx.state != MOTOR_STATE_FAULT) {
            MotorStates_setState(MOTOR_STATE_RUN);
        }
        if (s_pwm) {
            PwmMotor_setEnable(s_pwm, true);
        }
    }
}

MotorContext_t MotorControl_getContext(void)
{
    return MotorStates_get();
}

void MotorControl_onFault(void)
{
    // Latch fault and shut down torque/output
    MotorStates_setState(MOTOR_STATE_FAULT);
    MotorStates_setEnable(false);
    MotorStates_setTorqueCmd(0.0f);
    PI_reset(&s_speed_pi);

    if (s_pwm) {
        PwmMotor_setEnable(s_pwm, false);
        PwmMotor_applyPhaseState(s_pwm, 0, 0, 0, 0.0f);
    }
}

// -----------------------------
// Slow loop (e.g. 1 kHz)
// -----------------------------
void MotorControl_stepSlow(void)
{
    MotorContext_t ctx = MotorStates_get();
    PosEst_t       est = PosEst_get();

    if (ctx.state == MOTOR_STATE_FAULT) {
        MotorStates_setTorqueCmd(0.0f);
        return;
    }

    if (!ctx.cmd.enable) {
        MotorStates_setState(MOTOR_STATE_IDLE);
        MotorStates_setTorqueCmd(0.0f);
        PI_reset(&s_speed_pi);
        return;
    }

    if (ctx.state == MOTOR_STATE_IDLE) {
        MotorStates_setState(MOTOR_STATE_RUN);
    }

    if (ctx.state == MOTOR_STATE_RUN) {
        float rpm_ref  = ctx.cmd.rpm_cmd;
        float rpm_meas = est.mech_speed;   // from speed estimator

        PI_Status_t status;
        float torque_cmd = PI_step(&s_speed_pi,
                                   rpm_ref,
                                   rpm_meas,
                                   true,      // anti-windup
                                   &status);

        torque_cmd = clampf(torque_cmd, -MAX_TORQUE_CMD, +MAX_TORQUE_CMD);
        MotorStates_setTorqueCmd(torque_cmd);

        (void)status; // optional: inspect saturation
    }
}

// -----------------------------
// Fast loop (e.g. 10–20 kHz)
// -----------------------------
void MotorControl_stepFast(void)
{
    MotorContext_t ctx = MotorStates_get();

    // Update estimator
    PosEst_update();
    PosEst_t est = PosEst_get();

    if (!s_pwm) {
        return; // no PWM backend configured
    }

    if (ctx.state != MOTOR_STATE_RUN || !ctx.cmd.enable) {
        MotorControl_applyCommutation(0, 0.0f, false);
        return;
    }

    if (!est.valid) {
        // No valid sector yet → keep outputs off (or do open-loop startup later)
        MotorControl_applyCommutation(0, 0.0f, ctx.cmd.direction);
        return;
    }

    uint8_t sector = est.sector;
    if (sector >= 6) {
        MotorControl_applyCommutation(0, 0.0f, ctx.cmd.direction);
        return;
    }

    float tq   = ctx.cmd.torque_cmd;
    float duty = fabsf(tq);

    duty = clampf(duty, 0.0f, MAX_DUTY);

    bool direction = ctx.cmd.direction;

    // Optional: let torque sign flip direction instead:
    // if (tq < 0.0f) {
    //     direction = !direction;
    // }

    MotorControl_applyCommutation(sector, duty, direction);
}

// -----------------------------
// Local helper: sector -> phase state -> PWM
// -----------------------------
static void MotorControl_applyCommutation(uint8_t sector,
                                          float duty,
                                          bool direction)
{
    if (!s_pwm) return;

    int u, v, w;

    HallComm_getPhaseState(sector, &u, &v, &w);

    if (direction) {
        u = -u;
        v = -v;
        w = -w;
    }

    PwmMotor_applyPhaseState(s_pwm, u, v, w, duty);
}