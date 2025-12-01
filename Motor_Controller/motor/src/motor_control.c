// motor_control.c
#include "motor_control.h"
#include "motor_config.h"
#include "position_estimator.h"
#include "pi_controller.h"    // <-- use shared PI controller
#include <string.h>           // memset
#include <math.h>             // fabsf

// ---------------- Tunable constants ----------------

// Max allowed change in commanded speed per second (rpm/s)
#define MOTOR_RPM_SLEW_RATE      2000.0f   // e.g. 2000 rpm per second

// Threshold below which we consider the motor "stopped enough" to flip direction
#define MOTOR_RPM_REV_THRESHOLD   100.0f   // rpm

// Threshold below which we consider the motor fully stopped for IDLE
#define MOTOR_RPM_STOP_THRESHOLD   50.0f   // rpm

// Startup (open-loop) commutation settings
#define STARTUP_DUTY             0.20f     // fixed duty during open-loop
#define STARTUP_STEPS_TOTAL      36        // number of sector steps (e.g. 6 sectors * 6 revs)
#define STARTUP_TICKS_PER_STEP   5         // how many slow-loop ticks per sector
#define STARTUP_HANDOVER_RPM     50.0f     // when rpm_mech > this, hand over to RUN

// PI controller defaults (for speed loop)
#define SPEED_PI_KP_DEFAULT        0.0015f
#define SPEED_PI_KI_DEFAULT        0.0005f
#define SPEED_PI_OUT_MIN_DEFAULT   0.0f
#define SPEED_PI_OUT_MAX_DEFAULT   1.0f

// ---------------- Static context & handles ----------------

static MotorContext_t s_ctx;
static PwmMotor_t    *s_pwm = NULL;

// Current duty command (0..1) that fast loop will apply
static float s_duty_cmd = 0.0f;

// Slew / direction management
static float s_rpm_cmd_target  = 0.0f;  // internal target for slew
static float s_rpm_cmd_request = 0.0f;  // last requested rpm (user/API)
static bool  s_dir_current     = false; // actual direction (0=fwd,1=rev)
static bool  s_dir_requested   = false; // requested direction

// Startup open-loop state
static int      s_startup_active       = 0;
static uint8_t  s_startup_sector       = 0;
static uint32_t s_startup_step_count   = 0;
static uint32_t s_startup_tick_in_step = 0;

// Shared PI controller instance (from pi_controller.c)
static PI_Controller_t s_speed_pi;

// ---------------- Slew‑rate & direction logic ----------------

// Update s_rpm_cmd_target and direction based on requested values and actual speed.
static void update_target_and_direction(void)
{
    float rpm_abs = fabsf(s_ctx.meas.rpm_mech);

    if (s_dir_requested != s_dir_current) {
        // Direction change requested
        if (rpm_abs <= MOTOR_RPM_REV_THRESHOLD) {
            // Slow enough: flip direction now
            s_dir_current       = s_dir_requested;
            s_ctx.cmd.direction = s_dir_current;     // 0=fwd,1=rev
            s_rpm_cmd_target    = s_rpm_cmd_request;
        } else {
            // Too fast to reverse: brake toward zero (target = 0)
            s_rpm_cmd_target    = 0.0f;
        }
    } else {
        // Direction unchanged: follow requested rpm
        s_rpm_cmd_target = s_rpm_cmd_request;
    }

    // Clamp target
    if (s_rpm_cmd_target > MOTOR_RPM_MAX)  s_rpm_cmd_target = MOTOR_RPM_MAX;
    if (s_rpm_cmd_target < 0.0f)           s_rpm_cmd_target = 0.0f;
}

// Slew s_ctx.cmd.rpm_cmd toward s_rpm_cmd_target with a rate limit.
static void update_speed_slew(void)
{
    // Per-step max change in RPM, based on configured slow loop rate
    float max_step = MOTOR_RPM_SLEW_RATE / (float)SPEED_LOOP_HZ;

    float diff = s_rpm_cmd_target - s_ctx.cmd.rpm_cmd;
    if (diff > max_step) {
        diff = max_step;
    } else if (diff < -max_step) {
        diff = -max_step;
    }

    s_ctx.cmd.rpm_cmd += diff;

    // Safety clamp
    if (s_ctx.cmd.rpm_cmd > MOTOR_RPM_MAX) s_ctx.cmd.rpm_cmd = MOTOR_RPM_MAX;
    if (s_ctx.cmd.rpm_cmd < 0.0f)          s_ctx.cmd.rpm_cmd = 0.0f;
}

// ---------------- Public API ----------------

void MotorControl_init(PwmMotor_t *pwm)
{
    s_pwm = pwm;
    memset(&s_ctx, 0, sizeof(s_ctx));

    s_ctx.state         = MOTOR_STATE_IDLE;
    s_ctx.fault         = MOTOR_FAULT_NONE;
    s_ctx.cmd.enable    = false;
    s_ctx.cmd.direction = false;   // default forward (0=fwd,1=rev)
    s_ctx.cmd.rpm_cmd   = 0.0f;
    s_ctx.cmd.torque_cmd= 0.0f;

    s_duty_cmd          = 0.0f;
    s_rpm_cmd_target    = 0.0f;
    s_rpm_cmd_request   = 0.0f;
    s_dir_current       = false;   // forward
    s_dir_requested     = false;

    // startup state
    s_startup_active       = 0;
    s_startup_sector       = 0;
    s_startup_step_count   = 0;
    s_startup_tick_in_step = 0;

    // Initialize the shared speed PI controller
    float Ts = 1.0f / (float)SPEED_LOOP_HZ;  // slow-loop period
    PI_init(&s_speed_pi,
            SPEED_PI_KP_DEFAULT,
            SPEED_PI_KI_DEFAULT,
            Ts,
            SPEED_PI_OUT_MIN_DEFAULT,
            SPEED_PI_OUT_MAX_DEFAULT);

    if (s_pwm) {
        PwmMotor_stop(s_pwm);      // ensure outputs off
    }
}

MotorContext_t MotorControl_getContext(void)
{
    return s_ctx;
}

void MotorControl_setEnable(bool en)
{
    // If we’re in FAULT, ignore attempts to re-enable
    if (s_ctx.state == MOTOR_STATE_FAULT && en) {
        return;
    }
    s_ctx.cmd.enable = en;
}

void MotorControl_setSpeedCmd(float rpm_cmd, bool direction)
{
    // Clamp user request
    if (rpm_cmd > MOTOR_RPM_MAX)  rpm_cmd = MOTOR_RPM_MAX;
    if (rpm_cmd < 0.0f)           rpm_cmd = 0.0f;

    s_rpm_cmd_request = rpm_cmd;
    s_dir_requested   = direction;   // 0=fwd,1=rev
}

void MotorControl_setFault(MotorFault_t fault)
{
    // Don't clear faults or overwrite first cause here
    if (s_ctx.state == MOTOR_STATE_FAULT) {
        return;
    }

    s_ctx.fault = fault;
    s_ctx.state = MOTOR_STATE_FAULT;

    // Immediately shut everything down
    s_ctx.cmd.enable      = false;
    s_ctx.cmd.rpm_cmd     = 0.0f;
    s_rpm_cmd_target      = 0.0f;
    s_ctx.cmd.torque_cmd  = 0.0f;
    s_rpm_cmd_request     = 0.0f;
    s_duty_cmd            = 0.0f;

    if (s_pwm) {
        PwmMotor_stop(s_pwm);
    }
}

// Explicit clear-fault API: call from UDP or UI when it's safe to try again.
void MotorControl_clearFault(void)
{
    // Reset fault and state, but keep motor disabled so host must re-enable.
    s_ctx.fault          = MOTOR_FAULT_NONE;
    s_ctx.state          = MOTOR_STATE_IDLE;
    s_ctx.cmd.enable     = false;
    s_ctx.cmd.rpm_cmd    = 0.0f;
    s_ctx.cmd.torque_cmd = 0.0f;
    s_rpm_cmd_target     = 0.0f;
    s_rpm_cmd_request    = 0.0f;
    s_duty_cmd           = 0.0f;

    // Reset startup sequence
    s_startup_active       = 0;
    s_startup_sector       = 0;
    s_startup_step_count   = 0;
    s_startup_tick_in_step = 0;

    // Reset PI integrator
    PI_reset(&s_speed_pi);
    // Don't touch s_dir_current / s_dir_requested; let host decide direction.
}

void MotorControl_updateBusVoltage(float vbus)
{
    s_ctx.meas.v_bus = vbus;

    if (s_ctx.state == MOTOR_STATE_FAULT) {
        return;
    }

#ifndef MOTOR_DISABLE_BUS_FAULTS
    if (vbus > MOTOR_BUS_V_MAX_V) {
        MotorControl_setFault(MOTOR_FAULT_OVERVOLT);
    } else if (vbus < MOTOR_BUS_V_MIN_V && vbus > 0.1f) {
        MotorControl_setFault(MOTOR_FAULT_UNDERVOLT);
    }
#endif
}

// ---------------- Internal helpers ----------------

static void update_measurements(void)
{
    // Use Position Estimator for RPM; it already pulls from SpeedMeasurement
    PosEst_t pe = PosEst_get();
    s_ctx.meas.rpm_mech = pe.mech_speed;
    s_ctx.meas.rpm_elec = pe.elec_speed;
    // TODO: wire actual bus current/phase currents if you have them
}

// State handlers

static void handle_idle_state(void)
{
    s_duty_cmd = 0.0f;
    if (s_pwm) {
        PwmMotor_stop(s_pwm);
    }

    // Transition out of IDLE when enable is asserted and user
    // actually wants some non-zero speed
    if (s_ctx.cmd.enable && s_rpm_cmd_request > 0.0f) {
        // initialize startup sequence
        s_startup_active       = 1;
        s_startup_step_count   = 0;
        s_startup_tick_in_step = 0;

        // try to start from current hall sector if valid, else 0
        PosEst_t pe = PosEst_get();
        if (pe.sector < 6) {
            s_startup_sector = pe.sector;
        } else {
            s_startup_sector = 0;
        }

        s_ctx.state = MOTOR_STATE_ALIGN;   // use ALIGN as "startup" state
    }
}

static void handle_align_state(void)
{
    // Open-loop 6-step startup.
    // We ignore the PI loop here and just drive a fixed duty and manually
    // advance sector. Once the rotor has some speed (or we've done enough
    // steps), we transition to RUN and let the normal loop take over.

    // If somehow disabled or user zeroed the command, bail back to IDLE:
    if (!s_ctx.cmd.enable || s_rpm_cmd_request <= 0.0f) {
        s_ctx.state          = MOTOR_STATE_IDLE;
        s_startup_active     = 0;
        s_ctx.cmd.rpm_cmd    = 0.0f;
        s_ctx.cmd.torque_cmd = 0.0f;
        s_duty_cmd           = 0.0f;
        if (s_pwm) {
            PwmMotor_stop(s_pwm);
        }
        return;
    }

    // Force a known duty during startup
    s_ctx.cmd.torque_cmd = STARTUP_DUTY;
    s_duty_cmd           = STARTUP_DUTY;

    // Update counters at slow-loop rate
    if (s_startup_active) {
        s_startup_tick_in_step++;
        if (s_startup_tick_in_step >= STARTUP_TICKS_PER_STEP) {
            s_startup_tick_in_step = 0;
            s_startup_step_count++;
            s_startup_sector = (uint8_t)((s_startup_sector + 1) % 6);
        }
    }

    // If we've reached some RPM or finished the open-loop sequence,
    // hand over to RUN.
    float rpm_abs = fabsf(s_ctx.meas.rpm_mech);
    if (rpm_abs > STARTUP_HANDOVER_RPM ||
        s_startup_step_count >= STARTUP_STEPS_TOTAL) {
        s_startup_active     = 0;
        s_ctx.state          = MOTOR_STATE_RUN;
        // initialize PI state so it doesn't jump too hard
        s_ctx.cmd.rpm_cmd    = s_rpm_cmd_request;
        s_ctx.cmd.torque_cmd = STARTUP_DUTY;
        PI_reset(&s_speed_pi);
    }
}

static void handle_run_state(float dt_s)
{
    (void)dt_s; // currently unused; reserved for future

    float rpm_abs = fabsf(s_ctx.meas.rpm_mech);

    // If motor disabled or user requested zero speed and we're basically stopped,
    // fall back to IDLE.
    if (!s_ctx.cmd.enable ||
        (s_rpm_cmd_request <= 0.0f && rpm_abs < MOTOR_RPM_STOP_THRESHOLD)) {
        s_ctx.state          = MOTOR_STATE_IDLE;
        s_ctx.cmd.rpm_cmd    = 0.0f;
        s_rpm_cmd_target     = 0.0f;
        s_duty_cmd           = 0.0f;
        if (s_pwm) {
            PwmMotor_stop(s_pwm);
        }
        return;
    }

    // Speed PI: ref = slewed rpm command, meas = actual rpm
    PI_Status_t pi_status;
    float duty = PI_step(&s_speed_pi,
                         s_ctx.cmd.rpm_cmd,      // ref
                         s_ctx.meas.rpm_mech,    // meas
                         true,                   // use anti-windup
                         &pi_status);            // optional, can be ignored

    // Clamp for safety as well
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    // This is also our "torque command" for now (0..1)
    s_ctx.cmd.torque_cmd = duty;
    s_duty_cmd           = duty;
}

static void handle_fault_state(void)
{
    // Stay in FAULT until an explicit reset
    s_ctx.cmd.enable      = false;
    s_ctx.cmd.rpm_cmd     = 0.0f;
    s_ctx.cmd.torque_cmd  = 0.0f;
    s_rpm_cmd_target      = 0.0f;
    s_rpm_cmd_request     = 0.0f;
    s_duty_cmd            = 0.0f;

    if (s_pwm) {
        PwmMotor_stop(s_pwm);
    }
}

// ---------------- Slow loop ----------------

void MotorControl_stepSlow(void)
{
    // NOTE: This is called from slow loop (e.g. SPEED_LOOP_HZ Hz)
    static float s_last_time_s = 0.0f;

    float now_s = 0.0f; // If you want real dt, pass now_s in from caller
    float dt_s  = 0.0f;

    if (s_last_time_s > 0.0f) {
        dt_s = now_s - s_last_time_s;
        if (dt_s < 0.0f) dt_s = 0.0f;
    }
    s_last_time_s = now_s;

    // 1) Update measurements (speed, etc.)
    update_measurements();

    // 2) Update internal target and direction based on user requests & actual speed
    update_target_and_direction();

    // 3) Slew rpm_cmd toward target
    update_speed_slew();

    // 4) State machine
    switch (s_ctx.state) {
    case MOTOR_STATE_IDLE:
        handle_idle_state();
        break;
    case MOTOR_STATE_ALIGN:
        handle_align_state();
        break;
    case MOTOR_STATE_RUN:
        handle_run_state(dt_s);
        break;
    case MOTOR_STATE_FAULT:
    default:
        handle_fault_state();
        break;
    }
}

// ---------------- Fast loop ----------------

void MotorControl_stepFast(void)
{
    // If disabled or faulted, always turn everything off.
    if (!s_ctx.cmd.enable || s_ctx.fault != MOTOR_FAULT_NONE) {
        s_duty_cmd = 0.0f;
        if (s_pwm) {
            PwmMotor_stop(s_pwm);
        }
        return;
    }

    // ALIGN = open-loop startup
    if (s_ctx.state == MOTOR_STATE_ALIGN) {
        // Use the startup sector and fixed duty
        uint8_t sector = s_startup_sector;
        if (sector >= 6) {
            sector = 0;
        }

        float duty = s_ctx.cmd.torque_cmd;
        if (duty < 0.0f) duty = 0.0f;
        if (duty > 1.0f) duty = 1.0f;

        bool dir_fwd = (s_ctx.cmd.direction == 0);
        if (s_pwm) {
            PwmMotor_setSixStep(s_pwm, sector, duty, dir_fwd);
        }
        return;
    }

    // Normal RUN mode (closed-loop with PI)
    if (s_ctx.state != MOTOR_STATE_RUN) {
        // any other state => outputs off
        s_duty_cmd = 0.0f;
        if (s_pwm) {
            PwmMotor_stop(s_pwm);
        }
        return;
    }

    // RUN: use estimator sector + PI duty
    PosEst_t pe = PosEst_get();
    uint8_t sector = pe.sector;
    if (sector >= 6) {
        MotorControl_setFault(MOTOR_FAULT_TIMING);
        return;
    }

    float duty = s_ctx.cmd.torque_cmd;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    bool dir_fwd = (s_ctx.cmd.direction == 0);
    if (s_pwm) {
        PwmMotor_setSixStep(s_pwm, sector, duty, dir_fwd);
    }
}