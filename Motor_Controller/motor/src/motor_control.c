// motor_control.c

#include "motor_control.h"

#include "motor_config.h"
#include "position_estimator.h"
#include <string.h>   // memset
#include <math.h>     // fabsf

// ---------------- Tunable constants ----------------

// Max allowed change in commanded speed per second (rpm/s)
#define MOTOR_RPM_SLEW_RATE      2000.0f   // e.g. 2000 rpm per second

// Threshold below which we consider the motor "stopped enough" to flip direction
#define MOTOR_RPM_REV_THRESHOLD   100.0f   // rpm

// Threshold below which we consider the motor fully stopped for IDLE
#define MOTOR_RPM_STOP_THRESHOLD   50.0f   // rpm

// ---------------- Static context & handles ----------------

static MotorContext_t s_ctx;
static PwmMotor_t    *s_pwm = NULL;

// Current duty command (0..1) that fast loop will apply
static float s_duty_cmd = 0.0f;

// Slew / direction management
static float s_rpm_cmd_target  = 0.0f;  // internal target for slew
static float s_rpm_cmd_request = 0.0f;  // last requested rpm (user/API)

static bool  s_dir_current   = false;   // actual direction (0=fwd,1=rev)
static bool  s_dir_requested = false;   // requested direction

// ---------------- Simple speed PI controller ----------------

typedef struct {
    float kp;
    float ki;
    float integrator;
    float out_min;
    float out_max;
} SpeedPI_t;

static SpeedPI_t s_speed_pi;

// Reasonable defaults
#define SPEED_PI_KP_DEFAULT        0.0015f
#define SPEED_PI_KI_DEFAULT        0.0005f
#define SPEED_PI_OUT_MIN_DEFAULT   0.0f
#define SPEED_PI_OUT_MAX_DEFAULT   1.0f

static void SpeedPI_init(SpeedPI_t *pi)
{
    memset(pi, 0, sizeof(*pi));
    pi->kp      = SPEED_PI_KP_DEFAULT;
    pi->ki      = SPEED_PI_KI_DEFAULT;
    pi->out_min = SPEED_PI_OUT_MIN_DEFAULT;
    pi->out_max = SPEED_PI_OUT_MAX_DEFAULT;
}

static float SpeedPI_step(SpeedPI_t *pi, float error)
{
    // Integrate
    pi->integrator += pi->ki * error;

    // Anti-windup: clamp integrator
    if (pi->integrator > pi->out_max) pi->integrator = pi->out_max;
    if (pi->integrator < pi->out_min) pi->integrator = pi->out_min;

    float out = pi->kp * error + pi->integrator;

    if (out > pi->out_max) out = pi->out_max;
    if (out < pi->out_min) out = pi->out_min;

    return out;
}

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
    if (s_ctx.cmd.rpm_cmd > MOTOR_RPM_MAX)  s_ctx.cmd.rpm_cmd = MOTOR_RPM_MAX;
    if (s_ctx.cmd.rpm_cmd < 0.0f)          s_ctx.cmd.rpm_cmd = 0.0f;
}

// ---------------- Public API ----------------

void MotorControl_init(PwmMotor_t *pwm)
{
    s_pwm = pwm;

    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.state = MOTOR_STATE_IDLE;
    s_ctx.fault = MOTOR_FAULT_NONE;

    s_ctx.cmd.enable     = false;
    s_ctx.cmd.direction  = false;   // default forward (0=fwd,1=rev)
    s_ctx.cmd.rpm_cmd    = 0.0f;
    s_ctx.cmd.torque_cmd = 0.0f;

    s_duty_cmd           = 0.0f;

    s_rpm_cmd_target     = 0.0f;
    s_rpm_cmd_request    = 0.0f;

    s_dir_current        = false;   // forward
    s_dir_requested      = false;

    SpeedPI_init(&s_speed_pi);

    if (s_pwm) {
        PwmMotor_stop(s_pwm);       // ensure outputs off
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
    if (s_ctx.state == MOTOR_STATE_FAULT) {
        // Already in fault; overwrite or keep first cause as you prefer
        s_ctx.fault = fault;
        return;
    }

    s_ctx.fault = fault;
    s_ctx.state = MOTOR_STATE_FAULT;

    // Immediately shut everything down
    s_ctx.cmd.enable      = false;
    s_ctx.cmd.rpm_cmd     = 0.0f;
    s_rpm_cmd_target      = 0.0f;
    s_rpm_cmd_request     = 0.0f;
    s_duty_cmd            = 0.0f;

    if (s_pwm) {
        PwmMotor_stop(s_pwm);
    }
}

void MotorControl_updateBusVoltage(float vbus)
{
    // Store in measurement struct
    s_ctx.meas.v_bus = vbus;

    // If we're already in FAULT, don't spam more faults
    if (s_ctx.state == MOTOR_STATE_FAULT) {
        return;
    }

    // Simple OV / UV checking
    if (vbus > MOTOR_BUS_V_MAX_V) {
        MotorControl_setFault(MOTOR_FAULT_OVERVOLT);
    } else if (vbus < MOTOR_BUS_V_MIN_V && vbus > 0.1f) {
        // small >0 to ignore noise when bus is basically off
        MotorControl_setFault(MOTOR_FAULT_UNDERVOLT);
    }
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
    // In IDLE, keep outputs off
    s_duty_cmd = 0.0f;
    if (s_pwm) {
        PwmMotor_stop(s_pwm);
    }

    // Transition out of IDLE when enable is asserted and user
    // actually wants some non-zero speed
    if (s_ctx.cmd.enable && s_rpm_cmd_request > 0.0f) {
        s_ctx.state = MOTOR_STATE_RUN;
    }
}

static void handle_align_state(void)
{
    // Placeholder for alignment logic
    s_duty_cmd = 0.0f;
    if (s_pwm) {
        PwmMotor_stop(s_pwm);
    }

    // For now: quick "fake" align -> RUN
    s_ctx.state = MOTOR_STATE_RUN;
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

    // Speed PI: error = (slewed command) - measured
    float error_rpm = s_ctx.cmd.rpm_cmd - s_ctx.meas.rpm_mech;
    float duty      = SpeedPI_step(&s_speed_pi, error_rpm);

    // This is also our "torque command" for now (0..1)
    s_ctx.cmd.torque_cmd = duty;
    s_duty_cmd           = duty;
}

static void handle_fault_state(void)
{
    // Stay in FAULT until an explicit reset (not implemented yet)
    s_ctx.cmd.enable      = false;
    s_ctx.cmd.rpm_cmd     = 0.0f;
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

    float dt_s = 0.0f;
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
    // NOTE: This is called from fast loop (e.g. FAST_LOOP_HZ)

    // If not in RUN or enable is false or we’re in FAULT: outputs off
    if (s_ctx.state != MOTOR_STATE_RUN ||
        !s_ctx.cmd.enable ||
        s_ctx.fault != MOTOR_FAULT_NONE) {

        s_duty_cmd = 0.0f;
        if (s_pwm) {
            PwmMotor_stop(s_pwm);
        }
        return;
    }

    // Get latest position estimate
    PosEst_t pe = PosEst_get();

    // Sector is 0..5 for 6-step
    uint8_t sector = pe.sector;
    if (sector >= 6) {
        // Invalid sector -> treat as timing/position fault
        MotorControl_setFault(MOTOR_FAULT_TIMING);
        return;
    }

    // Use torque_cmd (0..1) as duty
    float duty = s_ctx.cmd.torque_cmd;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    // Direction: 0 = forward, 1 = reverse (we keep s_ctx.cmd.direction in sync)
    bool dir_fwd = (s_ctx.cmd.direction == 0);

    if (s_pwm) {
        PwmMotor_setSixStep(s_pwm, sector, duty, dir_fwd);
    }
}