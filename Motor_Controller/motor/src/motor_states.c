#include "motor_states.h"
#include <string.h>     // memset

// ---------------------------------------------------------
// Global motor context (owned by control layer)
// ---------------------------------------------------------
static MotorContext_t g_motor_ctx;


// ---------------------------------------------------------
// Initialization
// ---------------------------------------------------------
void MotorStates_init(void)
{
    memset(&g_motor_ctx, 0, sizeof(g_motor_ctx));

    g_motor_ctx.state = MOTOR_STATE_IDLE;

    // Measurements default to 0
    g_motor_ctx.meas.rpm_mech  = 0.0f;
    g_motor_ctx.meas.rpm_elec  = 0.0f;
    g_motor_ctx.meas.i_bus     = 0.0f;
    g_motor_ctx.meas.i_phase_u = 0.0f;
    g_motor_ctx.meas.i_phase_v = 0.0f;
    g_motor_ctx.meas.i_phase_w = 0.0f;
    g_motor_ctx.meas.v_bus     = 0.0f;

    // Default commands
    g_motor_ctx.cmd.rpm_cmd    = 0.0f;
    g_motor_ctx.cmd.torque_cmd = 0.0f;
    g_motor_ctx.cmd.enable     = false;
    g_motor_ctx.cmd.direction  = false;   // forward
}


// ---------------------------------------------------------
// Getters
// ---------------------------------------------------------
MotorContext_t MotorStates_get(void)
{
    return g_motor_ctx;      // returned by value (safe snapshot)
}

MotorContext_t *MotorStates_ptr(void)
{
    return &g_motor_ctx;     // direct pointer (optional)
}


// ---------------------------------------------------------
// Command setters
// ---------------------------------------------------------
void MotorStates_setCommand(const MotorCommand_t *cmd)
{
    if (!cmd) return;
    g_motor_ctx.cmd = *cmd;
}

void MotorStates_setEnable(bool en)
{
    g_motor_ctx.cmd.enable = en;
}

void MotorStates_setDirection(bool dir)
{
    g_motor_ctx.cmd.direction = dir;
}

void MotorStates_setSpeedCmd(float rpm)
{
    g_motor_ctx.cmd.rpm_cmd = rpm;
}

void MotorStates_setTorqueCmd(float tq)
{
    g_motor_ctx.cmd.torque_cmd = tq;
}


// ---------------------------------------------------------
// Measurement setters
// ---------------------------------------------------------
void MotorStates_setMeasurements(const MotorMeasurements_t *meas)
{
    if (!meas) return;
    g_motor_ctx.meas = *meas;
}

void MotorStates_updateElectricalSpeed(float rpm_elec)
{
    g_motor_ctx.meas.rpm_elec = rpm_elec;
}

void MotorStates_updateMechanicalSpeed(float rpm_mech)
{
    g_motor_ctx.meas.rpm_mech = rpm_mech;
}

void MotorStates_updateVbus(float v)
{
    g_motor_ctx.meas.v_bus = v;
}

void MotorStates_updateCurrents(float iu, float iv, float iw, float ibus)
{
    g_motor_ctx.meas.i_phase_u = iu;
    g_motor_ctx.meas.i_phase_v = iv;
    g_motor_ctx.meas.i_phase_w = iw;
    g_motor_ctx.meas.i_bus     = ibus;
}


// ---------------------------------------------------------
// State machine setter
// ---------------------------------------------------------
void MotorStates_setState(MotorState_t new_state)
{
    g_motor_ctx.state = new_state;
}