#pragma once
#include <stdbool.h>

typedef enum {
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_ALIGN,
    MOTOR_STATE_RUN,
    MOTOR_STATE_FAULT
} MotorState_t;

typedef struct {
    float rpm_mech;
    float rpm_elec;
    float i_bus;
    float i_phase_u;
    float i_phase_v;
    float i_phase_w;
    float v_bus;
} MotorMeasurements_t;

typedef struct {
    float rpm_cmd;
    float torque_cmd;
    bool  enable;
    bool  direction;   // 0=fwd,1=rev
} MotorCommand_t;

typedef struct {
    MotorState_t        state;
    MotorMeasurements_t meas;
    MotorCommand_t      cmd;
} MotorContext_t;


// ------------ API ------------
void MotorStates_init(void);

MotorContext_t MotorStates_get(void);
MotorContext_t *MotorStates_ptr(void);

void MotorStates_setCommand(const MotorCommand_t *cmd);
void MotorStates_setEnable(bool en);
void MotorStates_setDirection(bool dir);
void MotorStates_setSpeedCmd(float rpm);
void MotorStates_setTorqueCmd(float tq);

void MotorStates_setMeasurements(const MotorMeasurements_t *meas);
void MotorStates_updateElectricalSpeed(float rpm_elec);
void MotorStates_updateMechanicalSpeed(float rpm_mech);
void MotorStates_updateVbus(float v);
void MotorStates_updateCurrents(float iu, float iv, float iw, float ibus);

void MotorStates_setState(MotorState_t new_state);
