// motor_states.h
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "motor_config.h"

typedef enum {
    MOTOR_FAULT_NONE = 0,
    MOTOR_FAULT_OVERCURRENT,
    MOTOR_FAULT_OVERVOLT,
    MOTOR_FAULT_UNDERVOLT,
    MOTOR_FAULT_HALL_TIMEOUT,
    MOTOR_FAULT_DRV8302,
    MOTOR_FAULT_TIMING     // <-- NEW: fast-loop/jitter/timing fault
} MotorFault_t;

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
    bool  direction;   // 0=fwd, 1=rev
} MotorCommand_t;

typedef struct {
    MotorState_t        state;
    MotorFault_t        fault;   // <-- make sure this exists
    MotorMeasurements_t meas;
    MotorCommand_t      cmd;
} MotorContext_t;