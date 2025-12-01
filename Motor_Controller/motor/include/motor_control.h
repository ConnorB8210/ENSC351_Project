#pragma once
#include <stdbool.h>
#include "motor_states.h"
#include "pwm_motor.h"
#define MOTOR_DISABLE_BUS_FAULTS 1
// Initialize motor control with a pointer to the phase driver
void MotorControl_init(PwmMotor_t *pwm);

// Called from the fast loop (e.g. FAST_LOOP_HZ)
// Handles commutation + duty application
void MotorControl_stepFast(void);

// Called from the slow loop (e.g. SPEED_LOOP_HZ)
// Handles state machine, PI speed control, slew limiting, etc.
void MotorControl_stepSlow(void);

// Get a snapshot of the current context (state, commands, measurements)
MotorContext_t MotorControl_getContext(void);

// High-level API: enable/disable motor (state machine will respect this)
void MotorControl_setEnable(bool en);

// High-level API: set requested speed and direction.
// rpm_cmd   : desired mechanical RPM (>=0)
// direction : 0 = forward, 1 = reverse
void MotorControl_setSpeedCmd(float rpm_cmd, bool direction);

// Report a fault (overcurrent, timing, hall timeout, etc.)
// This forces the state machine into MOTOR_STATE_FAULT and disables outputs.
void MotorControl_setFault(MotorFault_t fault);

// Explicitly clear a latched fault.
// Puts the controller back to MOTOR_STATE_IDLE with enable=false,
// and zeroes rpm/torque commands. Host must call setEnable() again.
void MotorControl_clearFault(void);

// Feed measured bus voltage into the controller.
// This stores v_bus into the measurement struct and automatically
// trips OVERVOLT / UNDERVOLT faults based on motor_config.h limits.
void MotorControl_updateBusVoltage(float vbus);