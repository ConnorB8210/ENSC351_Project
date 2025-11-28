#pragma once
#include <stdbool.h>
#include "motor_states.h"

void MotorControl_init(void);

// Called at the **fast** control rate (e.g. 10 kHz)
void MotorControl_stepFast(void);

// Called at the **slow** rate (e.g. 1 kHz) for speed loop etc.
void MotorControl_stepSlow(void);

// High-level API used by app layer:
void MotorControl_setSpeedCmd(float rpm, bool direction);
void MotorControl_setEnable(bool enable);
MotorContext_t MotorControl_getContext(void);

// Fault & state handling
void MotorControl_onFault(void);