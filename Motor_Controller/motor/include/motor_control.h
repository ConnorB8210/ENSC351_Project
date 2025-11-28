// motor_control.h
#pragma once

#include <stdbool.h>
#include "motor_states.h"
#include "pwm_motor.h"

/**
 * @brief Initialize motor control module.
 *
 * @param pwm Pointer to initialized PwmMotor_t instance.
 */
void MotorControl_init(PwmMotor_t *pwm);

void MotorControl_setSpeedCmd(float rpm, bool direction);
void MotorControl_setEnable(bool enable);

MotorContext_t MotorControl_getContext(void);
void MotorControl_onFault(void);

/**
 * @brief Slow control loop (e.g. 1 kHz).
 */
void MotorControl_stepSlow(void);

/**
 * @brief Fast control loop (e.g. 10–20 kHz).
 */
void MotorControl_stepFast(void);