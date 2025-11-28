#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "gpio.h"

/**
 * Motor PWM / gate driver abstraction for DRV830x-style boards.
 *
 * For each phase we control two gate lines:
 *   - INH_X : high-side gate input
 *   - INL_X : low-side gate input
 *
 * PwmMotor_applyPhaseState() takes per-phase sign (+1, -1, 0) and
 * a normalized duty [0..1], and maps it onto those six lines.
 *
 * NOTE: This version uses simple GPIO high/low outputs; it does NOT
 * yet generate real high-frequency PWM. That can be added later by
 * swapping the internals while keeping this API.
 */

typedef struct
{
    GPIO_Handle *gpio;  // underlying GPIO handle

    // Indexes into gpio->offsets[] for each gate line
    unsigned int inh_a_idx;
    unsigned int inl_a_idx;
    unsigned int inh_b_idx;
    unsigned int inl_b_idx;
    unsigned int inh_c_idx;
    unsigned int inl_c_idx;

    bool enabled;
} PwmMotor_t;

/**
 * @brief Initialize motor PWM/gate outputs.
 *
 * All gate lines are configured as outputs and driven low.
 *
 * @param m             Motor PWM instance
 * @param chip_path     e.g. "/dev/gpiochip0"
 * @param inh_a_offset  GPIO offset for INH_A
 * @param inl_a_offset  GPIO offset for INL_A
 * @param inh_b_offset  GPIO offset for INH_B
 * @param inl_b_offset  GPIO offset for INL_B
 * @param inh_c_offset  GPIO offset for INH_C
 * @param inl_c_offset  GPIO offset for INL_C
 * @return true on success, false on error
 */
bool PwmMotor_init(PwmMotor_t *m,
                   const char *chip_path,
                   unsigned int inh_a_offset,
                   unsigned int inl_a_offset,
                   unsigned int inh_b_offset,
                   unsigned int inl_b_offset,
                   unsigned int inh_c_offset,
                   unsigned int inl_c_offset);

/**
 * @brief Enable or disable the motor outputs (all phases).
 *
 * When disabled, all gate lines are driven low.
 */
void PwmMotor_setEnable(PwmMotor_t *m, bool enable);

/**
 * @brief Apply phase state for 3-phase BLDC commutation.
 *
 * u, v, w are phase "signs":
 *   +1 = high-side active  (INH=1, INL=0)
 *   -1 = low-side active   (INH=0, INL=1)
 *    0 = phase off/floating (INH=0, INL=0)
 *
 * duty is a normalized magnitude [0..1]. For now this implementation
 * only uses duty as "on/off" (duty > 0 → active). Later you can wire
 * this into a real hardware PWM backend.
 */
void PwmMotor_applyPhaseState(PwmMotor_t *m,
                              int u, int v, int w,
                              float duty);

/**
 * @brief Drive all outputs low and release GPIO handle.
 */
void PwmMotor_deinit(PwmMotor_t *m);