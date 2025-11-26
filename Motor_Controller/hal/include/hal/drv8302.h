#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "gpio.h"

typedef struct
{
    GPIO_Handle *gpio;      // underlying generic GPIO handle
    unsigned int en_gate_idx;   // index into gpio->offsets[]
    unsigned int nfault_idx;
    unsigned int noctw_idx;
} Drv8302Hal_t;

/**
 * @brief Initialize DRV8302/DRV8301 gate & fault pins
 *
 * @param drv         HAL instance
 * @param chip_path   e.g. "/dev/gpiochip0"
 * @param en_gate_ofs GPIO line offset for EN_GATE
 * @param nfault_ofs  GPIO line offset for nFAULT
 * @param noctw_ofs   GPIO line offset for nOCTW
 *
 * The EN_GATE line is configured as output, nFAULT/nOCTW as inputs.
 */
bool Drv8302Hal_init(Drv8302Hal_t *drv,
                     const char *chip_path,
                     unsigned int en_gate_ofs,
                     unsigned int nfault_ofs,
                     unsigned int noctw_ofs);

/**
 * @brief Enable or disable the DRV gate driver (EN_GATE pin)
 */
void Drv8302Hal_setEnable(Drv8302Hal_t *drv, bool enable);

/**
 * @brief Read raw nFAULT pin (0 = fault active, 1 = OK)
 */
int Drv8302Hal_read_nFault(Drv8302Hal_t *drv);

/**
 * @brief Read raw nOCTW pin (0 = warning/OC/OT active, 1 = OK)
 */
int Drv8302Hal_read_nOctw(Drv8302Hal_t *drv);

/**
 * @brief Cleanup (releases underlying GPIO handle)
 */
void Drv8302Hal_deinit(Drv8302Hal_t *drv);