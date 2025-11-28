// speed_measurement.h
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "hall.h"
#include "bemf.h"
#include "bemf_sector.h"

typedef enum {
    SPEED_SRC_HALL = 0,
    SPEED_SRC_BEMF
} SpeedSource_t;

typedef struct {
    float   rpm_mech;
    float   rpm_elec;
    float   last_period_s;
    uint8_t sector;      // 0..5 valid, 0xFF = invalid / unknown
    bool    valid;
} SpeedEstimate_t;

void SpeedMeas_init(void);

/**
 * @brief Select whether speed/sector comes from HALL or BEMF.
 */
void SpeedMeas_setMode(SpeedSource_t src);

/**
 * @brief Attach Hall handle (used in HALL mode).
 */
void SpeedMeas_setHallHandle(HallHandle_t *hh);

/**
 * @brief Attach BEMF handle (used in BEMF mode).
 *
 * The ADC/BEMF module is updated elsewhere via Bemf_update().
 */
void SpeedMeas_setBemfHandle(BemfHandle_t *bh);

/**
 * @brief Initialize BEMF tracking after alignment / open-loop startup.
 *
 * @param start_sector  initial sector (0..5)
 * @param dir           direction (BEMF_DIR_FWD / BEMF_DIR_REV)
 */
void SpeedMeas_bemfAlign(uint8_t start_sector, BemfDir_t dir);

/**
 * @brief Update speed estimation.
 *
 * In HALL mode:
 *   - reads Hall bits
 *   - detects sector changes
 *   - computes RPM from time between sector edges
 *
 * In BEMF mode:
 *   - uses BemfSector_update() + BemfSectorState_t
 *
 * Call this from a periodic (fast) task with monotonic now_s [seconds].
 */
void SpeedMeas_update(float now_s);

/**
 * @brief Get latest speed + sector estimate.
 */
SpeedEstimate_t SpeedMeas_get(void);