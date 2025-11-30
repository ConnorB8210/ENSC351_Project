#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "bemf.h"

// Direction of rotation for BEMF sector tracking
typedef enum {
    BEMF_DIR_FWD = 0,
    BEMF_DIR_REV = 1
} BemfDir_t;

// State of the BEMF-based sector/speed estimator
typedef struct
{
    float    last_zero_ts;    // last zero-crossing time [s]
    float    last_period_s;   // last sector period [s] (60 el. deg segment)

    float    rpm_elec;        // estimated electrical speed [rpm]
    float    rpm_mech;        // estimated mechanical speed [rpm]

    uint8_t  sector;          // current electrical sector 0..5
    bool     zero_valid;      // we have seen at least one ZC
    bool     valid;           // overall validity flag

    BemfDir_t dir;            // assumed direction of rotation
} BemfSectorState_t;

/**
 * @brief Initialize BEMF sector tracking state.
 *
 * @param s            state struct
 * @param start_sector initial electrical sector 0..5
 * @param dir          assumed direction (BEMF_DIR_FWD / BEMF_DIR_REV)
 */
void BemfSector_init(BemfSectorState_t *s,
                     uint8_t start_sector,
                     BemfDir_t dir);

/**
 * @brief Update BEMF-based sector & speed estimate.
 *
 * Call this from the SLOW loop (e.g. 1 kHz) *after* Bemf_update()
 * has refreshed phase & Vbus voltages.
 *
 * @param s     state
 * @param bemf  pointer to BemfHandle_t (for voltages)
 * @param now_s current time [s] (monotonic)
 */
void BemfSector_update(BemfSectorState_t *s,
                       const BemfHandle_t *bemf,
                       float now_s);

/**
 * @brief Get a copy of the current sector state.
 */
BemfSectorState_t BemfSector_get(const BemfSectorState_t *s);