// bemf_sector.h
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "bemf.h"

// Direction sign: +1 = forward, -1 = reverse
typedef enum {
    BEMF_DIR_FWD  = +1,
    BEMF_DIR_REV  = -1
} BemfDir_t;

typedef struct
{
    // Outputs
    uint8_t sector;       // 0..5 valid, 0xFF = invalid/unknown
    float   rpm_elec;     // electrical RPM
    float   rpm_mech;     // mechanical RPM
    float   last_period_s;
    bool    valid;

    // Internal state
    float   last_zc_time;     // last zero-cross time (s)
    float   prev_zc_time;     // previous zero-cross
    float   last_sample_time; // previous sample time (s)
    float   last_diff;        // last neutral diff on floating phase
    int     last_sign;        // -1 / 0 / +1
    BemfDir_t dir;            // direction for sector advance
} BemfSectorState_t;

/**
 * @brief Initialize BEMF sector detector.
 *
 * @param s            State object
 * @param start_sector Initial sector (0..5) after alignment
 * @param dir          Direction (BEMF_DIR_FWD / BEMF_DIR_REV)
 */
void BemfSector_init(BemfSectorState_t *s,
                     uint8_t start_sector,
                     BemfDir_t dir);

/**
 * @brief Set direction at runtime (e.g. fwd/rev command).
 */
void BemfSector_setDirection(BemfSectorState_t *s, BemfDir_t dir);

/**
 * @brief Force sector (e.g. after alignment or open-loop startup).
 */
void BemfSector_setSector(BemfSectorState_t *s, uint8_t sector);

/**
 * @brief Update sector & speed estimation from BEMF.
 *
 * Call this from the fast loop with:
 *   - BEMF handle (updated by Bemf_update() earlier in the loop)
 *   - current time in seconds
 *
 * Internally:
 *   - chooses floating phase from current sector
 *   - looks at Bemf_getNeutralDiff() on that phase
 *   - detects zero-crossings
 *   - on each ZC: advances sector and updates speed estimate
 */
void BemfSector_update(BemfSectorState_t *s,
                       const BemfHandle_t *bemf,
                       float now_s);

/**
 * @brief Get a copy of the current sector state.
 */
BemfSectorState_t BemfSector_get(const BemfSectorState_t *s);