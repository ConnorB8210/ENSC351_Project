// sensorless_handover.h
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "speed_measurement.h"   // SpeedEstimate_t, SpeedSource_t
#include "position_estimator.h"  // PosMode_t
#include "bemf_sector.h"         // BemfDir_t

typedef struct
{
    bool  enabled;           // handover logic active
    bool  done;              // true once we've switched to BEMF
    float min_rpm_mech;      // minimum mechanical RPM before we consider BEMF
    int   min_valid_samples; // how many consecutive valid samples required
    int   valid_count;       // running count of valid samples over threshold
} SensorlessHandover_t;

/**
 * @brief Initialize handover helper.
 *
 * @param h                  instance
 * @param min_rpm_mech       minimum mech RPM to allow handover (e.g. 500.0f)
 * @param min_valid_samples  consecutive samples over threshold (e.g. 50)
 */
void SensorlessHandover_init(SensorlessHandover_t *h,
                             float min_rpm_mech,
                             int   min_valid_samples);

/**
 * @brief Enable or disable the handover process.
 *
 * When disabled, this helper does nothing and never switches mode.
 */
void SensorlessHandover_setEnable(SensorlessHandover_t *h, bool enable);

/**
 * @brief Step the handover logic.
 *
 * Call this from the *slow loop* AFTER SpeedMeas_update(), while
 * you're still running in Hall mode.
 *
 * @param h              instance
 * @param now_s          current time in seconds (monotonic)
 * @param direction_fwd  true for forward, false for reverse
 *
 * @return true if the helper *just* completed the handover in this call.
 */
bool SensorlessHandover_step(SensorlessHandover_t *h,
                             float now_s,
                             bool direction_fwd);