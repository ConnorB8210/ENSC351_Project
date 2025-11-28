// position_estimator.c

#include "position_estimator.h"
#include "motor_config.h"
#include "speed_measurement.h"   // for SpeedMeas_get()
#include <string.h>          // memset

// Current estimator mode (Hall or BEMF)
static PosMode_t s_mode = POS_MODE_HALL;

// Latest position/speed estimate
static PosEst_t s_est;

// ---------------------------------------------------------
// Initialization
// ---------------------------------------------------------
void PosEst_init(PosMode_t mode)
{
    memset(&s_est, 0, sizeof(s_est));
    s_mode      = mode;
    s_est.valid = false;
}

// Change estimator mode (e.g. HALL <-> BEMF)
void PosEst_setMode(PosMode_t mode)
{
    s_mode = mode;
    // optional: reset estimate when changing mode
    s_est.elec_angle = 0.0f;
    s_est.elec_speed = 0.0f;
    s_est.mech_speed = 0.0f;
    s_est.sector     = 0;
    s_est.valid      = false;
}

// ---------------------------------------------------------
// Main update (called from fast or slow control loop)
// ---------------------------------------------------------
void PosEst_update(void)
{
    // Right now we mostly hook into the speed measurement module.
    // Sector/angle logic can be filled in once Hall/BEMF paths are ready.
    SpeedEstimate_t spd = SpeedMeas_get();

    s_est.mech_speed = spd.rpm_mech;
    s_est.elec_speed = spd.rpm_elec;
    s_est.valid      = spd.valid;

    // TODO: fill these in properly once Hall/BEMF plumbing is done.
    // For now we just leave sector/angle at 0 so things compile & run.

    if (!spd.valid) {
        s_est.sector     = 0;
        s_est.elec_angle = 0.0f;
        return;
    }

    if (s_mode == POS_MODE_HALL) {
        // TODO:
        //  - read hall bits
        //  - convert to sector via HallComm_hallToSector()
        //  - map sector -> approximate electrical angle (e.g. center of sector)
        //
        // Example once you have hall bits:
        //
        //   uint8_t hall_bits = ...; // from your Hall HAL
        //   uint8_t sector = HallComm_hallToSector(hall_bits);
        //   s_est.sector = sector;
        //   s_est.elec_angle = (float)sector * (2.0f * M_PI / 6.0f);
        //
        s_est.sector     = 0;
        s_est.elec_angle = 0.0f;
    }
    else { // POS_MODE_BEMF
        // TODO:
        //  - use BEMF zero‑cross / sector detection
        //  - estimate electrical angle from floating phase BEMF
        s_est.sector     = 0;
        s_est.elec_angle = 0.0f;
    }
}

// ---------------------------------------------------------
// Getter
// ---------------------------------------------------------
PosEst_t PosEst_get(void)
{
    return s_est;
}