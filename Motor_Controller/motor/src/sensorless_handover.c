// sensorless_handover.c

#include "sensorless_handover.h"

#include "speed_measurement.h"    // SpeedMeas_get, SpeedMeas_setMode, ...
#include "position_estimator.h"   // PosEst_setMode

void SensorlessHandover_init(SensorlessHandover_t *h,
                             float min_rpm_mech,
                             int   min_valid_samples)
{
    if (!h) return;

    h->enabled           = true;
    h->done              = false;

    h->min_rpm_mech      = min_rpm_mech;
    h->min_valid_samples = (min_valid_samples > 0) ? min_valid_samples : 1;
    h->valid_count       = 0;
}

void SensorlessHandover_setEnable(SensorlessHandover_t *h, bool enable)
{
    if (!h) return;
    h->enabled     = enable;
    h->done        = false;
    h->valid_count = 0;
}

bool SensorlessHandover_step(SensorlessHandover_t *h,
                             float now_s,
                             uint8_t current_sector,
                             bool direction_fwd)
{
    (void)now_s;  // not used right now, but kept for future tweaks

    if (!h || !h->enabled) {
        return false;
    }

    if (h->done) {
        // Already switched to BEMF
        return false;
    }

    // Make sure SpeedMeas is still in Hall mode; if not, do nothing.
    // (If you don't expose a getter for mode, you can remove this check.)
    // We'll just assume we are in Hall mode, and rely on the app
    // to only call this in that phase.

    SpeedEstimate_t est = SpeedMeas_get();

    // Need valid speed and sector from Hall path
    if (!est.valid || est.sector == 0xFF || est.sector >= 6) {
        h->valid_count = 0;
        return false;
    }

    if (est.rpm_mech >= h->min_rpm_mech) {
        h->valid_count++;
    } else {
        h->valid_count = 0;
    }

    if (h->valid_count < h->min_valid_samples) {
        // Not stable long enough yet
        return false;
    }

    // --- Conditions satisfied → perform handover ---

    // 1) Align BEMF sector tracker with current electrical sector
    BemfDir_t dir = direction_fwd ? BEMF_DIR_FWD : BEMF_DIR_REV;
    SpeedMeas_bemfAlign(current_sector, dir);

    // 2) Switch SpeedMeas to use BEMF as the source
    SpeedMeas_setMode(SPEED_SRC_BEMF);

    // 3) Switch position estimator mode
    PosEst_setMode(POS_MODE_BEMF);

    h->done = true;
    return true;
}