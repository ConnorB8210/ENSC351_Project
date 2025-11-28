// position_estimator.c

#include "position_estimator.h"
#include "speed_measurement.h"
#include "hall.h"   // for HallHandle_t in prototypes
#include "bemf.h"   // for BemfHandle_t in prototypes

#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static PosMode_t s_mode = POS_MODE_HALL;
static PosEst_t  s_est;

// We no longer need these inside this module, but keep the
// setters so existing code compiles.
void PosEst_setHallHandle(HallHandle_t *hh)
{
    (void)hh;
}

void PosEst_setBemfHandle(BemfHandle_t *bh)
{
    (void)bh;
}

void PosEst_init(PosMode_t mode)
{
    memset(&s_est, 0, sizeof(s_est));
    s_mode      = mode;
    s_est.valid = false;
}

void PosEst_setMode(PosMode_t mode)
{
    s_mode = mode;

    s_est.elec_angle = 0.0f;
    s_est.elec_speed = 0.0f;
    s_est.mech_speed = 0.0f;
    s_est.sector     = 0;
    s_est.valid      = false;
}

void PosEst_update(void)
{
    SpeedEstimate_t spd = SpeedMeas_get();

    // Always take speeds from SpeedMeas (regardless of source)
    s_est.mech_speed = spd.rpm_mech;
    s_est.elec_speed = spd.rpm_elec;

    if (!spd.valid || spd.sector == 0xFF || spd.sector >= 6) {
        s_est.sector     = 0;
        s_est.elec_angle = 0.0f;
        s_est.valid      = false;
        return;
    }

    uint8_t sector = spd.sector;
    s_est.sector   = sector;

    // For both HALL and BEMF modes we currently approximate angle
    // as the center of the 60-degree sector. Later you can refine
    // POS_MODE_BEMF to integrate electrical speed between ZCs.
    (void)s_mode;  // reserved for future behavior differences

    const float sectors_per_elec_rev = 6.0f;
    float angle_step = 2.0f * (float)M_PI / sectors_per_elec_rev;

    s_est.elec_angle = ((float)sector + 0.5f) * angle_step;
    s_est.valid      = true;
}

PosEst_t PosEst_get(void)
{
    return s_est;
}