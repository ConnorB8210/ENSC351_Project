#include "speed_measurement.h"
#include "motor_config.h"

#include <string.h>   // memset

// Number of hall sectors per electrical revolution in 6‑step BLDC
#define SECTORS_PER_ELEC_REV  6.0f

// Optional: ignore absurdly small periods (noise / glitches)
#define MIN_PERIOD_S          1e-5f

// Internal state
static SpeedEstimate_t s_est;
static float           s_last_edge_ts = 0.0f;
static int             s_have_edge    = 0;

// ---------------------------------------------------------
// Initialization
// ---------------------------------------------------------
void SpeedMeas_init(void)
{
    memset(&s_est, 0, sizeof(s_est));
    s_est.valid         = false;
    s_last_edge_ts      = 0.0f;
    s_have_edge         = 0;
}

// ---------------------------------------------------------
// Hall edge callback
// ---------------------------------------------------------
//
// timestamp_s: current time in seconds (monotonic)
// sector:      0..5 (not used yet but kept for future extensions)
//
// This function computes the time between the last hall edge and
// this one, then converts that into electrical RPM and mechanical RPM.
//
// One electrical revolution spans SECTORS_PER_ELEC_REV hall sectors.
// So: T_elec = period_sector * SECTORS_PER_ELEC_REV
//      f_elec = 1 / T_elec
//      rpm_elec = f_elec * 60
//      rpm_mech = rpm_elec / MOTOR_POLE_PAIRS
//
void SpeedMeas_onHallEdge(float timestamp_s, uint8_t sector)
{
    (void)sector;  // currently unused, but kept for future (e.g. direction check)

    if (!s_have_edge) {
        // First edge after init/reset: just latch timestamp
        s_last_edge_ts = timestamp_s;
        s_have_edge    = 1;
        s_est.valid    = false;
        return;
    }

    float dt = timestamp_s - s_last_edge_ts;
    if (dt <= MIN_PERIOD_S) {
        // Ignore bogus / repeated timestamps
        return;
    }

    s_last_edge_ts     = timestamp_s;
    s_est.last_period_s = dt;

    // Electrical period for one full revolution
    float T_elec = dt * SECTORS_PER_ELEC_REV;
    if (T_elec <= MIN_PERIOD_S) {
        s_est.valid = false;
        return;
    }

    float f_elec   = 1.0f / T_elec;         // Hz
    float rpm_elec = f_elec * 60.0f;

    s_est.rpm_elec = rpm_elec;
    s_est.rpm_mech = rpm_elec / (float)MOTOR_POLE_PAIRS;
    s_est.valid    = true;
}

// ---------------------------------------------------------
// Getter
// ---------------------------------------------------------
SpeedEstimate_t SpeedMeas_get(void)
{
    return s_est;
}