// bemf_sector.c

#include "bemf_sector.h"
#include "motor_config.h"   // MOTOR_POLE_PAIRS

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// One BEMF zero-cross (on the floating phase) per sector.
// 6 sectors per electrical revolution.
#define SECTORS_PER_ELEC_REV   6.0f

// Thresholds / limits
#define BEMF_ZC_THRESHOLD_V    0.2f    // neutral diff must exceed this before we trust sign
#define BEMF_MIN_PERIOD_S      1e-5f
#define BEMF_STANDSTILL_TIMEOUT_S  0.5f

// Map sector -> floating phase index (0=U,1=V,2=W)
// This must match your commutation table used in HallComm_getPhaseState().
//
// From earlier mapping:
//   Sector 0: U+, V-, W floating -> W (2)
//   Sector 1: U+, W-, V floating -> V (1)
//   Sector 2: V+, W-, U floating -> U (0)
//   Sector 3: V+, U-, W floating -> W (2)
//   Sector 4: W+, U-, V floating -> V (1)
//   Sector 5: W+, V-, U floating -> U (0)
static uint8_t floating_phase_for_sector(uint8_t sector)
{
    switch (sector % 6) {
        case 0: return 2; // W
        case 1: return 1; // V
        case 2: return 0; // U
        case 3: return 2; // W
        case 4: return 1; // V
        case 5: return 0; // U
        default: return 0; // should never happen
    }
}

// Helper to normalize sector index into 0..5
static inline uint8_t norm_sector(int s)
{
    int r = s % 6;
    if (r < 0) r += 6;
    return (uint8_t)r;
}

void BemfSector_init(BemfSectorState_t *s,
                     uint8_t start_sector,
                     BemfDir_t dir)
{
    if (!s) return;

    memset(s, 0, sizeof(*s));

    s->sector         = norm_sector(start_sector);
    s->rpm_elec       = 0.0f;
    s->rpm_mech       = 0.0f;
    s->last_period_s  = 0.0f;
    s->valid          = false;
    s->last_zc_time   = 0.0f;
    s->prev_zc_time   = 0.0f;
    s->last_sample_time = 0.0f;
    s->last_diff      = 0.0f;
    s->last_sign      = 0;
    s->dir            = dir;
}

void BemfSector_setDirection(BemfSectorState_t *s, BemfDir_t dir)
{
    if (!s) return;
    s->dir = dir;
}

void BemfSector_setSector(BemfSectorState_t *s, uint8_t sector)
{
    if (!s) return;
    s->sector = norm_sector(sector);
    // Reset phase of ZC detection so we don't misinterpret old state
    s->last_sign    = 0;
    s->last_diff    = 0.0f;
    s->last_zc_time = 0.0f;
    s->prev_zc_time = 0.0f;
    s->valid        = false;
}

void BemfSector_update(BemfSectorState_t *s,
                       const BemfHandle_t *bemf,
                       float now_s)
{
    if (!s || !bemf) return;

    // Standstill / timeout: if we haven't seen a ZC for a long time,
    // decay the validity of speed estimate.
    if (s->last_zc_time > 0.0f &&
        (now_s - s->last_zc_time) > BEMF_STANDSTILL_TIMEOUT_S) {
        s->rpm_elec      = 0.0f;
        s->rpm_mech      = 0.0f;
        s->last_period_s = 0.0f;
        s->valid         = false;
    }

    // Pick the floating phase for the *current* sector.
    uint8_t float_phase = floating_phase_for_sector(s->sector);

    // Neutral referenced diff (phase - Vbus/2)
    float diff = Bemf_getNeutralDiff(bemf, float_phase);

    // Determine sign, but only if magnitude is above threshold
    int sign = 0;
    if (diff >  BEMF_ZC_THRESHOLD_V) sign = +1;
    else if (diff < -BEMF_ZC_THRESHOLD_V) sign = -1;
    else sign = 0; // in deadband around zero

    // Check for sign change → zero-cross
    if (s->last_sign != 0 && sign != 0 && sign != s->last_sign) {
        // Approximate zero-cross time by linear interpolation between last
        // and current sample: t_zc ~= t_prev + (dt * |last_diff| /
        // (|last_diff| + |diff|)). For simplicity, we use the midpoint.
        float t_prev = s->last_sample_time;
        float t_now  = now_s;

        float t_zc = 0.5f * (t_prev + t_now);

        // Compute period between zero-crosses of floating phase (center of sectors)
        if (s->last_zc_time > 0.0f) {
            float dt_zc = t_zc - s->last_zc_time;
            if (dt_zc > BEMF_MIN_PERIOD_S) {
                // One zero-cross per sector → 6 sectors per elec rev
                float T_elec = dt_zc * SECTORS_PER_ELEC_REV;
                float f_elec = 1.0f / T_elec;        // Hz
                float rpm_e  = f_elec * 60.0f;       // electrical RPM

                s->rpm_elec      = rpm_e;
                s->rpm_mech      = rpm_e / (float)MOTOR_POLE_PAIRS;
                s->last_period_s = T_elec;
                s->valid         = true;
            }
        }

        // Update ZC times
        s->prev_zc_time = s->last_zc_time;
        s->last_zc_time = t_zc;

        // Advance sector based on direction
        int next = (int)s->sector + (int)s->dir;
        s->sector = norm_sector(next);
    }

    // Update stored values for next call
    s->last_diff       = diff;
    s->last_sign       = sign;
    s->last_sample_time = now_s;
}

BemfSectorState_t BemfSector_get(const BemfSectorState_t *s)
{
    if (!s) {
        BemfSectorState_t dummy;
        memset(&dummy, 0, sizeof(dummy));
        dummy.sector = 0xFF;
        dummy.valid  = false;
        return dummy;
    }
    return *s;
}