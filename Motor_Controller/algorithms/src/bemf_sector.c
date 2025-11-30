#include "bemf_sector.h"
#include "motor_config.h"   // MOTOR_POLE_PAIRS, BEMF_VALID_MIN_V
#include <string.h>
#include <math.h>

// One electrical revolution = 6 commutation sectors
#define BEMF_SECTORS_PER_ELEC_REV  6.0f
// Minimum acceptable time between ZCs (guard against dt ~0)
#define BEMF_MIN_PERIOD_S          1e-5f
// Simple noise threshold for zero-cross detection on floating phase
#define BEMF_ZERO_THRESH_V         0.05f   // tweak based on real noise

// Small helper
static inline float fabsf_local(float x) { return (x >= 0.0f) ? x : -x; }

// Map sector -> floating phase index for your commutation table:
// From hall_commutator.c:
//
// Sector  U   V   W   Floating phase
//   0     +   -   0   W (2)
//   1     +   0   -   V (1)
//   2     0   +   -   U (0)
//   3     -   +   0   W (2)
//   4     -   0   +   V (1)
//   5     0   -   +   U (0)
//
static uint8_t floating_phase_for_sector(uint8_t sector)
{
    static const uint8_t lut[6] = { 2, 1, 0, 2, 1, 0 };
    sector %= 6;
    return lut[sector];
}

static uint8_t next_sector(uint8_t sec, BemfDir_t dir)
{
    sec %= 6;
    if (dir == BEMF_DIR_FWD) {
        return (uint8_t)((sec + 1U) % 6U);
    } else {
        return (uint8_t)(sec == 0U ? 5U : (sec - 1U));
    }
}

void BemfSector_init(BemfSectorState_t *s,
                     uint8_t start_sector,
                     BemfDir_t dir)
{
    if (!s) return;
    memset(s, 0, sizeof(*s));
    s->sector       = (uint8_t)(start_sector % 6U);
    s->dir          = dir;
    s->last_zero_ts = 0.0f;
    s->last_period_s= 0.0f;
    s->rpm_elec     = 0.0f;
    s->rpm_mech     = 0.0f;
    s->zero_valid   = false;
    s->valid        = false;
}

void BemfSector_update(BemfSectorState_t *s,
                       const BemfHandle_t *bemf,
                       float now_s)
{
    if (!s || !bemf) return;

    // Require a sensible bus voltage; otherwise BEMF is meaningless.
    float vbus = Bemf_getVbus(bemf);
    if (vbus < BEMF_VALID_MIN_V) {
        s->valid      = false;
        s->rpm_elec   = 0.0f;
        s->rpm_mech   = 0.0f;
        s->last_period_s = 0.0f;
        return;
    }

    // Determine which phase is currently floating given the sector.
    uint8_t float_phase = floating_phase_for_sector(s->sector);

    // Neutral‑referenced BEMF of floating phase:
    float v_phase_neutral = Bemf_getNeutralDiff(bemf, float_phase);

    // Basic zero‑cross detection with simple hysteresis:
    // We track sign transitions from "negative region" to "positive region"
    // to mark the sector crossing.
    typedef enum {
        ZC_REGION_NEG = -1,
        ZC_REGION_ZERO = 0,
        ZC_REGION_POS = +1
    } ZcRegion_t;

    static float prev_v = 0.0f;
    static ZcRegion_t prev_region = ZC_REGION_ZERO;

    ZcRegion_t region;
    if (v_phase_neutral >  BEMF_ZERO_THRESH_V)      region = ZC_REGION_POS;
    else if (v_phase_neutral < -BEMF_ZERO_THRESH_V) region = ZC_REGION_NEG;
    else                                            region = ZC_REGION_ZERO;

    bool crossed = false;
    // Detect NEG -> POS crossing (optionally allow NEG->ZERO->POS)
    if ((prev_region == ZC_REGION_NEG && region == ZC_REGION_POS) ||
        (prev_region == ZC_REGION_NEG && region == ZC_REGION_ZERO)) {
        crossed = true;
    }

    prev_v      = v_phase_neutral;
    prev_region = region;

    if (!crossed) {
        // No zero-cross this update; nothing else to do.
        return;
    }

    // --- We detected a zero-crossing on the floating phase ---
    float dt = now_s - s->last_zero_ts;
    if (dt < BEMF_MIN_PERIOD_S) {
        // Ignore unrealistically small intervals
        return;
    }

    s->last_zero_ts  = now_s;
    s->last_period_s = dt;

    // For a standard 6‑step scheme, we get one zero‑cross per 60 el. degrees.
    // That means an electrical period T_elec = dt * 6.
    float T_elec = dt * BEMF_SECTORS_PER_ELEC_REV;
    if (T_elec < BEMF_MIN_PERIOD_S) {
        s->valid    = false;
        s->rpm_elec = 0.0f;
        s->rpm_mech = 0.0f;
    } else {
        float f_elec   = 1.0f / T_elec;        // Hz
        float rpm_elec = f_elec * 60.0f;       // rpm
        s->rpm_elec    = rpm_elec;
        s->rpm_mech    = rpm_elec / (float)MOTOR_POLE_PAIRS;
        s->valid       = true;
    }

    s->zero_valid = true;

    // Advance sector according to direction
    s->sector = next_sector(s->sector, s->dir);
}

BemfSectorState_t BemfSector_get(const BemfSectorState_t *s)
{
    if (!s) {
        BemfSectorState_t zero;
        memset(&zero, 0, sizeof(zero));
        return zero;
    }
    return *s;
}