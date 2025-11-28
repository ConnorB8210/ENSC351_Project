// speed_measurement.c

#include "speed_measurement.h"
#include "motor_config.h"
#include "hall_commutator.h"

#include <string.h>   // memset

#define SECTORS_PER_ELEC_REV        6.0f
#define MIN_PERIOD_S                1e-5f
#define STANDSTILL_TIMEOUT_S        0.5f    // after 0.5 s without edge -> invalid

static SpeedEstimate_t  s_est;
static SpeedSource_t    s_mode        = SPEED_SRC_HALL;

static HallHandle_t    *s_hall        = NULL;

// BEMF sensorless backend
static BemfHandle_t    *s_bemf        = NULL;
static BemfSectorState_t s_bemf_state;

// Hall-only internal state
static uint8_t  s_last_sector  = 0xFF;
static float    s_last_edge_ts = 0.0f;
static int      s_have_edge    = 0;

void SpeedMeas_init(void)
{
    memset(&s_est, 0, sizeof(s_est));
    s_est.valid   = false;
    s_est.sector  = 0xFF;

    s_mode        = SPEED_SRC_HALL;
    s_hall        = NULL;

    s_bemf        = NULL;
    BemfSector_init(&s_bemf_state, 0, BEMF_DIR_FWD);

    s_last_sector  = 0xFF;
    s_last_edge_ts = 0.0f;
    s_have_edge    = 0;
}

void SpeedMeas_setMode(SpeedSource_t src)
{
    s_mode = src;
    // Reset estimates when switching source
    s_est.rpm_mech      = 0.0f;
    s_est.rpm_elec      = 0.0f;
    s_est.last_period_s = 0.0f;
    s_est.sector        = 0xFF;
    s_est.valid         = false;

    // Reset hall-side timing
    s_last_sector  = 0xFF;
    s_last_edge_ts = 0.0f;
    s_have_edge    = 0;

    // Reset BEMF state (sector will be re-aligned with SpeedMeas_bemfAlign)
    BemfSector_init(&s_bemf_state, 0, BEMF_DIR_FWD);
}

void SpeedMeas_setHallHandle(HallHandle_t *hh)
{
    s_hall = hh;
}

void SpeedMeas_setBemfHandle(BemfHandle_t *bh)
{
    s_bemf = bh;
}

void SpeedMeas_bemfAlign(uint8_t start_sector, BemfDir_t dir)
{
    BemfSector_init(&s_bemf_state, start_sector, dir);
}

static void update_hall(float now_s)
{
    if (!s_hall) {
        s_est.valid  = false;
        s_est.sector = 0xFF;
        return;
    }

    // 1) Read hall bits and convert to sector
    uint8_t hall_bits = Hall_readBits(s_hall);
    uint8_t sector    = HallComm_hallToSector(hall_bits);

    if (sector == 0xFF) {
        // Invalid hall pattern -> invalidate
        s_est.valid  = false;
        s_est.sector = 0xFF;
        return;
    }

    // 2) Standstill / timeout check
    if (s_have_edge && (now_s - s_last_edge_ts) > STANDSTILL_TIMEOUT_S) {
        s_est.rpm_mech      = 0.0f;
        s_est.rpm_elec      = 0.0f;
        s_est.last_period_s = 0.0f;
        s_est.valid         = false;
        // keep sector as-is
    }

    if (!s_have_edge) {
        // First valid sector
        s_last_sector   = sector;
        s_last_edge_ts  = now_s;
        s_have_edge     = 1;
        s_est.valid     = false;
        s_est.sector    = sector;
        return;
    }

    // 3) On sector change -> edge
    if (sector != s_last_sector) {
        float dt = now_s - s_last_edge_ts;
        if (dt > MIN_PERIOD_S) {
            s_last_edge_ts      = now_s;
            s_last_sector       = sector;
            s_est.last_period_s = dt;
            s_est.sector        = sector;

            float T_elec = dt * SECTORS_PER_ELEC_REV;
            if (T_elec > MIN_PERIOD_S) {
                float f_elec   = 1.0f / T_elec;
                float rpm_elec = f_elec * 60.0f;

                s_est.rpm_elec = rpm_elec;
                s_est.rpm_mech = rpm_elec / (float)MOTOR_POLE_PAIRS;
                s_est.valid    = true;
            } else {
                s_est.valid = false;
            }
        }
    } else {
        s_est.sector = sector;
    }
}

static void update_bemf(float now_s)
{
    if (!s_bemf) {
        s_est.valid  = false;
        s_est.sector = 0xFF;
        return;
    }

    // Bemf_update() should already have been called before this in the loop.
    BemfSector_update(&s_bemf_state, s_bemf, now_s);

    BemfSectorState_t bs = BemfSector_get(&s_bemf_state);

    s_est.rpm_elec      = bs.rpm_elec;
    s_est.rpm_mech      = bs.rpm_mech;
    s_est.last_period_s = bs.last_period_s;
    s_est.sector        = bs.valid ? bs.sector : 0xFF;
    s_est.valid         = bs.valid;
}

void SpeedMeas_update(float now_s)
{
    switch (s_mode) {
        case SPEED_SRC_BEMF:
            update_bemf(now_s);
            break;
        case SPEED_SRC_HALL:
        default:
            update_hall(now_s);
            break;
    }
}

SpeedEstimate_t SpeedMeas_get(void)
{
    return s_est;
}