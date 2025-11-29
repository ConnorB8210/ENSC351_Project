#include <stdint.h>   // for uint8_t

// Map a 6-step sector + direction into (u,v,w) signs (+1, -1, 0)
static void sector_to_signs(uint8_t sector,
                            bool forward,
                            int *u, int *v, int *w)
{
    // Default: all phases off
    *u = 0; *v = 0; *w = 0;

    if (sector > 5) {
        return; // invalid, keep all 0
    }

    // Forward 6-step sequence (typical BLDC):
    // sector:  U   V   W
    //   0     +1  -1   0
    //   1     +1   0  -1
    //   2      0  +1  -1
    //   3     -1  +1   0
    //   4     -1   0  +1
    //   5      0  -1  +1
    if (forward) {
        switch (sector) {
        case 0: *u = +1; *v = -1; *w =  0; break;
        case 1: *u = +1; *v =  0; *w = -1; break;
        case 2: *u =  0; *v = +1; *w = -1; break;
        case 3: *u = -1; *v = +1; *w =  0; break;
        case 4: *u = -1; *v =  0; *w = +1; break;
        case 5: *u =  0; *v = -1; *w = +1; break;
        default: break;
        }
    } else {
        // Reverse sequence: invert sign pattern (one simple way)
        switch (sector) {
        case 0: *u = -1; *v = +1; *w =  0; break;
        case 1: *u = -1; *v =  0; *w = +1; break;
        case 2: *u =  0; *v = -1; *w = +1; break;
        case 3: *u = +1; *v = -1; *w =  0; break;
        case 4: *u = +1; *v =  0; *w = -1; break;
        case 5: *u =  0; *v = +1; *w = -1; break;
        default: break;
        }
    }
}

void PwmMotor_stop(PwmMotor_t *m)
{
    if (!m || !m->gpio) return;

    // Disable and force all phases off
    PwmMotor_setEnable(m, false);
}

void PwmMotor_setSixStep(PwmMotor_t *m,
                         uint8_t sector,
                         float duty,
                         bool forward)
{
    if (!m || !m->gpio) return;

    // Clamp duty
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    int u = 0, v = 0, w = 0;
    sector_to_signs(sector, forward, &u, &v, &w);

    // Enable gates only if we actually want to drive
    bool active = (duty > 0.0f);
    PwmMotor_setEnable(m, active);

    // Use existing phase API for actual drive
    PwmMotor_applyPhaseState(m, u, v, w, duty);
}