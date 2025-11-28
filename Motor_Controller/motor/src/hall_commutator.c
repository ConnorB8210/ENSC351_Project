#include "hall_commutator.h"

/*
 * Standard 120° BLDC Hall decoding table
 *
 * Hall pattern (U V W)  →  Sector
 *   1 0 1   = 5
 *   1 0 0   = 4
 *   1 1 0   = 3
 *   0 1 0   = 2
 *   0 1 1   = 1
 *   0 0 1   = 0
 *
 * All other patterns invalid (0x0, 0x7, etc.)
 */

uint8_t HallComm_hallToSector(uint8_t hall_bits)
{
    // Bits should be 0–7: (HALL_U << 2) | (HALL_V << 1) | (HALL_W)
    switch (hall_bits & 0x7) {
        case 0b101: return 5;
        case 0b100: return 4;
        case 0b110: return 3;
        case 0b010: return 2;
        case 0b011: return 1;
        case 0b001: return 0;
        default:    return 0xFF; // invalid pattern
    }
}

/*
 * Commutation table for U/V/W high/low drive
 *
 * Sector  U   V   W
 *   0     +   -   0
 *   1     +    0  -
 *   2      0   +  -
 *   3     -   +   0
 *   4     -    0  +
 *   5      0  -   +
 *
 * Output:
 *   +1 = high‑side active
 *   -1 = low‑side active
 *    0 = phase floating
 */

void HallComm_getPhaseState(uint8_t sector, int *u, int *v, int *w)
{
    if (!u || !v || !w) return;

    switch (sector) {
        case 0: *u = +1; *v = -1; *w =  0; break;
        case 1: *u = +1; *v =  0; *w = -1; break;
        case 2: *u =  0; *v = +1; *w = -1; break;
        case 3: *u = -1; *v = +1; *w =  0; break;
        case 4: *u = -1; *v =  0; *w = +1; break;
        case 5: *u =  0; *v = -1; *w = +1; break;

        default: // invalid sector
            *u = 0;
            *v = 0;
            *w = 0;
            break;
    }
}