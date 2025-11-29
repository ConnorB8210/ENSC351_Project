// hall_commutator.c
#include "hall_commutator.h"

/*
 * Standard 120° BLDC Hall decoding table
 *
 * Hall pattern (U V W)  →  Sector
 *   1 0 1   = 5   (0b101)
 *   1 0 0   = 4   (0b100)
 *   1 1 0   = 3   (0b110)
 *   0 1 0   = 2   (0b010)
 *   0 1 1   = 1   (0b011)
 *   0 0 1   = 0   (0b001)
 *
 * All other patterns invalid (0b000, 0b111, etc.)
 *
 * hall_bits encoding (must match Hall_readBits()):
 *   bit0 = Hall A (U)
 *   bit1 = Hall B (V)
 *   bit2 = Hall C (W)
 */

// 0..7 lookup: index = hall_bits & 0x7
//  000 -> invalid
//  001 -> sector 0
//  010 -> sector 2
//  011 -> sector 1
//  100 -> sector 4
//  101 -> sector 5
//  110 -> sector 3
//  111 -> invalid
static const uint8_t s_hall_to_sector[8] = {
    0xFF,  // 0b000
    0,     // 0b001
    2,     // 0b010
    1,     // 0b011
    4,     // 0b100
    5,     // 0b101
    3,     // 0b110
    0xFF   // 0b111
};

uint8_t HallComm_hallToSector(uint8_t hall_bits)
{
    return s_hall_to_sector[hall_bits & 0x7];
}

/*
 * Commutation table for U/V/W high/low drive
 *
 * Sector  U   V   W
 *   0     +   -   0
 *   1     +   0   -
 *   2     0   +   -
 *   3     -   +   0
 *   4     -   0   +
 *   5     0   -   +
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
        default:
            *u = 0;
            *v = 0;
            *w = 0;
            break;
    }
}