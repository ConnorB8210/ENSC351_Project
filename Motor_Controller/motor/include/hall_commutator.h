// hall_commutator.h
#pragma once

#include <stdint.h>

/**
 * @brief Convert raw Hall bits (A,B,C) into a 6-step sector index.
 *
 * hall_bits encoding must match Hall_readBits():
 *   bit0 = Hall A (U)
 *   bit1 = Hall B (V)
 *   bit2 = Hall C (W)
 *
 * Returns:
 *   0..5   = valid 6-step sector
 *   0xFF   = invalid pattern (0b000, 0b111, or any wiring glitch)
 */
uint8_t HallComm_hallToSector(uint8_t hall_bits);

/**
 * @brief Map 6-step sector into per-phase signs for U/V/W.
 *
 * Sector mapping (forward):
 *   Sector  U   V   W
 *     0     +   -   0
 *     1     +   0   -
 *     2     0   +   -
 *     3     -   +   0
 *     4     -   0   +
 *     5     0   -   +
 *
 * Output:
 *   +1 = high-side active
 *   -1 = low-side active
 *    0 = phase floating
 */
void HallComm_getPhaseState(uint8_t sector, int *u, int *v, int *w);