#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "pwm.h"
#include "motor_states.h"

typedef struct {
    uint8_t sector;     // 0..5
    bool direction;     // 0=fwd,1=rev
} HallSector_t;

// Convert hall bits (ABC) into sector index (0..5) or 0xFF if invalid
uint8_t HallComm_hallToSector(uint8_t hall_bits);

// Apply 6-step commutation pattern based on sector and desired duty
void HallComm_apply(float duty, bool direction, uint8_t sector);