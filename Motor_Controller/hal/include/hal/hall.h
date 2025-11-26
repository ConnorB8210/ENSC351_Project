#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "gpio.h"   // your existing module

typedef enum {
    HALL_A = 0,
    HALL_B = 1,
    HALL_C = 2
} HallChannel_t;

typedef struct {
    GPIO_Handle *gpio;  // underlying generic GPIO handle
} HallHandle_t;

// Initialize 3 hall inputs on a given chip (e.g. "/dev/gpiochip0")
// offsets are the *line numbers* for A, B, C
bool Hall_init(HallHandle_t *hh,
               const char *chip_path,
               unsigned int hall_a_offset,
               unsigned int hall_b_offset,
               unsigned int hall_c_offset);

// Read the three hall lines and return them as bits: b0=A, b1=B, b2=C
// e.g. ABC = 101b = 0b101 = 5
uint8_t Hall_readBits(HallHandle_t *hh);

// Optional convenience: read individual channel (0 or 1)
int Hall_readChannel(HallHandle_t *hh, HallChannel_t ch);

// Cleanup
void Hall_close(HallHandle_t *hh);