#include "hall.h"
#include <stdlib.h>
#include <stdio.h>
#include <gpiod.h>   // for enum gpiod_line_direction / edge

bool Hall_init(HallHandle_t *hh,
               const char *chip_path,
               unsigned int hall_a_offset,
               unsigned int hall_b_offset,
               unsigned int hall_c_offset)
{
    if (!hh || !chip_path) {
        return false;
    }

    unsigned int offsets[3] = {
        hall_a_offset,
        hall_b_offset,
        hall_c_offset
    };

    // Use your generic gpio_init:
    // - direction: input
    // - edge: BOTH (so you *can* do edge-based stuff later if you want)
    hh->gpio = gpio_init(chip_path,
                         offsets,
                         3,
                         GPIOD_LINE_DIRECTION_INPUT,
                         GPIOD_LINE_EDGE_BOTH);
    if (!hh->gpio) {
        fprintf(stderr, "Hall_init: failed to init GPIO lines %u,%u,%u on %s\n",
                hall_a_offset, hall_b_offset, hall_c_offset, chip_path);
        return false;
    }

    return true;
}

uint8_t Hall_readBits(HallHandle_t *hh)
{
    if (!hh || !hh->gpio) {
        return 0;
    }

    int vals[3] = {0};
    if (gpio_read_all(hh->gpio, vals) < 0) {
        // On error, just return previous state or 0; here we choose 0
        return 0;
    }

    // vals[] are libgpiod values (0 or 1). Pack as bits:
    // bit0 = Hall A, bit1 = Hall B, bit2 = Hall C
    uint8_t bits = 0;
    bits |= (vals[0] ? 1 : 0) << 0;  // A
    bits |= (vals[1] ? 1 : 0) << 1;  // B
    bits |= (vals[2] ? 1 : 0) << 2;  // C

    return bits;
}

int Hall_readChannel(HallHandle_t *hh, HallChannel_t ch)
{
    if (!hh || !hh->gpio) {
        return -1;
    }

    if (ch < 0 || ch > HALL_C) {
        return -1;
    }

    return gpio_read(hh->gpio, (unsigned int)ch);  // 0 or 1 (or -1 on error)
}

void Hall_close(HallHandle_t *hh)
{
    if (!hh) return;

    if (hh->gpio) {
        gpio_close(hh->gpio);
        hh->gpio = NULL;
    }
}