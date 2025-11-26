#include "drv8302.h"
#include <stdlib.h>
#include <stdio.h>
#include <gpiod.h>  // for direction/edge enums

bool Drv8302Hal_init(Drv8302Hal_t *drv,
                     const char *chip_path,
                     unsigned int en_gate_ofs,
                     unsigned int nfault_ofs,
                     unsigned int noctw_ofs)
{
    if (!drv || !chip_path) return false;

    // order: EN_GATE (output), nFAULT (input), nOCTW (input)
    unsigned int offsets[3] = {
        en_gate_ofs,
        nfault_ofs,
        noctw_ofs
    };

    // One request object for all three lines
    // Direction here is "mixed": we’ll configure via two requests to keep it simple,
    // or just do one request with them all as inputs and drive EN_GATE by value.
    // Easiest: treat all as outputs and only read the two fault pins.
    //
    // But better: two separate handles, one output, one input-group.
    // To keep it simple and reuse your GPIO_Handle, we’ll do ONE handle and:
    // - drive EN_GATE via gpio_write()
    // - read nFAULT/nOCTW via gpio_read()

    drv->gpio = gpio_init(chip_path,
                          offsets,
                          3,
                          GPIOD_LINE_DIRECTION_OUTPUT,   // we'll still read inputs fine
                          GPIOD_LINE_EDGE_NONE);
    if (!drv->gpio) {
        fprintf(stderr, "Drv8302Hal_init: gpio_init failed\n");
        return false;
    }

    drv->en_gate_idx = 0;
    drv->nfault_idx  = 1;
    drv->noctw_idx   = 2;

    // Default: gate disabled
    gpio_write(drv->gpio, drv->en_gate_idx, 0);

    return true;
}

void Drv8302Hal_setEnable(Drv8302Hal_t *drv, bool enable)
{
    if (!drv || !drv->gpio) return;
    gpio_write(drv->gpio, drv->en_gate_idx, enable ? 1 : 0);
}

int Drv8302Hal_read_nFault(Drv8302Hal_t *drv)
{
    if (!drv || !drv->gpio) return -1;
    return gpio_read(drv->gpio, drv->nfault_idx); // 0 = fault, 1 = OK
}

int Drv8302Hal_read_nOctw(Drv8302Hal_t *drv)
{
    if (!drv || !drv->gpio) return -1;
    return gpio_read(drv->gpio, drv->noctw_idx); // 0 = warning/OC/OT, 1 = OK
}

void Drv8302Hal_deinit(Drv8302Hal_t *drv)
{
    if (!drv) return;
    if (drv->gpio) {
        gpio_close(drv->gpio);
        drv->gpio = NULL;
    }
}