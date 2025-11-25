#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DRV8302_OK = 0,
    DRV8302_FAULT_OT,
    DRV8302_FAULT_OC,
    DRV8302_FAULT_UVLO,
    DRV8302_FAULT_UNKNOWN
} Drv8302Fault_t;

bool Drv8302_init(void);
bool Drv8302_clearFaults(void);
Drv8302Fault_t Drv8302_readFault(void);

// Optional: set current amp gain, etc., via SPI if board exposes it
bool Drv8302_configureDefaults(void);