#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float rpm_mech;
    float rpm_elec;
    float last_period_s;
    bool  valid;
} SpeedEstimate_t;

void SpeedMeas_init(void);

// Call from hall edge ISR / callback with timestamp in seconds and sector index
void SpeedMeas_onHallEdge(float timestamp_s, uint8_t sector);

// Call from control loop to get latest estimate (already filtered)
SpeedEstimate_t SpeedMeas_get(void);