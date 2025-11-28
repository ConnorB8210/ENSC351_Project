// position_estimator.h
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "hall.h"
#include "bemf.h"

typedef enum {
    POS_MODE_HALL = 0,
    POS_MODE_BEMF
} PosMode_t;

typedef struct {
    float    elec_angle;   // electrical angle [rad]
    float    elec_speed;   // electrical speed [rad/s] or RPM equiv
    float    mech_speed;   // mechanical speed [RPM]
    uint8_t  sector;       // 0..5 for 6-step
    bool     valid;
} PosEst_t;
void PosEst_setBemfHandle(BemfHandle_t *bh);
void PosEst_init(PosMode_t mode);
void PosEst_setMode(PosMode_t mode);
void PosEst_update(void);
PosEst_t PosEst_get(void);