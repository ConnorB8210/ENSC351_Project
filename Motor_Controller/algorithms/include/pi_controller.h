#pragma once
#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float out_min;
    float out_max;

    float integral;
    float Ts;        // sample time [s]
} PIController_t;

void PI_init(PIController_t *pi, float kp, float ki,
             float out_min, float out_max, float Ts);

float PI_step(PIController_t *pi, float ref, float meas, bool enable_antiwindup);
void  PI_reset(PIController_t *pi);