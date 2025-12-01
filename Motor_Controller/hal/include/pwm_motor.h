// include/hal/pwm_motor.h
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char period_path[128];
    char duty_path[128];
    char enable_path[128];
} PwmMotorChannel_t;

typedef struct {
    PwmMotorChannel_t ch[6];  // 0:INH-A,1:INL-A,2:INH-B,3:INL-B,4:INH-C,5:INL-C
    double            freq_hz;
    unsigned long long period_ns;
    bool              enabled;
} PwmMotor_t;

/**
 * pwm_root: usually "/dev/hat/pwm"
 * the offsets are GPIO numbers that have PWM devices:
 *   -> "/dev/hat/pwm/GPIO<offset>/period", etc.
 */
bool PwmMotor_init(PwmMotor_t *m,
                   const char *pwm_root,
                   unsigned int inh_a_gpio,
                   unsigned int inl_a_gpio,
                   unsigned int inh_b_gpio,
                   unsigned int inl_b_gpio,
                   unsigned int inh_c_gpio,
                   unsigned int inl_c_gpio);

void PwmMotor_setEnable(PwmMotor_t *m, bool enable);
void PwmMotor_applyPhaseState(PwmMotor_t *m,
                              int u, int v, int w,
                              float duty);
void PwmMotor_setSixStep(PwmMotor_t *m,
                         uint8_t sector,
                         float duty,
                         bool forward);
void PwmMotor_stop(PwmMotor_t *m);
void PwmMotor_deinit(PwmMotor_t *m);

#ifdef __cplusplus
}
#endif