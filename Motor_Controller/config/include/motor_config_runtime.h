// motor_config_runtime.h
#pragma once

#include <stdbool.h>
#include "motor_config.h"  // for the compile-time macros

// Central runtime config object. Initialized from motor_config.h
// macros, then optionally overridden from a config file.
typedef struct
{
    // Motor characteristics
    float pole_pairs;
    float kv_rpm_per_v;
    float r_phase_ohm;
    float l_phase_h;

    // Limits
    float i_max_a;
    float bus_v_max_v;
    float bus_v_min_v;
    float rpm_max;

    // Loop timing
    float fast_loop_hz;
    float slow_loop_hz;
    float pwm_freq_hz;

    // Sensorless / handover
    float sensorless_min_rpm_mech;
    int   sensorless_stable_samples;
} MotorRuntimeConfig;

// Global instance (defined in motor_config_runtime.c)
extern MotorRuntimeConfig g_motor_cfg;

/**
 * @brief Initialize runtime config from compile-time macros.
 *
 * Call once at startup before using any of the helper functions.
 */
void MotorConfig_initDefaults(void);

/**
 * @brief Load overrides from a simple key=value text file.
 *
 * Example file:
 *   MOTOR_KV_RPM_PER_V=900.0
 *   MOTOR_RPM_MAX=4000
 *   FAST_LOOP_HZ=15000
 *
 * Unknown keys are ignored; parse errors are non-fatal.
 *
 * @return 0 on success, -1 if the file could not be opened.
 */
int MotorConfig_loadFromFile(const char *path);

/**
 * @brief Perform sanity checks on the runtime config.
 *
 * Prints warnings to stderr if values look suspicious.
 * @return true if config looks OK, false if there are serious issues.
 */
bool MotorConfig_sanityCheck(void);

// ---------- Convenience conversion helpers ----------

/**
 * @brief Convert mechanical RPM to electrical RPM.
 */
static inline float mech_rpm_to_elec_rpm(float rpm_mech)
{
    return rpm_mech * g_motor_cfg.pole_pairs;
}

/**
 * @brief Convert electrical RPM to mechanical RPM.
 */
static inline float elec_rpm_to_mech_rpm(float rpm_elec)
{
    if (g_motor_cfg.pole_pairs <= 0.0f) return 0.0f;
    return rpm_elec / g_motor_cfg.pole_pairs;
}

/**
 * @brief Convert mechanical RPM to electrical frequency in Hz.
 */
static inline float mech_rpm_to_elec_hz(float rpm_mech)
{
    float rpm_elec = mech_rpm_to_elec_rpm(rpm_mech);
    return rpm_elec / 60.0f;
}

/**
 * @brief Convert electrical Hz to mechanical RPM.
 */
static inline float elec_hz_to_mech_rpm(float hz_elec)
{
    float rpm_elec = hz_elec * 60.0f;
    return elec_rpm_to_mech_rpm(rpm_elec);
}