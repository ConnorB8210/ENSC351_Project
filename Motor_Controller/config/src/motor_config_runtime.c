// motor_config_runtime.c

#include "motor_config_runtime.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

// Global instance
MotorRuntimeConfig g_motor_cfg;

// Trim helpers for parsing text files
static char *trim_leading(char *s)
{
    while (*s && isspace((unsigned char)*s)) s++;
    return s;
}

static void trim_trailing(char *s)
{
    char *end = s + strlen(s);
    while (end > s && isspace((unsigned char)*(end - 1))) {
        --end;
    }
    *end = '\0';
}

void MotorConfig_initDefaults(void)
{
    g_motor_cfg.pole_pairs   = (float)MOTOR_POLE_PAIRS;
    g_motor_cfg.kv_rpm_per_v = MOTOR_KV_RPM_PER_V;
    g_motor_cfg.r_phase_ohm  = MOTOR_R_PHASE_OHM;
    g_motor_cfg.l_phase_h    = MOTOR_L_PHASE_H;

    g_motor_cfg.i_max_a      = MOTOR_I_MAX_A;
    g_motor_cfg.bus_v_max_v  = MOTOR_BUS_V_MAX_V;
    g_motor_cfg.bus_v_min_v  = MOTOR_BUS_V_MIN_V;
    g_motor_cfg.rpm_max      = MOTOR_RPM_MAX;

    g_motor_cfg.fast_loop_hz = (float)FAST_LOOP_HZ;
    g_motor_cfg.slow_loop_hz = (float)SLOW_LOOP_HZ;
    g_motor_cfg.pwm_freq_hz  = (float)PWM_FREQUENCY_HZ;

    g_motor_cfg.sensorless_min_rpm_mech   = SENSORLESS_MIN_RPM_MECH;
    g_motor_cfg.sensorless_stable_samples = SENSORLESS_STABLE_SAMPLES;
}

static void apply_key_value(const char *key, const char *val_str)
{
    float fval = (float)atof(val_str);
    long  lval = strtol(val_str, NULL, 10);

    if (strcmp(key, "MOTOR_POLE_PAIRS") == 0) {
        if (fval > 0.0f) g_motor_cfg.pole_pairs = fval;
    } else if (strcmp(key, "MOTOR_KV_RPM_PER_V") == 0) {
        if (fval > 0.0f) g_motor_cfg.kv_rpm_per_v = fval;
    } else if (strcmp(key, "MOTOR_R_PHASE_OHM") == 0) {
        if (fval > 0.0f) g_motor_cfg.r_phase_ohm = fval;
    } else if (strcmp(key, "MOTOR_L_PHASE_H") == 0) {
        if (fval > 0.0f) g_motor_cfg.l_phase_h = fval;
    } else if (strcmp(key, "MOTOR_I_MAX_A") == 0) {
        if (fval > 0.0f) g_motor_cfg.i_max_a = fval;
    } else if (strcmp(key, "MOTOR_BUS_V_MAX_V") == 0) {
        if (fval > 0.0f) g_motor_cfg.bus_v_max_v = fval;
    } else if (strcmp(key, "MOTOR_BUS_V_MIN_V") == 0) {
        if (fval > 0.0f) g_motor_cfg.bus_v_min_v = fval;
    } else if (strcmp(key, "MOTOR_RPM_MAX") == 0) {
        if (fval > 0.0f) g_motor_cfg.rpm_max = fval;
    } else if (strcmp(key, "FAST_LOOP_HZ") == 0) {
        if (fval > 0.0f) g_motor_cfg.fast_loop_hz = fval;
    } else if (strcmp(key, "SLOW_LOOP_HZ") == 0) {
        if (fval > 0.0f) g_motor_cfg.slow_loop_hz = fval;
    } else if (strcmp(key, "PWM_FREQUENCY_HZ") == 0) {
        if (fval > 0.0f) g_motor_cfg.pwm_freq_hz = fval;
    } else if (strcmp(key, "SENSORLESS_MIN_RPM_MECH") == 0) {
        if (fval > 0.0f) g_motor_cfg.sensorless_min_rpm_mech = fval;
    } else if (strcmp(key, "SENSORLESS_STABLE_SAMPLES") == 0) {
        if (lval > 0) g_motor_cfg.sensorless_stable_samples = (int)lval;
    } else {
        // Unknown key: ignore
    }
}

int MotorConfig_loadFromFile(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        perror("MotorConfig_loadFromFile: fopen");
        return -1;
    }

    char line[256];
    int line_no = 0;

    while (fgets(line, sizeof(line), f)) {
        line_no++;
        char *p = trim_leading(line);
        trim_trailing(p);

        if (*p == '\0' || *p == '#') {
            continue; // blank or comment
        }

        char *eq = strchr(p, '=');
        if (!eq) {
            fprintf(stderr,
                    "MotorConfig_loadFromFile: line %d missing '=': %s\n",
                    line_no, p);
            continue;
        }

        *eq = '\0';
        char *key = trim_leading(p);
        trim_trailing(key);

        char *val = trim_leading(eq + 1);
        trim_trailing(val);

        if (*key == '\0' || *val == '\0') {
            fprintf(stderr,
                    "MotorConfig_loadFromFile: line %d invalid key/value\n",
                    line_no);
            continue;
        }

        apply_key_value(key, val);
    }

    fclose(f);
    return 0;
}

bool MotorConfig_sanityCheck(void)
{
    bool ok = true;

    if (g_motor_cfg.pole_pairs <= 0.0f) {
        fprintf(stderr, "MotorConfig: invalid pole_pairs (%.2f)\n",
                g_motor_cfg.pole_pairs);
        ok = false;
    }
    if (g_motor_cfg.kv_rpm_per_v <= 0.0f) {
        fprintf(stderr, "MotorConfig: invalid kv_rpm_per_v (%.2f)\n",
                g_motor_cfg.kv_rpm_per_v);
        ok = false;
    }
    if (g_motor_cfg.bus_v_min_v <= 0.0f ||
        g_motor_cfg.bus_v_min_v >= g_motor_cfg.bus_v_max_v) {
        fprintf(stderr,
                "MotorConfig: invalid bus voltage range [%.2f, %.2f]\n",
                g_motor_cfg.bus_v_min_v,
                g_motor_cfg.bus_v_max_v);
        ok = false;
    }
    if (g_motor_cfg.fast_loop_hz <= 0.0f) {
        fprintf(stderr, "MotorConfig: invalid fast_loop_hz (%.2f)\n",
                g_motor_cfg.fast_loop_hz);
        ok = false;
    }
    if (g_motor_cfg.slow_loop_hz <= 0.0f) {
        fprintf(stderr, "MotorConfig: invalid slow_loop_hz (%.2f)\n",
                g_motor_cfg.slow_loop_hz);
        ok = false;
    }
    if (g_motor_cfg.pwm_freq_hz <= 0.0f) {
        fprintf(stderr, "MotorConfig: invalid pwm_freq_hz (%.2f)\n",
                g_motor_cfg.pwm_freq_hz);
        ok = false;
    }

    if (!ok) {
        fprintf(stderr, "MotorConfig: sanity check FAILED\n");
    }

    return ok;
}