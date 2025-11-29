// main.c
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <errno.h>
#include <string.h>
#include <limits.h>

#include "motor_config.h"
#include "motor_config_runtime.h"

#include "hall.h"
#include "adc.h"
#include "bemf.h"
#include "drv8302.h"

#include "pwm_motor.h"
#include "motor_control.h"
#include "speed_measurement.h"
#include "position_estimator.h"
#include "sensorless_handover.h"

#include "fast_loop.h"
#include "slow_loop.h"

// ---------------- Global handles (used by loops etc.) ----------------

// Hall sensors
HallHandle_t g_hall;

// ADC + BEMF
int          g_adc_fd = -1;
BemfHandle_t g_bemf;

// PWM motor driver (phase U/V/W gate control)
PwmMotor_t   g_pwm_motor;

// Sensorless handover helper (Hall -> BEMF)
SensorlessHandover_t g_handover;

// Fast loop period (seconds) used by RT thread
float g_fast_period_s = 0.0f;

// Jitter fault flag (set if jitter exceeds threshold)
volatile bool g_fast_jitter_fault = false;

// ---------------- Utility: monotonic time in seconds ----------------

static float get_time_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (float)ts.tv_sec + (float)ts.tv_nsec * 1e-9f;
}

// ---------------- Jitter stats structure ----------------

typedef struct {
    uint64_t count;
    uint64_t min_ns;
    uint64_t max_ns;
    double   sum_ns;
    double   last_ns;
} LoopStats_t;

static LoopStats_t g_fast_stats = {
    .min_ns = UINT64_MAX,
    .max_ns = 0
};

// Jitter fault threshold (percent of target period)
#define FAST_JITTER_FAULT_PCT  10.0f

// ---------------- Hardware / system init ----------------

static int app_hw_init(void)
{
    // 1) Runtime motor config (defaults + optional file overrides)
    MotorConfig_initDefaults();
    // Ignore failure here; config file is optional
    MotorConfig_loadFromFile("motor_config.cfg");
    MotorConfig_sanityCheck();

    // 2) Hall sensors
    if (!Hall_init(&g_hall,
                   "/dev/gpiochip0",
                   HALL_A_OFFSET,
                   HALL_B_OFFSET,
                   HALL_C_OFFSET)) {
        fprintf(stderr, "Failed to init Hall sensors\n");
        return -1;
    }

    // 3) ADC + BEMF
    g_adc_fd = adc_init("/dev/spidev0.0");
    if (g_adc_fd < 0) {
        fprintf(stderr, "Failed to init ADC\n");
        return -1;
    }

    if (!Bemf_init(&g_bemf,
                   g_adc_fd,
                   BEMF_CH_U,
                   BEMF_CH_V,
                   BEMF_CH_W,
                   BEMF_CH_VBUS)) {
        fprintf(stderr, "Failed to init BEMF handle\n");
        return -1;
    }

    // 4) DRV8302 gate driver (optional but recommended)
    if (!Drv8302_init()) {
        fprintf(stderr, "DRV8302_init failed\n");
        return -1;
    }
    if (!Drv8302_configureDefaults()) {
        fprintf(stderr, "DRV8302_configureDefaults failed\n");
        // not fatal, but warn
    }
    if (!Drv8302_clearFaults()) {
        fprintf(stderr, "DRV8302_clearFaults failed\n");
        // again not necessarily fatal; depends how strict you want to be
    }

    // 5) PWM motor outputs (hooked to DRV8302 INH/INL pins)
    if (!PwmMotor_init(&g_pwm_motor,
                       "/dev/gpiochip0",
                       INH_A_OFFSET, INL_A_OFFSET,
                       INH_B_OFFSET, INL_B_OFFSET,
                       INH_C_OFFSET, INL_C_OFFSET)) {
        fprintf(stderr, "Failed to init PWM motor outputs\n");
        return -1;
    }

    // 6) Core motor control
    MotorControl_init(&g_pwm_motor);

    // 7) Speed + position estimator
    SpeedMeas_init();
    SpeedMeas_setHallHandle(&g_hall);
    SpeedMeas_setBemfHandle(&g_bemf);
    SpeedMeas_setMode(SPEED_SRC_HALL);       // start in Hall mode

    PosEst_init(POS_MODE_HALL);
    PosEst_setHallHandle(&g_hall);
    PosEst_setBemfHandle(&g_bemf);

    // 8) Sensorless handover (Hall -> BEMF)
    SensorlessHandover_init(&g_handover,
                            g_motor_cfg.sensorless_min_rpm_mech,
                            g_motor_cfg.sensorless_stable_samples);

    // 9) Loop timing using runtime config
    g_fast_period_s = 1.0f / g_motor_cfg.fast_loop_hz;
    float slow_period_s = 1.0f / g_motor_cfg.slow_loop_hz;

    FastLoop_init(g_fast_period_s);
    SlowLoop_init(slow_period_s);

    return 0;
}

// ---------------- Example: set initial command ----------------

static void app_set_initial_command(void)
{
    // Start disabled; you can later enable via CLI/UDP/etc.
    MotorControl_setEnable(false);
    MotorControl_setSpeedCmd(1000.0f, false);  // rpm_cmd = 1000, direction=false => forward
}

// ---------------- RT fast loop thread with jitter detection ----------------

static void *fast_loop_thread(void *arg)
{
    (void)arg;

    // ---- Set RT scheduling ----
    struct sched_param sp = { .sched_priority = 80 };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        perror("pthread_setschedparam");
        fprintf(stderr,
                "WARNING: Fast loop NOT running real-time. "
                "Expect large jitter.\n");
    }

    // ---- Prepare timing ----
    struct timespec next, now;
    clock_gettime(CLOCK_MONOTONIC, &next);

    const long target_period_ns = (long)(g_fast_period_s * 1e9);

    while (1) {
        // If jitter fault has occurred, we still keep the loop running
        // but you could choose to break here if you want to stop everything.
        float now_s = get_time_s();

        // One iteration of the fast loop
        FastLoop_step(now_s);

        // ---- Jitter measurement ----
        clock_gettime(CLOCK_MONOTONIC, &now);

        uint64_t now_ns =
            (uint64_t)now.tv_sec * 1000000000ULL + (uint64_t)now.tv_nsec;

        if (g_fast_stats.last_ns != 0) {
            uint64_t period = now_ns - (uint64_t)g_fast_stats.last_ns;

            if (period < g_fast_stats.min_ns)
                g_fast_stats.min_ns = period;
            if (period > g_fast_stats.max_ns)
                g_fast_stats.max_ns = period;

            g_fast_stats.sum_ns += (double)period;
            g_fast_stats.count++;

            // Once per second at the configured fast loop rate
            if (g_fast_stats.count == (uint64_t)g_motor_cfg.fast_loop_hz) {
                double avg_ns = g_fast_stats.sum_ns / g_fast_stats.count;
                uint64_t jitter_ns = g_fast_stats.max_ns - g_fast_stats.min_ns;
                double jitter_pct =
                    100.0 * (double)jitter_ns / (double)target_period_ns;

                printf(
                    "[FAST LOOP JITTER]\n"
                    "  Target: %ld ns (%.1f kHz)\n"
                    "  Min:    %llu ns\n"
                    "  Max:    %llu ns\n"
                    "  Avg:    %.1f ns\n"
                    "  Jitter: %llu ns (%.2f%% of period)\n",
                    target_period_ns,
                    (double)g_motor_cfg.fast_loop_hz / 1000.0,
                    (unsigned long long)g_fast_stats.min_ns,
                    (unsigned long long)g_fast_stats.max_ns,
                    avg_ns,
                    (unsigned long long)jitter_ns,
                    jitter_pct
                );

                // ---- Jitter fault detection ----
                if (jitter_pct > FAST_JITTER_FAULT_PCT && !g_fast_jitter_fault) {
                    g_fast_jitter_fault = true;
                    fprintf(stderr,
                            "!!! FAST LOOP JITTER FAULT: %.2f%% > %.2f%%\n",
                            jitter_pct, (double)FAST_JITTER_FAULT_PCT);

                    // Immediately disable motor for safety
                    MotorControl_setEnable(false);

                    // TODO: hook into your own fault system if you have one
                    // e.g. MotorControl_setFault(MOTOR_FAULT_TIMING);
                }

                // Reset for next 1s window
                g_fast_stats.count  = 0;
                g_fast_stats.min_ns = UINT64_MAX;
                g_fast_stats.max_ns = 0;
                g_fast_stats.sum_ns = 0.0;
            }
        }

        g_fast_stats.last_ns = (double)now_ns;

        // ---- Schedule next absolute wakeup ----
        next.tv_nsec += target_period_ns;
        while (next.tv_nsec >= 1000000000L) {
            next.tv_nsec -= 1000000000L;
            next.tv_sec++;
        }

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    return NULL;
}

// ---------------- main ----------------

int main(void)
{
    if (app_hw_init() < 0) {
        return 1;
    }

    app_set_initial_command();

    printf("Motor control app starting (fast loop RT thread)...\n");

    // Enable the motor once you’re ready. For now, we turn it on here.
    MotorControl_setEnable(true);

    // Start the fast loop thread (RT)
    pthread_t fast_thread;
    int err = pthread_create(&fast_thread, NULL, fast_loop_thread, NULL);
    if (err != 0) {
        fprintf(stderr,
                "Failed to create fast loop thread: %s\n", strerror(err));
        return 1;
    }

    // Main thread runs slow loop + any app-level logic
    while (1) {
        float now_s = get_time_s();

        SlowLoop_run(now_s);

        // You might want to check jitter fault here and react at a higher level:
        if (g_fast_jitter_fault) {
            // Example: print once; you could also transition to FAULT state
            // or blink an LED, send UDP, etc.
            // (You might want a 'reported' flag to avoid spamming.)
        }

        // Mild sleep; SlowLoop_run internally rate-limits
        struct timespec req = {
            .tv_sec  = 0,
            .tv_nsec = 1000000L  // 1 ms
        };
        nanosleep(&req, NULL);
    }

    return 0;
}