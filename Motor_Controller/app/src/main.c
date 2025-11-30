// main.c
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>

#include "motor_config.h"
#include "motor_states.h"
#include "motor_control.h"
#include "pwm_motor.h"
#include "hall.h"
#include "bemf.h"
#include "adc.h"
#include "position_estimator.h"
#include "udp_server.h"
#include "gpio.h"
#include "speed_measurement.h"
#include "sensorless_handover.h"
#include "status_display.h"    

typedef enum {
    SENSOR_MODE_HALL_ONLY = 0,
    SENSOR_MODE_AUTO      = 1,
    SENSOR_MODE_BEMF_ONLY = 2
} SensorMode_t;

// ---------------- Global hardware handles ----------------
static volatile sig_atomic_t g_stop = 0;

static int           g_adc_fd    = -1;
static BemfHandle_t  g_bemf;
static PwmMotor_t    g_pwm_motor;
static HallHandle_t  g_hall;

// Optional: simple gate‑enable GPIO (EN_GATE)
static GPIO_Handle  *g_drv_en_gpio = NULL;

static pthread_t            g_fast_thread;
static SensorlessHandover_t g_handover;
static SensorMode_t         g_sensor_mode = SENSOR_MODE_HALL_ONLY;

// Forward‑declared so udp_server.c / status_display.c can use them
SensorMode_t Control_getSensorMode(void);
void         Control_setSensorMode(SensorMode_t mode);

// ---------------- Time helpers ----------------
static double get_time_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

// ---------------- Sensor mode control ----------------
SensorMode_t Control_getSensorMode(void)
{
    return g_sensor_mode;
}

void Control_setSensorMode(SensorMode_t mode)
{
    g_sensor_mode = mode;

    switch (mode) {
    case SENSOR_MODE_HALL_ONLY:
        // Hall sensors only
        SpeedMeas_setMode(SPEED_SRC_HALL);
        PosEst_setMode(POS_MODE_HALL);
        SensorlessHandover_setEnable(&g_handover, false);
        break;

    case SENSOR_MODE_AUTO:
        // Start in Hall; allow handover helper to switch to BEMF
        SpeedMeas_setMode(SPEED_SRC_HALL);
        PosEst_setMode(POS_MODE_HALL);
        SensorlessHandover_setEnable(&g_handover, true);
        break;

    case SENSOR_MODE_BEMF_ONLY:
        // Force sensorless
        SpeedMeas_setMode(SPEED_SRC_BEMF);
        PosEst_setMode(POS_MODE_BEMF);
        SensorlessHandover_setEnable(&g_handover, false);
        break;

    default:
        break;
    }
}

// ---------------- Signal handler ----------------
static void handle_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}

// ---------------- Hardware init / deinit ----------------
static int app_hw_init(void)
{
    // --- ADC / BEMF ---
    g_adc_fd = adc_init("/dev/spidev0.0");
    if (g_adc_fd < 0) {
        fprintf(stderr, "adc_init failed\n");
        return -1;
    }

    if (!Bemf_initDefault(&g_bemf, g_adc_fd)) {
        fprintf(stderr, "Bemf_initDefault failed\n");
        adc_close(g_adc_fd);
        g_adc_fd = -1;
        return -1;
    }

    // --- PWM motor / DRV8302 gate driver ---
    if (!PwmMotor_init(&g_pwm_motor,
                       "/dev/gpiochip2",
                       INH_A_OFFSET, INL_A_OFFSET,
                       INH_B_OFFSET, INL_B_OFFSET,
                       INH_C_OFFSET, INL_C_OFFSET)) {
        fprintf(stderr, "PwmMotor_init failed\n");
        adc_close(g_adc_fd);
        g_adc_fd = -1;
        return -1;
    }

    // Optional: enable DRV8302 EN_GATE via GPIO
    {
        unsigned int offs[1] = { DRV_EN_GATE_OFFSET };
        g_drv_en_gpio = gpio_init("/dev/gpiochip1",
                                  offs,
                                  1,
                                  GPIOD_LINE_DIRECTION_OUTPUT,
                                  GPIOD_LINE_EDGE_NONE);
        if (!g_drv_en_gpio) {
            fprintf(stderr, "EN_GATE gpio_init failed\n");
            // Not fatal; we can still run without explicit gate enable
        } else {
            // Drive EN_GATE high (index 0 in this handle)
            gpio_write(g_drv_en_gpio, 0, 1);
        }
    }

    // --- Hall sensors ---
    if (!Hall_init(&g_hall,
                   "/dev/gpiochip2",
                   HALL_A_OFFSET,
                   HALL_B_OFFSET,
                   HALL_C_OFFSET)) {
        fprintf(stderr, "Hall_init failed\n");
        return -1;
    }

    // --- Speed measurement (Hall + BEMF) ---
    SpeedMeas_init();
    SpeedMeas_setHallHandle(&g_hall);
    SpeedMeas_setBemfHandle(&g_bemf);

    // --- Motor control + position estimator ---
    MotorControl_init(&g_pwm_motor);
    PosEst_init(POS_MODE_HALL);   // initial mode; Control_setSensorMode will refine

    // --- Sensorless handover helper ---
    SensorlessHandover_init(&g_handover,
                            SENSORLESS_MIN_RPM_MECH,     // from motor_config.h
                            SENSORLESS_STABLE_SAMPLES);  // from motor_config.h
    SensorlessHandover_setEnable(&g_handover, false);

    // Start in Hall‑only mode for first bring‑up
    Control_setSensorMode(SENSOR_MODE_HALL_ONLY);

    return 0;
}

static void app_hw_deinit(void)
{
    // Ensure motor is disabled
    MotorControl_setEnable(false);

    // PWM driver off
    PwmMotor_stop(&g_pwm_motor);
    PwmMotor_deinit(&g_pwm_motor);

    // Gate driver EN low + close
    if (g_drv_en_gpio) {
        gpio_write(g_drv_en_gpio, 0, 0);
        gpio_close(g_drv_en_gpio);
        g_drv_en_gpio = NULL;
    }

    // Hall close
    Hall_close(&g_hall);

    // ADC close
    if (g_adc_fd >= 0) {
        adc_close(g_adc_fd);
        g_adc_fd = -1;
    }
}

// ---------------- Fast loop (control) thread ----------------
static void *fast_loop_thread(void *arg)
{
    (void)arg;

    // Try to make this a real‑time SCHED_FIFO thread (best‑effort)
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = 80; // tune as needed, requires CAP_SYS_NICE
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        perror("pthread_setschedparam (SCHED_FIFO) failed; running non-RT");
    }

    const double Ts        = 1.0 / (double)FAST_LOOP_HZ;
    const double jitter_hi = Ts * 3.0;   // upper bound on acceptable dt
    const double jitter_lo = Ts * 0.1;   // lower bound

    double t_prev = get_time_s();
    double t_next = t_prev + Ts;

    static int timing_warn_count = 0;

    while (!g_stop) {
        double t_now = get_time_s();
        double dt    = t_now - t_prev;

        // Jitter / timing *logging* (no hard latch for first bring‑up)
        if (dt > jitter_hi || dt < jitter_lo) {
            if ((timing_warn_count++ % 100) == 0) {
                fprintf(stderr,
                        "WARN: fast-loop jitter dt=%.6f s (expected %.6f s)\n",
                        dt, Ts);
            }
            // If you later want to hard‑trip:
            // MotorControl_setFault(MOTOR_FAULT_TIMING);
        }

        t_prev = t_now;

        // Fast control step (commutation + duty apply)
        MotorControl_stepFast();

        // Simple sleep‑until‑next logic
        t_now = get_time_s();
        double sleep_s = t_next - t_now;
        if (sleep_s < 0.0) {
            // We're late; push next deadline forward
            t_next = t_now + Ts;
            continue;
        }

        struct timespec ts;
        ts.tv_sec  = (time_t)sleep_s;
        ts.tv_nsec = (long)((sleep_s - (double)ts.tv_sec) * 1e9);
        nanosleep(&ts, NULL);

        t_next += Ts;
    }

    return NULL;
}

// ---------------- Slow loop (1 kHz) ----------------
static void slow_loop_step(void)
{  
    // Time stamp for speed measurement + handover
    double now_s = get_time_s();

    // 1) Update BEMF / Vbus sensing
    Bemf_update(&g_bemf);
    float vbus = Bemf_getVbus(&g_bemf);

    // 2) Give bus voltage to motor control (stores v_bus + OV/UV faults)
    MotorControl_updateBusVoltage(vbus);

    // 3) Update speed / sector from Hall or BEMF
    SpeedMeas_update((float)now_s);
    //SpeedEstimate_t spd = SpeedMeas_get();

    // 4) Run sensorless handover helper (Hall -> BEMF) if AUTO mode
    if (g_sensor_mode == SENSOR_MODE_AUTO) {
        MotorContext_t ctx = MotorControl_getContext();
        bool dir_fwd = (ctx.cmd.direction == 0);  // 0 = forward

        (void)SensorlessHandover_step(&g_handover,
                                      (float)now_s,                                     
                                      dir_fwd);
    }

    // 5) Update position estimator (uses SpeedMeas_get())
    PosEst_update();

    // 6) Slow motor control (state machine + PI + slew/direction logic)
    MotorControl_stepSlow();
}

// ---------------- main() ----------------
int main(void)
{
    signal(SIGINT, handle_sigint);
    signal(SIGTERM, handle_sigint);

    if (app_hw_init() < 0) {
        fprintf(stderr, "Hardware init failed, exiting.\n");
        return 1;
    }

    // Start with motor disabled; UDP can enable/set RPM
    MotorControl_setEnable(false);

    // Start UDP server (remote control)
    if (!UDPServer_init()) {
        fprintf(stderr, "Warning: UDP server failed to start; continuing without it.\n");
    }

    // Start periodic status display (telemetry to stdout)
    StatusDisplay_init();

    // Launch fast loop RT thread
    int err = pthread_create(&g_fast_thread, NULL, fast_loop_thread, NULL);
    if (err != 0) {
        fprintf(stderr, "Failed to create fast loop thread: %s\n", strerror(err));
        UDPServer_cleanup();
        StatusDisplay_cleanup();
        app_hw_deinit();
        return 1;
    }

    printf("Motor control app running.\n");
    printf("  FAST_LOOP_HZ  = %d\n", FAST_LOOP_HZ);
    printf("  SPEED_LOOP_HZ = %d\n", SPEED_LOOP_HZ);

    // Main thread: slow loop (approx SPEED_LOOP_HZ) + supervision
    const double slow_Ts = 1.0 / (double)SPEED_LOOP_HZ;

    while (!g_stop) {
        double t0 = get_time_s();

        slow_loop_step();

        // Check UDP "stop" request
        if (UDPServer_wasStopRequested()) {
            printf("UDP requested shutdown.\n");
            g_stop = 1;
            break;
        }

        // Throttle to ~SPEED_LOOP_HZ (not hard RT, just approximate)
        double t1      = get_time_s();
        double elapsed = t1 - t0;
        double sleep_s = slow_Ts - elapsed;

        if (sleep_s > 0.0) {
            struct timespec ts;
            ts.tv_sec  = (time_t)sleep_s;
            ts.tv_nsec = (long)((sleep_s - (double)ts.tv_sec) * 1e9);
            nanosleep(&ts, NULL);
        }
    }

    printf("Shutting down...\n");

    // Stop UDP and fast loop
    UDPServer_cleanup();
    g_stop = 1;
    pthread_join(g_fast_thread, NULL);

    // Stop status display thread
    StatusDisplay_cleanup();

    // Make sure motor is off
    MotorControl_setEnable(false);
    app_hw_deinit();

    printf("Motor control app exited.\n");
    return 0;
}