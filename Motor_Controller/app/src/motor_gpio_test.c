// app/src/motor_gpio_test.c
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>

#include "motor_config.h"
#include "pwm_motor.h"
#include "hall.h"
#include "hall_commutator.h"
#include "bemf.h"
#include "adc.h"
#include "gpio.h"

static volatile sig_atomic_t g_stop = 0;

static void handle_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}

static double get_time_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

int main(void)
{
    signal(SIGINT, handle_sigint);
    signal(SIGTERM, handle_sigint);

    int adc_fd = -1;
    BemfHandle_t bemf;
    PwmMotor_t pwm;
    HallHandle_t hall;
    GPIO_Handle *drv_en_gpio = NULL;

    printf("Motor GPIO test starting...\n");

    // ---- ADC + BEMF init ----
    adc_fd = adc_init("/dev/spidev0.0");
    if (adc_fd < 0) {
        fprintf(stderr, "adc_init failed\n");
        return 1;
    }
    if (!Bemf_initDefault(&bemf, adc_fd)) {
        fprintf(stderr, "Bemf_initDefault failed\n");
        adc_close(adc_fd);
        return 1;
    }

    // ---- PWM motor (GPIO-backed) ----
    if (!PwmMotor_init(&pwm,
                       "/dev/gpiochip2",
                       INH_A_OFFSET, INL_A_OFFSET,
                       INH_B_OFFSET, INL_B_OFFSET,
                       INH_C_OFFSET, INL_C_OFFSET)) {
        fprintf(stderr, "PwmMotor_init failed\n");
        adc_close(adc_fd);
        return 1;
    }

    // ---- Optional EN_GATE control (comment out if it causes trouble) ----
#if 1
    {
        unsigned int offs[1] = { DRV_EN_GATE_OFFSET };
        drv_en_gpio = gpio_init("/dev/gpiochip1",
                                offs,
                                1,
                                GPIOD_LINE_DIRECTION_OUTPUT,
                                GPIOD_LINE_EDGE_NONE);
        if (!drv_en_gpio) {
            fprintf(stderr, "EN_GATE gpio_init failed (continuing anyway)\n");
        } else {
            // Drive EN_GATE high
            gpio_write(drv_en_gpio, 0, 1);
        }
    }
#endif

    // ---- Hall init ----
    if (!Hall_init(&hall,
                   "/dev/gpiochip2",
                   HALL_A_OFFSET,
                   HALL_B_OFFSET,
                   HALL_C_OFFSET)) {
        fprintf(stderr, "Hall_init failed\n");
        if (drv_en_gpio) {
            gpio_write(drv_en_gpio, 0, 0);
            gpio_close(drv_en_gpio);
        }
        PwmMotor_deinit(&pwm);
        adc_close(adc_fd);
        return 1;
    }

    printf("Motor GPIO test initialized.\n");
    printf("Cycling through 6-step sectors with fixed duty.\n");

    const float duty = 0.20f;        // 20% "duty" (really just ON/OFF in this GPIO version)
    const double step_interval_s = 0.200; // 200 ms per sector

    double t_start = get_time_s();
    int sector_cmd = 0;

    while (!g_stop) {
        double t_now = get_time_s();
        double t_rel = t_now - t_start;

        // Update BEMF / VBUS
        Bemf_update(&bemf);
        float vbus = Bemf_getVbus(&bemf);

        // Command sector via PwmMotor
        if (sector_cmd < 0 || sector_cmd > 5) {
            sector_cmd = 0;
        }
        PwmMotor_setSixStep(&pwm, (uint8_t)sector_cmd, duty, true); // forward

        // Read hall bits and sector
        uint8_t hall_bits = Hall_readBits(&hall);
        uint8_t hall_sector = HallComm_hallToSector(hall_bits);

        printf("t=%.3f  VBUS=%.2f V  sector_cmd=%d  duty=%.2f  "
               "hall_bits=0x%02X hall_sector=%u\n",
               t_rel, vbus, sector_cmd, duty,
               (unsigned int)hall_bits, (unsigned int)hall_sector);
        fflush(stdout);

        // Next sector
        sector_cmd = (sector_cmd + 1) % 6;

        // Sleep ~step_interval_s
        struct timespec ts;
        ts.tv_sec  = (time_t)step_interval_s;
        ts.tv_nsec = (long)((step_interval_s - (double)ts.tv_sec) * 1e9);
        nanosleep(&ts, NULL);
    }

    printf("Motor GPIO test exiting...\n");

    // Clean up
    PwmMotor_setSixStep(&pwm, 0, 0.0f, true); // all off
    PwmMotor_deinit(&pwm);

    if (drv_en_gpio) {
        gpio_write(drv_en_gpio, 0, 0); // EN_GATE low
        gpio_close(drv_en_gpio);
    }

    Hall_close(&hall);

    if (adc_fd >= 0) {
        adc_close(adc_fd);
    }

    return 0;
}