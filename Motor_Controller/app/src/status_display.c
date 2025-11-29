// status_display.c
#include "status_display.h"
#include "drumBeats.h"
#include "audioMixer.h"
#include "periodTimer.h"

#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>

static pthread_t display_thread;
static int keepRunning = 0;

static void* display_thread_func(void *arg);

void StatusDisplay_init(void)
{
    keepRunning = 1;
    if (pthread_create(&display_thread, NULL, display_thread_func, NULL) != 0) {
        perror("StatusDisplay: Failed to create status thread");
        exit(EXIT_FAILURE);
    }
    printf("Status display started.\n");
}

void StatusDisplay_cleanup(void)
{
    keepRunning = 0;
    pthread_join(display_thread, NULL);
    printf("Status display stopped.\n");
}

static void* display_thread_func(void *arg)
{
    (void)arg;  // suppress unused parameter warning

    while (keepRunning) {
        sleep(1);  // once per second

        // --- Get current mode, BPM, volume ---
        beat_mode_t mode = DrumBeats_getMode();
        int bpm = DrumBeats_getBpm();
        int vol = AudioMixer_getVolume();

        // --- Get audio buffer refill stats ---
        Period_statistics_t audioStats;
        Period_getStatisticsAndClear(PERIOD_EVENT_AUDIO_BUFFER, &audioStats);

        double audioMin = audioStats.minPeriodInMs;
        double audioMax = audioStats.maxPeriodInMs;
        double audioAvg = audioStats.avgPeriodInMs;
        long   audioN   = audioStats.numSamples;

        // --- Get accelerometer sampling stats ---
        Period_statistics_t accelStats;
        Period_getStatisticsAndClear(PERIOD_EVENT_ACCELEROMETER, &accelStats);

        double accelMin = accelStats.minPeriodInMs;
        double accelMax = accelStats.maxPeriodInMs;
        double accelAvg = accelStats.avgPeriodInMs;
        long   accelN   = accelStats.numSamples;

        // Print in required format:
        // M0 90bpm vol:80 Audio[16.283, 16.942] avg 16.667/61 Accel[12.276, 13.965] avg 12.998/77
        printf(
            "M%d %dbpm vol:%d "
            "Audio[%.3f, %.3f] avg %.3f/%ld "
            "Accel[%.3f, %.3f] avg %.3f/%ld\n",
            (int)mode, bpm, vol,
            audioMin, audioMax, audioAvg, audioN,
            accelMin, accelMax, accelAvg, accelN
        );

        fflush(stdout);
    }

    return NULL;
}