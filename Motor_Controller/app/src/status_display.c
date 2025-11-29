// status_display.c
#include "status_display.h"
#include "motor_control.h"
#include "motor_states.h"
#include "position_estimator.h"

#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

// Must match the enum in main.c
typedef enum {
    SENSOR_MODE_HALL_ONLY = 0,
    SENSOR_MODE_AUTO      = 1,
    SENSOR_MODE_BEMF_ONLY = 2
} SensorMode_t;

// Provided by main.c
extern SensorMode_t Control_getSensorMode(void);

static pthread_t display_thread;
static int keepRunning = 0;

static void* display_thread_func(void *arg);

// --- small helpers to stringify enums (same idea as in UDP server) ---
static const char* motor_state_to_str(MotorState_t s)
{
    switch (s) {
    case MOTOR_STATE_IDLE:  return "IDLE";
    case MOTOR_STATE_ALIGN: return "ALIGN";
    case MOTOR_STATE_RUN:   return "RUN";
    case MOTOR_STATE_FAULT: return "FAULT";
    default:                return "UNKNOWN";
    }
}

static const char* motor_fault_to_str(MotorFault_t f)
{
    switch (f) {
    case MOTOR_FAULT_NONE:        return "NONE";
    case MOTOR_FAULT_OVERCURRENT: return "OVERCURRENT";
    case MOTOR_FAULT_OVERVOLT:    return "OVERVOLT";
    case MOTOR_FAULT_UNDERVOLT:   return "UNDERVOLT";
    case MOTOR_FAULT_HALL_TIMEOUT:return "HALL_TIMEOUT";
    case MOTOR_FAULT_DRV8302:     return "DRV8302";
    case MOTOR_FAULT_TIMING:      return "TIMING";
    default:                      return "UNKNOWN";
    }
}

static const char* sensor_mode_to_str(SensorMode_t m)
{
    switch (m) {
    case SENSOR_MODE_HALL_ONLY: return "HALL_ONLY";
    case SENSOR_MODE_AUTO:      return "AUTO";
    case SENSOR_MODE_BEMF_ONLY: return "BEMF_ONLY";
    default:                    return "UNKNOWN";
    }
}

// ----------------------------------------------------
// Public API
// ----------------------------------------------------
void StatusDisplay_init(void)
{
    if (keepRunning) {
        // already running
        return;
    }

    keepRunning = 1;
    if (pthread_create(&display_thread, NULL, display_thread_func, NULL) != 0) {
        perror("StatusDisplay: Failed to create status thread");
        exit(EXIT_FAILURE);
    }
    printf("Status display started.\n");
}

void StatusDisplay_cleanup(void)
{
    if (!keepRunning) {
        return;
    }

    keepRunning = 0;
    pthread_join(display_thread, NULL);
    printf("Status display stopped.\n");
}

// ----------------------------------------------------
// Thread body
// ----------------------------------------------------
static void* display_thread_func(void *arg)
{
    (void)arg;  // suppress unused parameter warning

    while (keepRunning) {
        sleep(1);  // once per second

        MotorContext_t ctx = MotorControl_getContext();
        PosEst_t pe        = PosEst_get();
        SensorMode_t sm    = Control_getSensorMode();

        // ctx.meas.rpm_mech should be kept in sync by MotorControl_stepSlow()
        // and SpeedMeas/PosEst.
        // We also assume MotorControl_updateBusVoltage() is populating v_bus.
        float rpm_mech = ctx.meas.rpm_mech;
        float rpm_cmd  = ctx.cmd.rpm_cmd;
        float duty     = ctx.cmd.torque_cmd;   // currently used as duty scalar
        float vbus     = ctx.meas.v_bus;       // make sure your context has this
        int   dir      = ctx.cmd.direction;    // 0=fwd, 1=rev

        // Raw electrical telemetry from PosEst
        float elec_angle = pe.elec_angle;
        float elec_speed = pe.elec_speed;
        unsigned sector  = pe.sector;

        // Print in a compact, always-same-format line for easy logging/grep.
        //
        // Example:
        // STATE=2(RUN) FAULT=0(NONE) EN=1
        // RPM=1234.5 CMD=1500.0 DUTY=0.350 DIR=0
        // SECTOR=3 ELEC_ANG=1.57 ELEC_RPM=4938.1
        // VBUS=23.45 SENSOR_MODE=1(AUTO)
        //
        // All on one line:
        printf("STATE=%d(%s) FAULT=%d(%s) EN=%d "
               "RPM=%.1f CMD=%.1f DUTY=%.3f DIR=%d "
               "SECTOR=%u ELEC_ANG=%.3f ELEC_RPM=%.1f "
               "VBUS=%.2f SENSOR_MODE=%d(%s)\n",
               ctx.state,
               motor_state_to_str(ctx.state),
               ctx.fault,
               motor_fault_to_str(ctx.fault),
               ctx.cmd.enable ? 1 : 0,
               rpm_mech,
               rpm_cmd,
               duty,
               dir,
               sector,
               elec_angle,
               elec_speed,
               vbus,
               (int)sm,
               sensor_mode_to_str(sm));

        fflush(stdout);
    }

    return NULL;
}