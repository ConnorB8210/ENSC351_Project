// motor_config.h
// ---------------------------------------------------------
// Board / Wiring Overview (logical → physical)
// ---------------------------------------------------------
//
// GPIO (BeagleBone/BeagleY-ai) → DRV8302 board:
//
//   HALL_A_OFFSET  --> Motor Hall A input
//   HALL_B_OFFSET  --> Motor Hall B input
//   HALL_C_OFFSET  --> Motor Hall C input
//
//   INH_A_OFFSET   --> DRV8302 INH_A (Phase U high-side gate)
//   INL_A_OFFSET   --> DRV8302 INL_A (Phase U low-side gate)
//   INH_B_OFFSET   --> DRV8302 INH_B (Phase V high-side gate)
//   INL_B_OFFSET   --> DRV8302 INL_B (Phase V low-side gate)
//   INH_C_OFFSET   --> DRV8302 INH_C (Phase W high-side gate)
//   INL_C_OFFSET   --> DRV8302 INL_C (Phase W low-side gate)
//
//   DRV_EN_GATE_OFFSET  --> DRV8302 EN_GATE (enable gate driver, active HIGH)
//   DRV_NFAULT_OFFSET   --> DRV8302 nFAULT (open-drain fault output, active LOW)
//   DRV_NOCTW_OFFSET    --> DRV8302 nOCTW (overcurrent/overtemp warning, active LOW)
//
// ADC (MCP3208) Channels:
//
//   BEMF_CH_U   --> DRV8302 EMF-A output (phase U back-EMF, attenuated by 5.1/73.1)
//   BEMF_CH_V   --> DRV8302 EMF-B output (phase V back-EMF, attenuated by 5.1/73.1)
//   BEMF_CH_W   --> DRV8302 EMF-C output (phase W back-EMF, attenuated by 5.1/73.1)
//   BEMF_CH_VBUS--> DRV8302 VPD_D-O output (bus voltage, attenuated by 5.1/73.1)
//
// Notes:
//   - All numeric offsets and channels are defined below as macros
//     so they are easy to update in one place.
//   - The runtime config layer (motor_config_runtime.c) can override
//     some numerical parameters (Kv, limits, etc.) from a config file.
//
// ---------------------------------------------------------
// Motor electrical/mechanical constants
// ---------------------------------------------------------
#pragma once

// --- Motor characteristics ---
#define MOTOR_POLE_PAIRS            4
#define MOTOR_KV_RPM_PER_V          1000.0f     // user‑tunable
#define MOTOR_R_PHASE_OHM           0.30f       // phase‑to‑phase/2
#define MOTOR_L_PHASE_H             0.0001f     // approximate

// --- Operating limits ---
#define MOTOR_I_MAX_A               10.0f
#define MOTOR_BUS_V_MAX_V           40.0f
#define MOTOR_BUS_V_MIN_V           8.0f
#define MOTOR_RPM_MAX               5000.0f

// ---------------------------------------------------------
// Control loop timing
// ---------------------------------------------------------
#define FAST_LOOP_HZ                20000       // 20 kHz fast loop
#define SLOW_LOOP_HZ                1000        // 1 kHz slow loop

#define CONTROL_LOOP_HZ             FAST_LOOP_HZ
#define SPEED_LOOP_HZ               SLOW_LOOP_HZ

// PWM frequency (for 6‑step commutation)
#define PWM_FREQUENCY_HZ            20000       // 20 kHz

// ---------------------------------------------------------
// ADC / BEMF sensing configuration
// ---------------------------------------------------------
#define ADC_REF_V                   1.65f       // mid‑rail reference from board
#define CURRENT_SENSE_GAIN          12.22f      // external diff amp gain
#define CURRENT_SHUNT_OHM           0.01f       // if applicable

// ADC channel mapping (for SPI MCP3208)
#define BEMF_CH_U                   0
#define BEMF_CH_V                   1
#define BEMF_CH_W                   2
#define BEMF_CH_VBUS                3

// Minimum Vbus where BEMF values make sense
#define BEMF_VALID_MIN_V            1.0f

// ---------------------------------------------------------
// Hall sensor configuration
// ---------------------------------------------------------
// GPIO offsets (change to match your wiring!)
#define HALL_A_OFFSET               7  //GPIO16
#define HALL_B_OFFSET               15 //GPIO5
#define HALL_C_OFFSET               11 //GPIO18

// Timeout if no edges detected
#define HALL_TIMEOUT_MS             200

// ---------------------------------------------------------
// DRV8302 / PWM pinout configuration
// ---------------------------------------------------------
// These must match your board wiring to the GPIO chip
#define INH_A_OFFSET                8  //GPIO17
#define INL_A_OFFSET                16 //GPIO12

#define INH_B_OFFSET                17 //GPIO06
#define INL_B_OFFSET                18 //GPIO13

#define INH_C_OFFSET                9  //GPIO21
#define INL_C_OFFSET                14 //GPIO14

// DRV8302 management pins (optional)
#define DRV_EN_GATE_OFFSET          41 //GPIO22
#define DRV_NFAULT_OFFSET           42 //GPIO25
#define DRV_NOCTW_OFFSET            33 //GPIO27

// ---------------------------------------------------------
// Sensorless (BEMF) run‑up / handover configuration
// ---------------------------------------------------------
#define SENSORLESS_MIN_RPM_MECH     500.0f      // must exceed this RPM
#define SENSORLESS_STABLE_SAMPLES   100         // consecutive samples needed

// ---------------------------------------------------------
// Utility macros
// ---------------------------------------------------------
#define DEG2RAD(x) ((x) * 0.01745329252f)
#define RAD2DEG(x) ((x) * 57.295779513f)
