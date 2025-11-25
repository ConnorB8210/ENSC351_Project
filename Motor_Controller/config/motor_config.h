//No Logic Contained, just definitions for Motor and Board

#pragma once

// Motor mechanical / electrical data
#define MOTOR_POLE_PAIRS        4
#define MOTOR_KV_RPM_PER_V      1000.0f   // adjustable 
#define MOTOR_R_PHASE_OHM       0.3f      // phase-to-phase/2
#define MOTOR_L_PHASE_H         0.0001f   // rough

// Limits
#define MOTOR_I_MAX_A           10.0f
#define MOTOR_BUS_V_MAX_V       40.0f
#define MOTOR_BUS_V_MIN_V       8.0f
#define MOTOR_RPM_MAX           5000.0f

// PWM & timing
#define CONTROL_LOOP_HZ         10000     // inner loop
#define SPEED_LOOP_HZ           1000      // outer loop
#define PWM_FREQUENCY_HZ        20000

// Sensing
#define ADC_REF_V               1.65f
#define CURRENT_SENSE_GAIN      12.22f    // board manual
#define CURRENT_SHUNT_OHM       0.01f     // if applicable

// Hall timing sanity
#define HALL_TIMEOUT_MS         200       // if no edges -> fault