#ifndef PI_H
#define PI_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Discrete PI controller instance
 *
 * u(k) = Kp * e(k) + I(k)
 * I(k) = I(k-1) + Ki * Ts * e(k)
 */
typedef struct
{
    float kp;          // Proportional gain
    float ki;          // Integral gain
    float Ts;          // Sample time [s]

    float integrator;  // Integral state (already includes Ki*Ts steps)
    float out_min;     // Minimum output (saturation)
    float out_max;     // Maximum output (saturation)

    float last_output; // Last computed output (for info/debug)
} PI_Controller_t;

/**
 * @brief Status flags for the PI step
 */
typedef enum
{
    PI_OK = 0,
    PI_SAT_HIGH,
    PI_SAT_LOW
} PI_Status_t;

/**
 * @brief Initialize PI controller
 *
 * @param pi       Controller instance
 * @param kp       Proportional gain
 * @param ki       Integral gain
 * @param Ts       Sample time [s]
 * @param out_min  Minimum output (e.g. -1.0f)
 * @param out_max  Maximum output (e.g.  1.0f)
 */
void PI_init(PI_Controller_t *pi,
             float kp,
             float ki,
             float Ts,
             float out_min,
             float out_max);

/**
 * @brief Reset PI internal state (integrator + output)
 */
void PI_reset(PI_Controller_t *pi);

/**
 * @brief Set PI gains (keeps integrator & saturation limits as-is)
 */
void PI_setGains(PI_Controller_t *pi,
                 float kp,
                 float ki);

/**
 * @brief Perform one PI step
 *
 * @param pi            Controller instance
 * @param ref           Reference value
 * @param meas          Measured value
 * @param use_antiwindup If true, prevents integrator windup when saturated
 *
 * @return Controller output (also stored in pi->last_output)
 *
 * Notes on anti-windup:
 *  - When saturated at out_max and error would push further positive,
 *    the integral term is NOT accumulated.
 *  - When saturated at out_min and error would push further negative,
 *    the integral term is NOT accumulated.
 */
float PI_step(PI_Controller_t *pi,
              float ref,
              float meas,
              bool use_antiwindup,
              PI_Status_t *status_out);

#endif // PI_H