#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * Back-EMF measurement handle.
 *
 * adc_fd:   SPI ADC file descriptor (from adc_init("/dev/spidevX.Y"))
 * ch_emf_*: ADC channels that measure EMF_A/B/C from the DRV830x board
 * ch_vbus:  ADC channel that measures VPD_D-O (scaled Vbus)
 *
 * v_emf_*:  phase voltages in *real bus volts* (after de-attenuation)
 * v_vbus:   DC bus voltage in volts
 */
typedef struct {
    int      adc_fd;

    uint8_t  ch_emf_u;
    uint8_t  ch_emf_v;
    uint8_t  ch_emf_w;
    uint8_t  ch_vbus;

    float    v_emf_u;
    float    v_emf_v;
    float    v_emf_w;
    float    v_vbus;
} BemfHandle_t;

/**
 * @brief Initialize BEMF measurement handle.
 *
 * Does not own adc_fd; you open/close ADC outside this module.
 */
bool Bemf_init(BemfHandle_t *h,
               int adc_fd,
               uint8_t ch_emf_u,
               uint8_t ch_emf_v,
               uint8_t ch_emf_w,
               uint8_t ch_vbus);

/**
 * @brief Sample ADC channels and update phase + Vbus voltages.
 *
 * Call this from your fast loop (or a dedicated ADC task).
 */
void Bemf_update(BemfHandle_t *h);

/**
 * @brief Get phase voltage (in volts) for U/V/W.
 *
 * phase: 0 = U, 1 = V, 2 = W
 */
float Bemf_getPhaseVoltage(const BemfHandle_t *h, uint8_t phase);

/**
 * @brief Get DC bus voltage (in volts).
 */
float Bemf_getVbus(const BemfHandle_t *h);

/**
 * @brief Get phase voltage minus neutral (Vbus/2) in volts.
 *
 * This is the key quantity for zero-cross detection in 6-step BEMF
 * control: you watch for sign changes of this value on the floating
 * phase during each sector.
 *
 * phase: 0 = U, 1 = V, 2 = W
 */
float Bemf_getNeutralDiff(const BemfHandle_t *h, uint8_t phase);