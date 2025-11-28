// bemf.c

#include "bemf.h"
#include "adc.h"    // adc_read_channel()
#include <stddef.h> // NULL

// -------- Config / scaling --------
//
// ADC: MCP3208, 12-bit, typically 3.3 V reference
// DRV830x board: EMF and VPD outputs are attenuated by 5.1 / 73.1,
// so to recover real bus/phase voltage you multiply by 73.1 / 5.1.
//

#ifndef BEMF_ADC_MAX_COUNTS
#define BEMF_ADC_MAX_COUNTS  4095.0f
#endif

#ifndef BEMF_ADC_REF_V
#define BEMF_ADC_REF_V       3.3f
#endif

#ifndef BEMF_EMF_ATTEN_RATIO
// Board output is EMF * (5.1 / 73.1) so we multiply by the inverse.
#define BEMF_EMF_ATTEN_RATIO (73.1f / 5.1f)
#endif

// Same scaling for Vbus sense (VPD_D-O), if wired the same way.
#ifndef BEMF_VBUS_ATTEN_RATIO
#define BEMF_VBUS_ATTEN_RATIO (73.1f / 5.1f)
#endif

// Convert raw ADC counts -> pin voltage
static inline float adc_counts_to_pin_v(int counts)
{
    if (counts < 0) return 0.0f;
    return ( (float)counts * (BEMF_ADC_REF_V / BEMF_ADC_MAX_COUNTS) );
}

// Convert raw ADC counts on EMF pin -> phase voltage in bus volts
static inline float adc_emf_to_phase_v(int counts)
{
    float v_pin = adc_counts_to_pin_v(counts);
    return v_pin * BEMF_EMF_ATTEN_RATIO;
}

// Convert raw ADC counts on VPD_D-O -> DC bus volts
static inline float adc_vpd_to_vbus(int counts)
{
    float v_pin = adc_counts_to_pin_v(counts);
    return v_pin * BEMF_VBUS_ATTEN_RATIO;
}

bool Bemf_init(BemfHandle_t *h,
               int adc_fd,
               uint8_t ch_emf_u,
               uint8_t ch_emf_v,
               uint8_t ch_emf_w,
               uint8_t ch_vbus)
{
    if (!h || adc_fd < 0) {
        return false;
    }

    h->adc_fd   = adc_fd;
    h->ch_emf_u = ch_emf_u;
    h->ch_emf_v = ch_emf_v;
    h->ch_emf_w = ch_emf_w;
    h->ch_vbus  = ch_vbus;

    h->v_emf_u  = 0.0f;
    h->v_emf_v  = 0.0f;
    h->v_emf_w  = 0.0f;
    h->v_vbus   = 0.0f;

    return true;
}

void Bemf_update(BemfHandle_t *h)
{
    if (!h || h->adc_fd < 0) return;

    // Read raw ADC channels
    int raw_u = adc_read_channel(h->adc_fd, h->ch_emf_u);
    int raw_v = adc_read_channel(h->adc_fd, h->ch_emf_v);
    int raw_w = adc_read_channel(h->adc_fd, h->ch_emf_w);
    int raw_vbus = adc_read_channel(h->adc_fd, h->ch_vbus);

    // Convert to real voltages
    h->v_emf_u = adc_emf_to_phase_v(raw_u);
    h->v_emf_v = adc_emf_to_phase_v(raw_v);
    h->v_emf_w = adc_emf_to_phase_v(raw_w);
    h->v_vbus  = adc_vpd_to_vbus(raw_vbus);
}

float Bemf_getPhaseVoltage(const BemfHandle_t *h, uint8_t phase)
{
    if (!h) return 0.0f;

    switch (phase) {
        case 0: return h->v_emf_u;
        case 1: return h->v_emf_v;
        case 2: return h->v_emf_w;
        default: return 0.0f;
    }
}

float Bemf_getVbus(const BemfHandle_t *h)
{
    if (!h) return 0.0f;
    return h->v_vbus;
}

float Bemf_getNeutralDiff(const BemfHandle_t *h, uint8_t phase)
{
    if (!h) return 0.0f;

    float v_phase = Bemf_getPhaseVoltage(h, phase);
    float vbus    = h->v_vbus;

    if (vbus <= 0.0f) {
        return 0.0f;
    }

    float v_neutral = 0.5f * vbus;  // assume star connection, neutral ~ Vbus/2
    return v_phase - v_neutral;
}