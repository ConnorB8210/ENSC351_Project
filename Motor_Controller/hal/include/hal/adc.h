#ifndef ADC_H
#define ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize SPI ADC device
 * @param device SPI device path (e.g., "/dev/spidev0.0")
 * @return file descriptor or -1 on error
 */
int adc_init(const char *device);

/**
 * @brief Read a value from a specific ADC channel
 * @param fd SPI device file descriptor
 * @param channel ADC channel number
 * @return 12-bit ADC value, or -1 on error
 */
int adc_read_channel(int fd, int channel);

/**
 * @brief Close the SPI ADC device
 * @param fd SPI device file descriptor
 */
void adc_close(int fd);

#ifdef __cplusplus
}
#endif

#endif // ADC_H