#ifndef ADC_H
#define ADC_H

#include <stdint.h>

/**
 * @brief Initialize SPI ADC device (MCP3208)
 *
 * @param device SPI device path (e.g., "/dev/spidev0.0")
 * @return file descriptor >= 0 on success, -1 on error
 */
int adc_init(const char *device);

/**
 * @brief Read value from a specific MCP3208 ADC channel
 *
 * @param fd      SPI device file descriptor
 * @param channel ADC channel number (0–7)
 * @return 12-bit ADC value [0..4095], or -1 on error
 */
int adc_read_channel(int fd, int channel);

/**
 * @brief Read multiple channels [0..num_channels-1] in sequence
 *
 * @param fd           SPI device file descriptor
 * @param out_values   Caller-provided array to store results
 * @param num_channels Number of channels to read (1–8)
 * @return 0 on success, -1 on error
 */
int adc_read_channels(int fd, int *out_values, int num_channels);

/**
 * @brief Close SPI ADC device
 *
 * @param fd SPI device file descriptor
 */
void adc_close(int fd);

#endif // ADC_H