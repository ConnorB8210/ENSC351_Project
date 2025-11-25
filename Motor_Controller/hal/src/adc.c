#include "adc.h"
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

/**
 * @brief Initialize SPI ADC device
 * @param device SPI device (e.g., "/dev/spidev0.0")
 * @return file descriptor or -1 on error
 */
int adc_init(const char *device) {
    return open(device, O_RDWR);
}

/**
 * @brief Read value from a specific ADC channel
 * @param fd SPI device file descriptor
 * @param channel ADC channel number
 * @return 12-bit ADC value or -1 on error
 */
int adc_read_channel(int fd, int channel) {
    uint8_t tx[] = {
        0x06 | ((channel & 0x04) >> 2),
        (channel & 0x03) << 6,
        0x00
    };
    uint8_t rx[3] = {0};

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 3,
        .speed_hz = 1000000,
        .delay_usecs = 0,
        .bits_per_word = 8,
    };

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        perror("SPI transfer failed");
        return -1;
    }

    return ((rx[1] & 0x0F) << 8) | rx[2];
}

/**
 * @brief Close SPI ADC device
 */
void adc_close(int fd) {
    if (fd >= 0) close(fd);
}