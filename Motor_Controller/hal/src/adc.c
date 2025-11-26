#include "adc.h"

#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

#define SPI_MODE_DEFAULT   SPI_MODE_0
#define SPI_SPEED_DEFAULT  2000000    // 2 MHz: safe for MCP3208
#define SPI_BITS           8

/**
 * @brief Initialize SPI ADC device (MCP3208)
 * @param device SPI device (e.g., "/dev/spidev0.0")
 * @return file descriptor or -1 on error
 */
int adc_init(const char *device)
{
    int fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("SPI open failed");
        return -1;
    }

    uint8_t mode = SPI_MODE_DEFAULT;
    uint32_t speed = SPI_SPEED_DEFAULT;
    uint8_t bits = SPI_BITS;

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1) {
        perror("SPI mode set failed");
        close(fd);
        return -1;
    }

    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) {
        perror("SPI bits set failed");
        close(fd);
        return -1;
    }

    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) {
        perror("SPI speed set failed");
        close(fd);
        return -1;
    }

    return fd;
}

/**
 * @brief Read value from a specific MCP3208 ADC channel
 * @param fd SPI device file descriptor
 * @param channel ADC channel number (0–7)
 * @return 12-bit ADC value or -1 on error
 */
int adc_read_channel(int fd, int channel)
{
    if (channel < 0 || channel > 7) {
        fprintf(stderr, "adc_read_channel: invalid channel %d\n", channel);
        return -1;
    }

    // MCP3208 single-ended read command:
    // Byte 0: 0000 0110 | (Start=1,SGL=1) plus D2
    // Byte 1: D1 D0 xxxx xxxx
    // Byte 2: don't care (clocks out result)
    uint8_t tx[3];
    uint8_t rx[3] = {0};

    tx[0] = 0x06 | ((channel & 0x04) >> 2); // 0b00000110 plus D2
    tx[1] = (channel & 0x03) << 6;          // D1,D0 in bits 7,6
    tx[2] = 0x00;

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 3,
        .speed_hz = SPI_SPEED_DEFAULT,
        .delay_usecs = 0,
        .bits_per_word = SPI_BITS,
    };

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        perror("SPI transfer failed");
        return -1;
    }

    // 12-bit result: [rx1 bits3..0][rx2 bits7..0]
    int value = ((rx[1] & 0x0F) << 8) | rx[2];
    return value;
}

/**
 * @brief Read multiple channels [0..num_channels-1] in sequence
 */
int adc_read_channels(int fd, int *out_values, int num_channels)
{
    if (!out_values || num_channels <= 0 || num_channels > 8) {
        fprintf(stderr, "adc_read_channels: invalid args\n");
        return -1;
    }

    for (int ch = 0; ch < num_channels; ch++) {
        int v = adc_read_channel(fd, ch);
        if (v < 0) {
            return -1;
        }
        out_values[ch] = v;
    }

    return 0;
}

/**
 * @brief Close SPI ADC device
 */
void adc_close(int fd)
{
    if (fd >= 0) {
        close(fd);
    }
}