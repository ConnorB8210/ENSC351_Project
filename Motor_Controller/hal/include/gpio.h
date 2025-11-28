#ifndef GPIO_H
#define GPIO_H

#include <gpiod.h>
#include <stdint.h>
#include <stdbool.h>

// Opaque GPIO object
typedef struct {
    struct gpiod_chip *chip;
    struct gpiod_line_request *request;
    unsigned int *offsets;
    size_t num_lines;
} GPIO_Handle;

// Initialize GPIO handle for a chip path and offsets
GPIO_Handle* gpio_init(const char *chip_path,
                       const unsigned int *offsets,
                       size_t num_lines,
                       enum gpiod_line_direction direction,
                       enum gpiod_line_edge edge);

// Release GPIO handle and free resources
void gpio_close(GPIO_Handle *handle);

// Read value of single line
int gpio_read(GPIO_Handle *handle, unsigned int line_index);

// Write value to single line
int gpio_write(GPIO_Handle *handle, unsigned int line_index, int value);

// Read multiple lines into buffer
int gpio_read_all(GPIO_Handle *handle, int *values);

// Write multiple lines from buffer
int gpio_write_all(GPIO_Handle *handle, const int *values);

#endif // GPIO_H