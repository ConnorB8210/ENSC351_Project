#include "gpio.h"
#include <stdlib.h>
#include <stdio.h>

GPIO_Handle* gpio_init(const char *chip_path,
                       const unsigned int *offsets,
                       size_t num_lines,
                       enum gpiod_line_direction direction,
                       enum gpiod_line_edge edge)
{
    if (!chip_path || !offsets || num_lines == 0)
        return NULL;

    GPIO_Handle *handle = calloc(1, sizeof(GPIO_Handle));
    if (!handle) return NULL;

    handle->chip = gpiod_chip_open(chip_path);
    if (!handle->chip) {
        free(handle);
        return NULL;
    }

    struct gpiod_line_config *line_config = gpiod_line_config_new();
    struct gpiod_line_settings *line_settings = gpiod_line_settings_new();

    if (!line_config || !line_settings) {
        gpiod_chip_close(handle->chip);
        free(handle);
        return NULL;
    }

    gpiod_line_settings_set_direction(line_settings, direction);
    gpiod_line_settings_set_edge_detection(line_settings, edge);

    if (gpiod_line_config_add_line_settings(line_config, offsets, num_lines, line_settings) < 0) {
        gpiod_line_settings_free(line_settings);
        gpiod_line_config_free(line_config);
        gpiod_chip_close(handle->chip);
        free(handle);
        return NULL;
    }

    handle->request = gpiod_chip_request_lines(handle->chip, NULL, line_config);
    handle->offsets = calloc(num_lines, sizeof(unsigned int));
    for (size_t i = 0; i < num_lines; i++)
        handle->offsets[i] = offsets[i];

    handle->num_lines = num_lines;

    gpiod_line_settings_free(line_settings);
    gpiod_line_config_free(line_config);

    return handle;
}

void gpio_close(GPIO_Handle *handle)
{
    if (!handle) return;

    if (handle->request)
        gpiod_line_request_release(handle->request);
    if (handle->chip)
        gpiod_chip_close(handle->chip);
    free(handle->offsets);
    free(handle);
}

int gpio_read(GPIO_Handle *handle, unsigned int line_index)
{
    if (!handle || line_index >= handle->num_lines)
        return -1;

    return gpiod_line_request_get_value(handle->request, handle->offsets[line_index]);
}

int gpio_write(GPIO_Handle *handle, unsigned int line_index, int value)
{
    if (!handle || line_index >= handle->num_lines)
        return -1;

    return gpiod_line_request_set_value(handle->request,
                                        handle->offsets[line_index],
                                        value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
}

int gpio_read_all(GPIO_Handle *handle, int *values)
{
    if (!handle || !values)
        return -1;

    enum gpiod_line_value vals[handle->num_lines];
    if (gpiod_line_request_get_values(handle->request, vals) < 0)
        return -1;

    for (size_t i = 0; i < handle->num_lines; i++)
        values[i] = vals[i];

    return 0;
}

int gpio_write_all(GPIO_Handle *handle, const int *values)
{
    if (!handle || !values)
        return -1;

    enum gpiod_line_value vals[handle->num_lines];
    for (size_t i = 0; i < handle->num_lines; i++)
        vals[i] = values[i] ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;

    return gpiod_line_request_set_values(handle->request, vals);
}