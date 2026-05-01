/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_ping

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <drivers/input_processor.h>
#include <zephyr/input/input_analog_axis_hires.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/keymap.h>

struct zip_input_processor_ping_config {
    const struct device *axh_dev;
};

struct zip_input_processor_ping_data {
    // const struct device *dev;
};

static int zip_handle_event(const struct device *dev, struct input_event *event, 
                            uint32_t param1, uint32_t param2, 
                            struct zmk_input_processor_state *state) {

    const struct zip_input_processor_ping_config *config = dev->config;
    // struct zip_input_processor_ping_data *data = dev->data;

    if (event->value == 1) {
        const struct device *axh_dev = config->axh_dev;
        struct sensor_value val = { .val1 = 0, .val2 = 0 };
        int err = sensor_attr_set(axh_dev, SENSOR_CHAN_ALL, 
                              (enum sensor_attribute) param1, &val);
        if (err) {
            LOG_WRN("Fail to sensor_attr_set");
            return -EIO;
        }
    }

    event->value = 0;
    event->sync = false;

    // return ZMK_INPUT_PROC_CONTINUE; // 0;
    return ZMK_INPUT_PROC_STOP; // 1;
}

static struct zmk_input_processor_driver_api zip_driver_api = {
    .handle_event = zip_handle_event,
};

static int sy_init(const struct device *dev) {
    ARG_UNUSED(dev);
    // const struct zip_input_processor_ping_config *config = dev->config;
    // struct zip_input_processor_ping_data *data = dev->data;
    // data->dev = dev;
    return 0;
}

#define ZIP_PING_INST(n)                                                     \
    static struct zip_input_processor_ping_data data_##n = {};               \
    static struct zip_input_processor_ping_config config_##n = {             \
        .axh_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, analog_axis_hires_dev)), \
    };                                                                       \
    DEVICE_DT_INST_DEFINE(n, &sy_init, NULL, &data_##n, &config_##n,         \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY,           \
                          &zip_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ZIP_PING_INST)
