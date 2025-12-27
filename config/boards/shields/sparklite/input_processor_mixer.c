/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_mixer

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>

// #include <dt-bindings/zmk/input_mixer.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/keymap.h>

struct zip_input_processor_mixer_config {
    uint32_t sync_report_ms;
    uint32_t sync_report_yaw_ms;
    uint32_t yaw_div;
    uint32_t yaw_equator_threshold;
    uint32_t yaw_min_per_tick;
    uint32_t yaw_val_per_tick;
};

struct zip_input_processor_mixer_data {
    const struct device *dev;
    int64_t last_rpt_time;
    int16_t x;
    int16_t y;
    int64_t last_rpt_time_yaw;
    int16_t yaw1;
    int16_t yaw2;
};

static int sy_handle_event(const struct device *dev, struct input_event *event, uint32_t param1,
                           uint32_t param2, struct zmk_input_processor_state *state) {

    const struct zip_input_processor_mixer_config *config = dev->config;
    struct zip_input_processor_mixer_data *data = dev->data;

    #define IDX_SEN_X 1
    #define IDX_SEN_Y 0

    if (param1 == IDX_SEN_X && event->code == INPUT_REL_X) {
        data->x += event->value;
    }
    if (param1 == IDX_SEN_Y && event->code == INPUT_REL_Y) {
        data->y += event->value;
    }

    // accumulate twisting distance from 2 sensors,
    if (param1 == IDX_SEN_X && event->code == INPUT_REL_Y) {
        data->yaw1 += event->value;
    }
    if (param1 == IDX_SEN_Y && event->code == INPUT_REL_X) {
        data->yaw2 += event->value;
    }

    event->value = 0;
    event->sync = false;

    int64_t now = k_uptime_get();

    bool yawing = false;

    if (now - data->last_rpt_time_yaw > config->sync_report_yaw_ms) {
        int16_t yaw1 = data->yaw1;
        int16_t yaw2 = -data->yaw2;

        int16_t ydiff = abs( abs(yaw1) - abs(yaw2) );
        // LOG_DBG("yaw1: %d, yaw2: %d", yaw1, yaw2, ydiff);

        int16_t yaw = ((yaw1 + yaw2) * 0.5) / config->yaw_div;
        if (ydiff > config->yaw_equator_threshold) {
            data->yaw1 = data->yaw2 = 0;
            yaw = 0;
        }
        // LOG_WRN("###### yaw: %d", yaw);

        if (yaw) {
            yawing = true;
            if (abs(yaw) > config->yaw_min_per_tick) {
                int16_t w = (yaw > 0 ? 1 : -1) * config->yaw_val_per_tick;
                input_report(dev, INPUT_EV_REL, INPUT_REL_WHEEL, w, true, K_NO_WAIT);
            }
            data->yaw1 = data->yaw2 = 0;
            data->last_rpt_time_yaw = now;
        }
    }

    if (yawing) {
        data->x = data->y = 0;
        return ZMK_INPUT_PROC_STOP; // 1;
    }

    if (now - data->last_rpt_time > config->sync_report_ms) {
        bool have_x = data->x != 0;
        bool have_y = data->y != 0;

        if (have_x || have_y) {
            // LOG_DBG("x: %d, y: %d", data->x, data->y);
            data->last_rpt_time = now;

            if (have_x) {
                input_report(dev, INPUT_EV_REL, INPUT_REL_X, data->x, !have_y, K_NO_WAIT);
            }
            if (have_y) {
                input_report(dev, INPUT_EV_REL, INPUT_REL_Y, data->y, true, K_NO_WAIT);
            }
            data->x = data->y = 0;
        }
    }

    // return ZMK_INPUT_PROC_CONTINUE; // 0;
    return ZMK_INPUT_PROC_STOP; // 1;
}

static struct zmk_input_processor_driver_api sy_driver_api = {
    .handle_event = sy_handle_event,
};

static int sy_init(const struct device *dev) {
    const struct zip_input_processor_mixer_config *config = dev->config;
    struct zip_input_processor_mixer_data *data = dev->data;
    data->dev = dev;
    return 0;
}

#define TL_INST(n)                                                                                 \
    static struct zip_input_processor_mixer_data data_##n = {};                                    \
    static struct zip_input_processor_mixer_config config_##n = {                                  \
        .sync_report_ms = DT_INST_PROP(n, sync_report_ms),                                         \
        .sync_report_yaw_ms = DT_INST_PROP(n, sync_report_yaw_ms),                                 \
        .yaw_div = DT_INST_PROP(n, yaw_div),                                                       \
        .yaw_equator_threshold = DT_INST_PROP(n, yaw_equator_threshold),                           \
        .yaw_min_per_tick = DT_INST_PROP(n, yaw_min_per_tick),                                     \
        .yaw_val_per_tick = DT_INST_PROP(n, yaw_val_per_tick),                                     \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, &sy_init, NULL, &data_##n, &config_##n, POST_KERNEL,                  \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &sy_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TL_INST)
