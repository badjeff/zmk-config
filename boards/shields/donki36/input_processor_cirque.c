/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_cirque

#include <stdlib.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/keymap.h>

enum cirque_gesture_mode {
    CIRQUE_GESTURE_NONE = 0,
    CIRQUE_GESTURE_CURSOR,
    CIRQUE_GESTURE_SCROLL,
};

struct zip_input_processor_cirque_config {
    uint32_t edge_scroll_margin_h_pct;
    uint32_t edge_scroll_margin_v_pct;
    uint32_t edge_scroll_segment;
    uint32_t session_end_idle_ms;
    uint32_t sync_rel_xy_ms;
    uint32_t sync_rel_v_ms;
};

struct zip_input_processor_cirque_data {
    const struct device *dev;
    int64_t last_sync_rel_xy_ts;
    int64_t last_sync_rel_v_ts;
    int16_t sx, sy, sz; // sampling coord
    int16_t hx, hy;     // historial coord
    int16_t hs;         // historial segment coord
    int16_t dx, dy;     // cursor pending delta
    int16_t dv;         // scroll wheel pending delta
    enum cirque_gesture_mode gesture_mode;
    struct k_work_delayable session_end_work;
    int64_t gesture_start_ts;
    int64_t gesture_last_ts;
    int16_t gx, gy;     // gesture start coord
};

/* X-axis value range: [0..2047] (left to right) */
/* Y-axis value range: [0..1535] (top to bottom) */
/* Z-axis value range: [0..63] (hover to touch down) */
/* Circle center for Cirque trackpad (2047 x 1535 range) */
static const int16_t cx = 1024;
static const int16_t cy = 768;
static const float pi = 3.14159265358979323846f;

static float cirque_ellipse_dist(const struct zip_input_processor_cirque_config *config,
                                   int16_t x, int16_t y) {
    /*
     * Touch pad is a perfect circle but coordinates are reported in a rectangle.
     * The circular coordination of the center region maps to an ellipse in rectangular coordinates.
     * Detect center region by radius distance to the center of the circle touch pad
     * using an elliptical boundary; horizontal/vertical margins are percentages.
     */
    float margin_h_pct = (float)config->edge_scroll_margin_h_pct;
    float margin_v_pct = (float)config->edge_scroll_margin_v_pct;
    float a = MAX(1.0f, (float)cx * (1.0f - margin_h_pct * 0.01f));
    float b = MAX(1.0f, (float)cy * (1.0f - margin_v_pct * 0.01f));
    int16_t rx = x - cx;
    int16_t ry = y - cy;
    return ((float)rx * (float)rx) / (a * a) + ((float)ry * (float)ry) / (b * b);
}

static void cirque_session_end_work(struct k_work *work) {
    struct k_work_delayable *d_work = k_work_delayable_from_work(work);
    struct zip_input_processor_cirque_data *data =
        CONTAINER_OF(d_work, struct zip_input_processor_cirque_data, session_end_work);

    bool is_a_tap = false;
    if (data->gesture_mode != CIRQUE_GESTURE_NONE) {
        int64_t gt = data->gesture_last_ts - data->gesture_start_ts;
        int16_t gx = abs(data->gx - data->hx);
        int16_t gy = abs(data->gy - data->hy);
        LOG_DBG("TAP? gt: %lld, gx: %d, gy: %d", gt, gx, gy);
        is_a_tap = (gt < 139 && gx < 31 && gy < 31);
    }

    data->gesture_mode = CIRQUE_GESTURE_NONE;
    data->hx = data->hy = data->hs = -1;
    // LOG_DBG("");

    if (is_a_tap) {
        const struct device *dev = data->dev;
        const struct zip_input_processor_cirque_config *config = dev->config;
        float d = cirque_ellipse_dist(config, data->gx, data->gy);
        uint16_t btn = (d <= 1.0f) ? INPUT_BTN_TOUCH : INPUT_BTN_1;
        LOG_DBG("TAP: %f", d);
        input_report_key(data->dev, btn, true, true, K_FOREVER);
        k_sleep(K_MSEC(21));
        input_report_key(data->dev, btn, false, true, K_FOREVER);
    }
}

static int sy_handle_event(const struct device *dev, struct input_event *event, uint32_t param1,
                           uint32_t param2, struct zmk_input_processor_state *state) {

    const struct zip_input_processor_cirque_config *config = dev->config;
    struct zip_input_processor_cirque_data *data = dev->data;

    /* fetch all axis values from input sub-sys */
    if (event->code == INPUT_ABS_X) data->sx = event->value;
    else if (event->code == INPUT_ABS_Y) data->sy = event->value;
    else if (event->code == INPUT_ABS_Z) data->sz = event->value;

    int64_t now = k_uptime_get();

    /* on all axis value ready */
    if (event->sync) {
        // LOG_DBG("Z: %d, X: %d, Y: %d", data->sz, data->sx, data->sy);

        if (data->gesture_mode == CIRQUE_GESTURE_NONE) {
            float d = cirque_ellipse_dist(config, data->sx, data->sy);
            // LOG_DBG("d: %f, X: %d, Y: %d", d, data->sx, data->sy);
            if (d <= 1.0f) {
                /* touch on center region */
                data->gesture_mode = CIRQUE_GESTURE_CURSOR;
                // LOG_DBG("CURSOR");
            } else {
                /* touch on edge region (outside elliptical center region) */
                data->gesture_mode = CIRQUE_GESTURE_SCROLL;
                LOG_DBG("SCROLL");
            }
            if (data->gesture_mode != CIRQUE_GESTURE_NONE) {
                data->gesture_start_ts = now;
                data->gx = data->sx;
                data->gy = data->sy;
            }
        }

        int16_t x = 0;
        int16_t y = 0;

        if (data->gesture_mode == CIRQUE_GESTURE_CURSOR) {
            /* convert to relative input */
            x = data->sx - (data->hx >= 0 ? data->hx : data->sx);
            y = data->sy - (data->hy >= 0 ? data->hy : data->sy);
            data->hx = data->sx;
            data->hy = data->sy;
            data->dx += x;
            data->dy += y;
            data->gesture_last_ts = now;
            k_work_reschedule(&data->session_end_work, K_MSEC(config->session_end_idle_ms));
        }
        else if (data->gesture_mode == CIRQUE_GESTURE_SCROLL) {
            /* convert to wheel input */
            x = data->sx - (data->hx >= 0 ? data->hx : data->sx);
            y = data->sy - (data->hy >= 0 ? data->hy : data->sy);

            /* Convert tangential movement around circle to scroll */
            int16_t v = 0;

            /* Split circle into segments */
            int n_sgmt = (int)config->edge_scroll_segment;
            if (n_sgmt >= 2) {

                /* Vector from center to current finger position */
                int16_t rx = data->sx - cx;
                int16_t ry = data->sy - cy;

                float angle = atan2f((float)ry, (float)rx);
                if (angle < 0.0f) {
                    angle += 2.0f * pi;
                }
                float sgmt_angle = (2.0f * pi) / (float)n_sgmt;
                int sgmt_idx = (int)(angle / sgmt_angle) % n_sgmt;

                if (data->hs != sgmt_idx) {
                    int diff = sgmt_idx - data->hs;
                    if      (diff > n_sgmt / 2)  diff -= n_sgmt;
                    else if (diff < -n_sgmt / 2) diff += n_sgmt;

                    // ignore first touch in scroll session
                    if (data->hs != -1) {
                        v += diff > 0 ? -1 : 1;
                    }

                    data->hs = sgmt_idx;
                    LOG_DBG("seg: %d, v: %d", sgmt_idx, v);
                }
            }
            /* fallback to swipe up/down */
            else {
                int16_t tangential = x - y;
                if      (tangential > 3)    { v = 1; }
                else if (tangential < -3)   { v = -1; }
                else                        { v = 0; }
            }

            data->hx = data->sx;
            data->hy = data->sy;
            data->dv += v;
            data->gesture_last_ts = now;
            k_work_reschedule(&data->session_end_work, K_MSEC(config->session_end_idle_ms));
        }
        else if (data->gesture_mode == CIRQUE_GESTURE_NONE) {
            data->hx = data->hy = data->hs = -1;
        }

    } /* if (event->sync) */
    else {
        event->value = 0;
        event->sync = false;
        return ZMK_INPUT_PROC_STOP; // 1;
    }

    if (now - data->last_sync_rel_xy_ts > config->sync_rel_xy_ms) {
        bool have_dx = data->dx != 0;
        bool have_dy = data->dy != 0;
        if (have_dx || have_dy) {
            // LOG_DBG("x: %d, y: %d", data->dx, data->dy);
            data->last_sync_rel_xy_ts = now;
            if (have_dx) {
                input_report(dev, INPUT_EV_REL, INPUT_REL_X, data->dx, 
                    !have_dy, K_NO_WAIT);
            }
            if (have_dy) {
                input_report(dev, INPUT_EV_REL, INPUT_REL_Y, data->dy, 
                    true, K_NO_WAIT);
            }
            data->dx = data->dy = 0;
        }
    }

    if (now - data->last_sync_rel_v_ts > config->sync_rel_v_ms) {
        bool have_dv = data->dv != 0;
        if (have_dv) {
            // LOG_DBG("v: %d", data->dv > 0 ? 1 : -1);
            data->last_sync_rel_v_ts = now;
            input_report(dev, INPUT_EV_REL, INPUT_REL_WHEEL, 
                data->dv > 0 ? 1 : -1, true, K_NO_WAIT);
            data->dv = 0;
        }
    }

    event->value = 0;
    event->sync = false;
    return ZMK_INPUT_PROC_STOP; // 1;
}

static struct zmk_input_processor_driver_api sy_driver_api = {
    .handle_event = sy_handle_event,
};

static int sy_init(const struct device *dev) {
    // const struct zip_input_processor_cirque_config *config = dev->config;
    struct zip_input_processor_cirque_data *data = dev->data;
    data->dev = dev;
    data->hx = data->hy = data->hs = -1;
    data->dx = data->dy = data->dv = 0;
    data->gesture_mode = CIRQUE_GESTURE_NONE;
    k_work_init_delayable(&data->session_end_work, cirque_session_end_work);
    return 0;
}

#define TL_INST(n)                                                                        \
    static struct zip_input_processor_cirque_data data_##n = {};                          \
    static struct zip_input_processor_cirque_config config_##n = {                        \
        .edge_scroll_margin_h_pct = DT_INST_PROP(n, edge_scroll_margin_h_pct),            \
        .edge_scroll_margin_v_pct = DT_INST_PROP(n, edge_scroll_margin_v_pct),            \
        .edge_scroll_segment = DT_INST_PROP(n, edge_scroll_segment),                      \
        .session_end_idle_ms = DT_INST_PROP(n, session_end_idle_ms),                      \
        .sync_rel_xy_ms = DT_INST_PROP(n, sync_rel_xy_ms),                                \
        .sync_rel_v_ms = DT_INST_PROP(n, sync_rel_v_ms),                                  \
    };                                                                                    \
    DEVICE_DT_INST_DEFINE(n, &sy_init, NULL, &data_##n, &config_##n, POST_KERNEL,         \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &sy_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TL_INST)
